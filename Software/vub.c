#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <sys/file.h>
#include <gpiod.h>
#include <termios.h>
#include <signal.h>
#include <unistd.h>
#include <sndfile.h>
#include <portaudio.h>
#include <alsa/asoundlib.h>
#include <fftw3.h>
#include <math.h>

#define I2C_ADDR                      0x0A
#define I2C_CONF_WATCHDOG_THRESHOLD   20
#define I2C_LOCK                      "/var/lock/vivid_unit_extender_i2c.lock"

#define SAMPLE_RATE     44100
#define BUFFER_SIZE     1024
#define FREQ_DIVIDER    18

#define LED_ROWS        2
#define LED_COLUMNS     14

#define WAV_FILE        "/etc/vub/vub.wav"
#define WAV_VOLUME      1.0f

#define PMUGRF  0xff320000
#define GRF     0xff770000

#define COMMAND_BUFFER_SIZE 64
#define OUTPUT_BUFFER_SIZE  128

typedef struct {
    int row;
    int col;
    unsigned long us;
    char fade_in;
} LedFading;

static const int GPIO_IOMUX[5][4] = {
    {0x00000, 0x00004, -1, -1},           // PMUGRF_GPIO0*
    {0x00010, 0x00014, 0x00018, 0x0001c}, // PMUGRF_GPIO1*
    {0x0e000, 0x0e004, 0x0e008, 0x0e00c}, // GRF_GPIO2*
    {0x0e010, 0x0e014, 0x0e018, 0x0e01c}, // GRF_GPIO3*
    {0x0e020, 0x0e024, 0x0e028, 0x0e02c}, // GRF_GPIO4*
};

static const char * LEDS[LED_ROWS][LED_COLUMNS] = {
    { "2D3", "2B4", "2B3", "2B2", "2B1", "2B0", "2A7", "2A6", "2A5", "2A4", "2A3", "2A2", "2A1", "2A0" },
    { "1A1", "4D6", "4D2", "4D1", "4C4", "4C3", "4B5", "4B4", "4B3", "4B2", "4B1", "4B0", "1A4", "1A2" }
};

static struct gpiod_line *led_lines[LED_ROWS][LED_COLUMNS];

LedFading led_fadings[LED_ROWS][LED_COLUMNS];

pthread_t fade_threads[LED_ROWS][LED_COLUMNS];

float sensitivity = 100.0f;

volatile sig_atomic_t keep_running = 1;

volatile char wav_started = 0;

char command_buffer[COMMAND_BUFFER_SIZE];

char output_buffer[OUTPUT_BUFFER_SIZE];

int wd_bak = -1;

int lock_file() {
    int lock_fd = open(I2C_LOCK, O_CREAT | O_RDWR, 0666);
    if (lock_fd < 0) {
        perror("Failed to open lock file");
        return -1;
    }
    if (flock(lock_fd, LOCK_EX) < 0) {
        perror("Failed to acquire lock");
        close(lock_fd);
        return -1;
    }
    return lock_fd;
}

void unlock_file(int lock_fd) {
    flock(lock_fd, LOCK_UN);
    close(lock_fd);
}

void restart_with_sudo(int argc, char *argv[]) {
    char *new_argv[argc + 2];
    new_argv[0] = "sudo";
    for (int i = 0; i < argc; i++) {
        new_argv[i + 1] = argv[i];
    }
    new_argv[argc + 1] = NULL;
    execvp("sudo", (char*[]){"sudo", "-E", argv[0], NULL});
    perror("Failed to restart with sudo");
    exit(1);
}

int run_command(const char * cmd) {
  FILE *fp;
  if ((fp = popen(cmd, "r")) == NULL) {
      return -1;
  }
  int length = 0;
  if (fgets(output_buffer, OUTPUT_BUFFER_SIZE, fp) != NULL) {
      length = strlen(output_buffer);
  }
  if (pclose(fp)) {
      return -2;
  }
  return length;
}

int get_register(unsigned int address) {
    sprintf(command_buffer, "sudo io -4 -r 0x%x", address);
    int length = run_command(command_buffer);
    if (length < 12) {
      printf("get_register returned an error\n");
      return -1;
    }
    strcpy(output_buffer, &output_buffer[11]);
    return strtol(output_buffer, NULL, 16);
}

int set_register(unsigned int address, unsigned int value) {
    sprintf(command_buffer, "sudo io -4 -w 0x%x 0x%x", address, value);
    int length = run_command(command_buffer);
    if (length < 0) {
      printf("set_register returned an error\n");
    }
    return length;
}

int set_alt(int ch, int ln, int alt)
{
  int group = ln / 8;
  int index = ln % 8;
  if (alt < 0 || alt > 3)
  {
    fprintf(stderr, "Unsupported ALT value %d\n", alt);
    return -2;
  }
  int iomux = (GPIO_IOMUX[ch][group] == -1) ? -1 : get_register((ch < 2 ? PMUGRF : GRF) + GPIO_IOMUX[ch][group]);
  if (iomux != -1)
  {
    iomux &= ~(0x03 << (index << 1));
    iomux |= (alt << (index << 1));
    iomux |= (0x03 << ((index << 1) + 16));
    set_register((ch < 2 ? PMUGRF : GRF) + GPIO_IOMUX[ch][group], iomux);
    return 0;
  }
  return -1;
}

int parse_gpio(const char *gpio_str, int *chip_num, int *line_num) {
    if (strlen(gpio_str) < 3) {
        return -1;
    }

    *chip_num = gpio_str[0] - '0';

    char port = gpio_str[1];
    int pin = gpio_str[2] - '0';

    int port_base = 0;
    switch (port) {
        case 'A': port_base = 0; break;
        case 'B': port_base = 8; break;
        case 'C': port_base = 16; break;
        case 'D': port_base = 24; break;
        default: return -1;
    }
    *line_num = port_base + pin;

    return 0;
}


void init_leds() {
    for (int row = 0; row < LED_ROWS; row++) {
        for (int col = 0; col < LED_COLUMNS; col++) {
            const char *gpio_str = LEDS[row][col];
            int chip_num, line_num;

            if (parse_gpio(gpio_str, &chip_num, &line_num) < 0) {
                fprintf(stderr, "unable to parse GPIO: %s\n", gpio_str);
                continue;
            }
            
            set_alt(chip_num, line_num, 0);

            char chip_name[16];
            snprintf(chip_name, sizeof(chip_name), "/dev/gpiochip%d", chip_num);
            struct gpiod_chip *chip = gpiod_chip_open(chip_name);
            if (!chip) {
                perror("Can not open GPIO chip");
                continue;
            }

            struct gpiod_line *line = gpiod_chip_get_line(chip, line_num);
            if (!line) {
                fprintf(stderr, "Can not get GPIO line: %s\n", gpio_str);
                gpiod_chip_close(chip);
                continue;
            }

            if (gpiod_line_request_output(line, "Vivid_Unit_Breakout_Demo", 0) < 0) {
                fprintf(stderr, "Can not request output mode %s\n", gpio_str);
                gpiod_chip_close(chip);
                continue;
            }

            led_lines[row][col] = line;
        }
    }
}


void set_led_state(int row, int col, int state) {
    if (row >= LED_ROWS || col >= LED_COLUMNS || !led_lines[row][col]) {
        fprintf(stderr, "Invalid row or column: (%d, %d)\n", row, col);
        return;
    }

    struct gpiod_line *line = led_lines[row][col];
    gpiod_line_set_value(line, state ? 0 : 1);
}

void set_all_leds(int state) {
    for (int row = 0; row < LED_ROWS; row ++) {
        for (int col = 0; col < LED_COLUMNS; col ++) {
            set_led_state(row, col, state);
        }
    }
}

void random_all_leds(void) {
    for (int row = 0; row < LED_ROWS; row ++) {
        for (int col = 0; col < LED_COLUMNS; col ++) {
            set_led_state(row, col, rand()%2);
        }
    }
}

void* fade_led(void* arg) {
  LedFading * fade = (LedFading *)arg;
  int steps = 100;
  long delay = fade->us / steps;
  for (int i = steps; i >= 0; i --) {
      float duty_cycle = i / (float)steps;
      if (fade->fade_in) {
        duty_cycle = 1.0f - duty_cycle;
      }
      long on_time = duty_cycle * delay;
      long off_time = delay - on_time;
      set_led_state(fade->row, fade->col, 1);
      usleep(on_time);
      set_led_state(fade->row, fade->col, 0);
      usleep(off_time);
  }
}

void fade_all_leds(unsigned long us, char fade_in) {
    for (int row = 0; row < LED_ROWS; row ++) {
        for (int col = 0; col < LED_COLUMNS; col ++) {
            led_fadings[row][col].row = row;
            led_fadings[row][col].col = col;
            led_fadings[row][col].us = us;
            led_fadings[row][col].fade_in = fade_in;
            pthread_create(&fade_threads[row][col], NULL, fade_led, &led_fadings[row][col]);
        }
    }
    for (int row = 0; row < LED_ROWS; row ++) {
        for (int col = 0; col < LED_COLUMNS; col ++) {
          pthread_join(fade_threads[row][col], NULL);
        }
    }
}

void cleanup_leds() {
    for (int row = 0; row < LED_ROWS; row ++) {
        for (int col = 0; col < LED_COLUMNS; col ++) {
            if (led_lines[row][col]) {
                gpiod_line_release(led_lines[row][col]);
                led_lines[row][col] = NULL;
            }
        }
    }
}


void *play_wav(void *arg) {
    SF_INFO sfinfo;
    SNDFILE *sndfile = sf_open(WAV_FILE, SFM_READ, &sfinfo);
    if (!sndfile) {
        fprintf(stderr, "Could not open file: %s\n", sf_strerror(NULL));
        pthread_exit(NULL);
    }

    sensitivity = 15.0f;

    Pa_Initialize();

    PaStream *stream;
    Pa_OpenDefaultStream(&stream, 0, sfinfo.channels, paFloat32, SAMPLE_RATE, 256, NULL, NULL);

    while (!wav_started);
    
    Pa_StartStream(stream);
    
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);

    float *buffer = malloc(256 * sizeof(float) * sfinfo.channels);
    sf_count_t num_frames;
    while ((num_frames = sf_readf_float(sndfile, buffer, 256)) > 0) {
        for (int i = 0; i < num_frames * sfinfo.channels; i++) {
            buffer[i] *= WAV_VOLUME;
        }
        Pa_WriteStream(stream, buffer, num_frames);
        pthread_testcancel();
    }
    
    sensitivity = 100.0f;
    
    printf("Demo has finished.\n");
    printf("The program continues as audio spectrum display.\n");
    printf("Press q to exit.\n");

    free(buffer);
    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();
    sf_close(sndfile);
    pthread_exit(NULL);
}


void capture_audio(int16_t *buffer, snd_pcm_t *capture_handle) {
    int err = snd_pcm_readi(capture_handle, buffer, BUFFER_SIZE);
    if (err == -EPIPE) {
        snd_pcm_prepare(capture_handle);
    } else if (err < 0) {
        fprintf(stderr, "audio capture failed: %s\n", snd_strerror(err));
    }
}

void perform_fft(int16_t *buffer, fftw_complex *output) {
    fftw_plan plan;
    double *in = (double*) fftw_malloc(sizeof(double) * BUFFER_SIZE);
    for (int i = 0; i < BUFFER_SIZE; i++) {
        in[i] = buffer[i] / 32768.0;
    }
    plan = fftw_plan_dft_r2c_1d(BUFFER_SIZE, in, output, FFTW_ESTIMATE);
    fftw_execute(plan);
    fftw_destroy_plan(plan);
    fftw_free(in);
}

void display_fft(double low_freq_magnitude, double high_freq_magnitude) {
    int low_band_leds = (int)(low_freq_magnitude * LED_COLUMNS);
    int high_band_leds = (int)(high_freq_magnitude * LED_COLUMNS);
    if (low_band_leds > LED_COLUMNS) {
        low_band_leds = LED_COLUMNS;
    }
    if (high_band_leds > LED_COLUMNS) {
        high_band_leds = LED_COLUMNS;
    }
    for (int i = 0; i < LED_COLUMNS; i++) {
        set_led_state(0, i, i < low_band_leds ? 1 : 0);
        set_led_state(1, i, i < high_band_leds ? 1 : 0);
    }
}

void signal_handler(int signal) {
    if (signal == SIGINT) {
        keep_running = 0;
    }
}

void alsa_silent_error_handler(const char *file, int line, const char *function, int err, const char *fmt, ...) {
    // empty handler to suppress ALSA error messages
}

void set_noncanonical_mode(struct termios *orig_termios) {
    struct termios new_termios;
    tcgetattr(STDIN_FILENO, orig_termios);
    new_termios = *orig_termios;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
}


void restore_terminal_mode(void) {
    struct termios orig_termios;
    tcgetattr(STDIN_FILENO, &orig_termios);
    orig_termios.c_lflag |= ICANON | ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
}

void* monitor_keyboard(void* arg) {
    char ch;
    while (keep_running) {
        ch = getchar();
        if (ch == 'q' || ch == 'Q') {
            keep_running = 0;
            break;
        }
    }
    return NULL;
}

int main(int argc, char **argv) {
  
    if (geteuid() != 0) {
        restart_with_sudo(argc, argv);
        return 1;
    }
    
    signal(SIGINT, signal_handler);
   
    struct termios orig_termios;
    set_noncanonical_mode(&orig_termios);
    atexit(restore_terminal_mode);

    pthread_t keyboard_thread;
    if (pthread_create(&keyboard_thread, NULL, monitor_keyboard, NULL) != 0) {
        perror("Can not open keyboard monitor.\n");
        return 1;
    }
    
    // disable watchdog (because I2C pins will also be used as output)
    printf("Temporarily disable watchdog.\n");
    int lock_fd = lock_file();
    if (lock_fd < 0) {
        perror("Failed to lock I2C device for disabling watchdog");
        return 1;
    }
    sprintf(command_buffer, "sudo i2cget -y 2 0x%x %d", I2C_ADDR, I2C_CONF_WATCHDOG_THRESHOLD);
    run_command(command_buffer);
    if (strstr(output_buffer, "Error") == NULL) {
      wd_bak = (int)strtol(output_buffer, NULL, 16);
      sprintf(command_buffer, "sudo i2cset -y 2 0x%x %d 0", I2C_ADDR, I2C_CONF_WATCHDOG_THRESHOLD);
      run_command(command_buffer);
    }
    unlock_file(lock_fd);
    
    printf("Demo has started.\n");
    
    init_leds();
    fade_all_leds(600000, 0);
    
    snd_pcm_t *capture_handle;
    snd_pcm_hw_params_t *hw_params;

    snd_lib_error_set_handler(alsa_silent_error_handler);
    
    if (snd_pcm_open(&capture_handle, "default", SND_PCM_STREAM_CAPTURE, 0) < 0) {
        perror("Can not open sound interface.\n");
        return -1;
    }
    
    pthread_t wav_thread;
    pthread_create(&wav_thread, NULL, play_wav, NULL);
    
    usleep(500000);
    wav_started = 1;
    
    // begin: pre-defined patterns
    
    if (keep_running) fade_all_leds(500000, 1);
    if (keep_running) fade_all_leds(1500000, 0);
    if (keep_running) usleep(3500000);
    if (keep_running) set_all_leds(1);
    if (keep_running) usleep(300000);
    if (keep_running) set_all_leds(0);
    if (keep_running) usleep(2500000);
    
    for (int i = 0; keep_running && i < 180; i ++) {
        if (keep_running) random_all_leds();
        if (keep_running) usleep(30000);
    }
    if (keep_running) set_all_leds(0);
    
    if (keep_running) usleep(1400000);
        
    if (keep_running) set_led_state(0, 0, 1);
    if (keep_running) set_led_state(0, 1, 1);
    if (keep_running) set_led_state(1, 0, 1);
    if (keep_running) set_led_state(1, 1, 1);
    if (keep_running) usleep(1700000);  

    if (keep_running) set_led_state(0, 2, 1);
    if (keep_running) set_led_state(0, 3, 1);
    if (keep_running) set_led_state(1, 2, 1);
    if (keep_running) set_led_state(1, 3, 1);
    if (keep_running) usleep(1700000); 

    if (keep_running) set_led_state(0, 4, 1);
    if (keep_running) set_led_state(0, 5, 1);
    if (keep_running) set_led_state(1, 4, 1);
    if (keep_running) set_led_state(1, 5, 1);
    if (keep_running) usleep(750000);

    if (keep_running) set_led_state(0, 6, 1);
    if (keep_running) set_led_state(0, 7, 1);
    if (keep_running) set_led_state(1, 6, 1);
    if (keep_running) set_led_state(1, 7, 1);
    if (keep_running) usleep(1700000);
    
    if (keep_running) set_led_state(0, 8, 1);
    if (keep_running) set_led_state(0, 9, 1);
    if (keep_running) set_led_state(1, 8, 1);
    if (keep_running) set_led_state(1, 9, 1);
    if (keep_running) usleep(400000);

    if (keep_running) set_led_state(0, 10, 1);
    if (keep_running) set_led_state(0, 11, 1);
    if (keep_running) set_led_state(1, 10, 1);
    if (keep_running) set_led_state(1, 11, 1);
    if (keep_running) usleep(400000);

    if (keep_running) set_led_state(0, 12, 1);
    if (keep_running) set_led_state(0, 13, 1);
    if (keep_running) set_led_state(1, 12, 1);
    if (keep_running) set_led_state(1, 13, 1);
    if (keep_running) usleep(1000000);
    
    if (keep_running) fade_all_leds(1000000, 0);
        
    if (keep_running) usleep(3500000);
    
    // end: pre-defined patterns

    snd_pcm_hw_params_malloc(&hw_params);
    snd_pcm_hw_params_any(capture_handle, hw_params);
    snd_pcm_hw_params_set_access(capture_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(capture_handle, hw_params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_rate(capture_handle, hw_params, SAMPLE_RATE, 0);
    snd_pcm_hw_params_set_channels(capture_handle, hw_params, 1);
    snd_pcm_hw_params(capture_handle, hw_params);
    snd_pcm_hw_params_free(hw_params);
    snd_pcm_prepare(capture_handle);

    int16_t buffer[BUFFER_SIZE];
    fftw_complex *output = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (BUFFER_SIZE / 2 + 1));

    while (keep_running) {
        capture_audio(buffer, capture_handle);
        perform_fft(buffer, output);

        double low_freq_magnitude = 0.0;
        double high_freq_magnitude = 0.0;

        for (int i = 0; i < BUFFER_SIZE / 2; i++) {
            double magnitude = sqrt(output[i][0] * output[i][0] + output[i][1] * output[i][1]);
            if (i < BUFFER_SIZE / FREQ_DIVIDER) {
                low_freq_magnitude += magnitude;
            } else {
                high_freq_magnitude += magnitude;
            }
        }

        low_freq_magnitude /= (BUFFER_SIZE / sensitivity);
        high_freq_magnitude /= (BUFFER_SIZE / sensitivity);

        display_fft(low_freq_magnitude, high_freq_magnitude);

        usleep(500);
    }
    
    pthread_join(keyboard_thread, NULL);
    
    pthread_cancel(wav_thread);
    pthread_join(wav_thread, NULL);

    fftw_free(output);
    snd_pcm_close(capture_handle);
    
    cleanup_leds();
    
    // restore I2C interface
    printf("Restore I2C interface.\n");
    int chip_num, line_num;
    parse_gpio("2A0", &chip_num, &line_num);
    set_alt(chip_num, line_num, 2);
    parse_gpio("2A1", &chip_num, &line_num);
    set_alt(chip_num, line_num, 2);
    usleep(5000);

    // restore watchdog settings
    if (wd_bak > 0) {
      printf("Restore watchdog settings.\n");
      int lock_fd = lock_file();
      if (lock_fd >= 0) {
          sprintf(command_buffer, "sudo i2cset -y 2 0x%x %d %d", I2C_ADDR, I2C_CONF_WATCHDOG_THRESHOLD, wd_bak);
          run_command(command_buffer);
          unlock_file(lock_fd);
      } else {
          perror("Failed to lock I2C device for restoring watchdog settings");
      }      
    }

    return 0;
}
