#include <gtk/gtk.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <signal.h>
#include <sys/wait.h>


#define VUB_DIR "/usr/bin"

static GtkWidget *label;
static GtkWidget *button;
static gint counter = 0;
static gboolean demo_running = FALSE;
static pid_t vub_pid = -1;
static int pipe_fd[2];

gboolean update_label(gpointer data) {
    if (!demo_running) return G_SOURCE_REMOVE;
    char text[100];
    snprintf(text, sizeof(text), "Demo is running %d", counter++);
    gtk_label_set_text(GTK_LABEL(label), text);
    return G_SOURCE_CONTINUE;
}

void stop_demo() {
    if (vub_pid > 0) {
        write(pipe_fd[1], "q", 1);
        waitpid(vub_pid, NULL, 0);
        vub_pid = -1;
    }
    demo_running = FALSE;
    gtk_button_set_label(GTK_BUTTON(button), "Start");
    gtk_label_set_text(GTK_LABEL(label), "Click the button to start the demo");
}

void start_demo(GtkWidget *widget, gpointer data) {
    if (demo_running) {
        stop_demo();
        return;
    }
    
    pipe(pipe_fd);

    gchar *cmd_path = g_build_filename(VUB_DIR, "vub", NULL);
    vub_pid = fork();
    if (vub_pid == 0) {
        dup2(pipe_fd[0], STDIN_FILENO);
        close(pipe_fd[1]);
        execl(cmd_path, "vub", NULL);
        perror("Failed to start vub");
        _exit(EXIT_FAILURE);
    }
    g_free(cmd_path);

    if (vub_pid > 0) {
        demo_running = TRUE;
        counter = 0;
        gtk_button_set_label(GTK_BUTTON(button), "Stop");
        gtk_label_set_text(GTK_LABEL(label), "Demo is running 0");
        g_timeout_add_seconds(1, update_label, NULL);
    }
}

void activate(GtkApplication *app, gpointer user_data) {
    GtkWidget *window;

    window = gtk_application_window_new(app);
    gtk_window_set_title(GTK_WINDOW(window), "VU Breakout Demo");
    gtk_window_set_default_size(GTK_WINDOW(window), 300, 200);

    GtkWidget *box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 10);
    gtk_container_set_border_width(GTK_CONTAINER(box), 20);
    gtk_container_add(GTK_CONTAINER(window), box);

    label = gtk_label_new("Click the button to start the demo");
    gtk_box_pack_start(GTK_BOX(box), label, TRUE, TRUE, 0);

    button = gtk_button_new_with_label("Start");
    g_signal_connect(button, "clicked", G_CALLBACK(start_demo), NULL);
    gtk_box_pack_start(GTK_BOX(box), button, TRUE, TRUE, 0);

    gtk_widget_show_all(window);
}

int main(int argc, char **argv) {
    GtkApplication *app;
    int status;

    app = gtk_application_new("com.example.vubw", G_APPLICATION_FLAGS_NONE);
    g_signal_connect(app, "activate", G_CALLBACK(activate), NULL);

    status = g_application_run(G_APPLICATION(app), argc, argv);
    stop_demo();
    g_object_unref(app);

    return status;
}
