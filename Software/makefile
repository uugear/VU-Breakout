all: debpkg

debpkg: vub vubw
	cp vub debpkg/usr/bin/vub
	cp vub.wav debpkg/etc/vub/vub.wav
	cp vubw debpkg/usr/bin/vubw
	chmod 755 debpkg/DEBIAN/postinst
	dpkg --build debpkg "vub_arm64.deb"

vub: vub.c vub.wav
	gcc vub.c -o vub -lasound -lfftw3 -lm -lsndfile -lportaudio -lpthread -lgpiod

vubw: vubw.c vub
	gcc vubw.c -o vubw `pkg-config --cflags --libs gtk+-3.0`

clean:
	rm -f *.deb
	rm -f debpkg/usr/bin/vub
	rm -f debpkg/usr/bin/vubw
	rm -f vub
	rm -f vubw
