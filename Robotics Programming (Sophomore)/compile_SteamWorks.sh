rm Objs/arduino.a Objs/*.c.o
flags="-g -Os -w -f function-sections -fdata-sections -MMD"
g++ /usr/share/arduino/hardware/arduino/pcduino/cores/arduino/wiring.c -c -I. -I/usr/share/arduino/hardware/arduino/pcduino/cores/arduino -I/usr/share/arduino/hardware/arduino/pcduino/variants/sunxi/ -o Objs/wiring.c.o -g
g++ /usr/share/arduino/hardware/arduino/pcduino/cores/arduino/wiring_digital.c -c -I. -I/usr/share/arduino/hardware/arduino/pcduino/cores/arduino -I/usr/share/arduino/hardware/arduino/pcduino/variants/sunxi/ -o Objs/wiring_digital.c.o -g
g++ /usr/share/arduino/hardware/arduino/pcduino/cores/arduino/wiring_analog.c -c -I. -I/usr/share/arduino/hardware/arduino/pcduino/cores/arduino -I/usr/share/arduino/hardware/arduino/pcduino/variants/sunxi/ -o Objs/wiring_analog.c.o -g
g++ /usr/share/arduino/hardware/arduino/pcduino/cores/arduino/platform.c -c -I. -I/usr/share/arduino/hardware/arduino/pcduino/cores/arduino -I/usr/share/arduino/hardware/arduino/pcduino/variants/sunxi/ -o Objs/platform.c.o -g

/usr/bin/ar rcs Objs/arduino.a Objs/wiring_digital.c.o Objs/wiring_analog.c.o Objs/platform.c.o Objs/wiring.c.o

g++ UdpSocketLinux.cpp -c -o Objs/UdpSocketLinux.c.o -g
g++ -I/usr/share/arduino/hardware/arduino/pcduino/variants/sunxi/ -I/usr/share/arduino/hardware/arduino/pcduino/cores/arduino/ -I/usr/include/pcduino/ -I/usr/include/opencv2 -I/usr/local/include/opencv2 -I../ -L~/opencv-mp/opencv-2.4.11/lib/ -o SteamWorks SteamWorks_threaded.c -lopencv_core -lopencv_imgproc -lopencv_highgui Objs/UdpSocketLinux.c.o Objs/arduino.a -g -lpthread

g++ watchdog.c -O2 -o watchdog.exe
