# MPLAB IDE generated this makefile for use with GNU make.
# Project: odometrija.mcp
# Date: Sat Mar 30 12:55:24 2013

AS = pic30-as.exe
CC = pic30-gcc.exe
LD = pic30-ld.exe
AR = pic30-ar.exe
HX = pic30-bin2hex.exe
RM = rm

odometrija.hex : odometrija.cof
	$(HX) "odometrija.cof"

odometrija.cof : main.o init.o uart.o timer.o pwm.o can.o kretanje.o
	$(CC) -mcpu=33FJ128MC802 "main.o" "init.o" "uart.o" "timer.o" "pwm.o" "can.o" "kretanje.o" -o"odometrija.cof" -Wl,-Tp33FJ128MC802.gld,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,-Map="odometrija.map",--report-mem

main.o : can.h pwm.h ../../../../../../program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ128MC802.h globals.h ../../../../../../program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ128MC802.h globals.h kretanje.h sinus.h uart.h ../../../../../../program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ128MC802.h init.h main.c
	$(CC) -mcpu=33FJ128MC802 -x c -c "main.c" -o"main.o" -D__DEBUG -g -Wall

init.o : pwm.h uart.h ../../../../../../program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ128MC802.h init.h init.c
	$(CC) -mcpu=33FJ128MC802 -x c -c "init.c" -o"init.o" -D__DEBUG -g -Wall

uart.o : ../../../../../../program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ128MC802.h uart.h uart.c
	$(CC) -mcpu=33FJ128MC802 -x c -c "uart.c" -o"uart.o" -D__DEBUG -g -Wall

timer.o : ../../../../../../program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ128MC802.h ../../../../../../program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33Fxxxx.h ../../../../../../program\ files\ (x86)/microchip/mplab\ c30/support/peripheral_30F_24H_33F/timer.h ../../../../../../program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ128MC802.h timer.c
	$(CC) -mcpu=33FJ128MC802 -x c -c "timer.c" -o"timer.o" -D__DEBUG -g -Wall

pwm.o : ../../../../../../program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ128MC802.h pwm.c
	$(CC) -mcpu=33FJ128MC802 -x c -c "pwm.c" -o"pwm.o" -D__DEBUG -g -Wall

can.o : can.h ../../../../../../program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ128MC802.h ../../../../../../program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33fxxxx.h ../../../../../../program\ files\ (x86)/microchip/mplab\ c30/support/generic/h/libpic30.h can.c
	$(CC) -mcpu=33FJ128MC802 -x c -c "can.c" -o"can.o" -D__DEBUG -g -Wall

kretanje.o : ../../../../../../program\ files\ (x86)/microchip/mplab\ c30/include/math.h can.h pwm.h uart.h ../../../../../../program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ128MC802.h globals.h kretanje.h ../../../../../../program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC33F/h/p33FJ128MC802.h globals.h kretanje.c
	$(CC) -mcpu=33FJ128MC802 -x c -c "kretanje.c" -o"kretanje.o" -D__DEBUG -g -Wall

clean : 
	$(RM) "main.o" "init.o" "uart.o" "timer.o" "pwm.o" "can.o" "kretanje.o" "odometrija.cof" "odometrija.hex"

