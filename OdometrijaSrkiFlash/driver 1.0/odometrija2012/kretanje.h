/* 
 * File:   kretanje.h
 * Author: mrmot
 *
 * Created on November 24, 2012, 3:05 PM
 */

#ifndef KRETANJE_H
#define	KRETANJE_H

#include "globals.h"
enum States
{
    STATUS_IDLE = 'I',
    STATUS_MOVING = 'M',
    STATUS_STUCK = 'S',
    STATUS_ROTATING = 'R',
    STATUS_ERROR = 'É'
};

void resetDriver(void);
// zadavanje pozicije
void setPosition(int X, int Y, int orientation);

void sendStatusAndPosition(void);

//zadavanje brzine i ubrzanja
void setSpeedAccel(float v);

//funkcija za stizanje u tacku (Xd, Yd)
void gotoXY(int Xd, int Yd, unsigned char krajnja_brzina, char smer);

//funkcija za kretanje pravo s trapezoidnim profilom brzine
void kretanje_pravo(int duzina, unsigned char krajnja_brzina);

//funkcija za dovodjenje robota u zeljenu apsolutnu orjentaciju
void apsolutni_ugao(int ugao);

//funkcija za okretanje oko svoje ose s trapezoidnim profilom brzine
char okret(int ugao);//ne vraca indikaciju

//funkcija za kretanje po kruznoj putanji
void kurva(long Xc, long Yc, int Fi, char smer);

void stop(void);
void setSpeed(unsigned char tmp);

enum States getStatus(void);

void forceStatus(enum States);

#endif	/* KRETANJE_H */
