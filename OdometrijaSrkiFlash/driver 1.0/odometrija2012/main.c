#define FCY	30000000ULL
#include <libpic30.h>
#include <xc.h>
#include "init.h"
#include "sinus.h"
#include "kretanje.h"
#include "globals.h"
#include "uart.h"
#include "pwm.h"
#include "can.h"

_FOSCSEL(FNOSC_PRI);	//primarni oscilator bez PLL -> ukljucujem ga prvo samo na primarni, kada se stabilizuje ukljucicu mu PLL
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON & POSCMD_XT & IOL1WAY_OFF);  //HS oscilator za >4Mhz (internet, navodno datasheet).. XT-> 
//kompromis izmedju brzine i potrosnje, u sustini isto kao HS, koji se fokusira na brzinu!
// u application notu pise DA JE ZA F > 10MHz HS, ZATO STOJI XT!
//OSCIOFNC_ON -> ako je oscilator u XT ili HS modu nema veze sta tu stoji.. Inace odredjuje ulogu pina!
// ako je programiran ('0') taj pin OSC2 je GPIO
// else je FCY output
// FCKSM_CSECMD -> Clock switch enabled, clock monitoring disabled

_FWDT(FWDTEN_OFF); // UBIJAM VOCDOGA
_FPOR(PWMPIN_ON &/* & BOREN_OFF & */HPOL_ON & LPOL_ON & FPWRT_PWR2 & ALTI2C_ON);
// FPWRT_PWR2 -> ?
// ALTI2C_ON-> mapiram i2c pinove na SDA/SCL

void LeviPWM(unsigned int PWM);
void DesniPWM(unsigned int PWM);

unsigned char rxData[8];
unsigned char txData[8];

//POMOCNE PROMENLJIVE:
char brint = 0;
long positionR, positionL;              //trenutne pozicije na enkoderima
int vR, vL, greska_pred, greska_pred_R; //trenutne brzine na motorima
int zaglavL, zaglavR;
unsigned char brzinaL;
long L=0, orientation=0, teta=0;
long long int Xlong=0, Ylong=0;
float vmax, accel;
float omega, alfa;
long X=0, Y=0;
long brojac,i;
unsigned long sys_time = 0;
//PROMENLJIVE POTREBNE ZA REGULACIJU
int PWML, PWMD;
long t_ref=0, d_ref=0;
float v_ref;

void test(long num)
{
unsigned char c1, c10, c100, c1000, c10000, c100000;
unsigned char buffer[8];

if(num < 0)
{
num = -num;
buffer[0] = '-';
}
else
buffer[0] = '+';

c1 = num % 10;
num = num / 10;

c10 = num % 10;
num = num / 10;

c100 = num % 10;
num = num / 10;

c1000 = num % 10;
num = num / 10;

c10000 = num % 10;
num = num / 10;

c100000 = num % 10;
num = num / 10;

buffer[1] = c100000 + '0';
buffer[2] = c10000 + '0';
buffer[3] = c1000 + '0';
buffer[4] = c100 + '0';
buffer[5] = c10 + '0';
buffer[6] = c1 + '0';

CAN_write(buffer, MAIN_BOARD_IDENTIFICATOR, 0b11);
}

void incSend(void)
{
test(positionL);
test(positionR);
}



void __attribute__((__interrupt__)) _T1Interrupt(void)
{
    long d, t;
    long sint, cost;
    long x, y;
    long greska;
    int brzina, commande_distance, commande_rotation;

    sys_time++;

    if (++brint == 5)
    {
        brint = 0;
        // **********************************************************************
        // ODOMETRIJA (ide na 5ms)
        // **********************************************************************

        //CITANJE PODATAKA O POZICIJAMA OBA MOTORA:
        vR = -(int)POS2CNT;	//ocitavamo broj inkremenata u zadnjoj periodi
        POS2CNT = 0;		//resetujemo brojac inkremenata
        positionR += vR;	//azuriramo trenutnu poziciju

        vL = (int)POS1CNT;	//ocitavamo broj inkremenata u zadnjoj periodi
        POS1CNT = 0;		//resetujemo brojac inkremenata
        positionL += vL;	//azuriramo trenutnu poziciju
		
        L = (positionR + positionL) / 2;
        orientation = (positionR - positionL) % K1;
        if(orientation > K1/2)
            orientation -= K1;
        if(orientation < -K1/2)
            orientation += K1;
        
        teta = (orientation * 16384) / (K1 / 2);
		
	
        if (teta < 0)
        	teta += 32768;

        d = vR + vL;	//ovo se kasnije deli sa 2
        if(teta < 8192)
        {
            t = teta;
            sint = sinus[t];
            cost = sinus[8191 - t];
        }
        else
            if (teta < 16384)
            {
                t = teta - 8192;
                sint = sinus[8191 - t];
                cost = -sinus[t];
            }
            else
                if (teta < 24576)
                {
                    t = teta - 16384;
                    sint = -sinus[t];
                    cost = -sinus[8191 - t];
                }
                else
                {
                    t = teta - 24576;
                    sint = -sinus[8191 - t];
                    cost = sinus[t];
                }

        x = d * cost;
        y = d * sint;

        Xlong += x;
        Ylong += y;

        X = ((long long)Xlong) >> 16;
        X /= K2;
        Y = ((long long)Ylong) >> 16;
        Y /= K2;

        //regulator rastojanja
        brzina = (vL + vR) / 2;
        greska = d_ref - L;
        zaglavL = greska >= 0 ? greska : -greska;

        commande_distance = greska * Gp_D - Gd_D * brzina;

        //regulator orjentacije
        brzina = vL - vR;
        greska = (orientation - t_ref) % K1;
        if (greska > K1/2)
            greska -= K1;
        if (greska < -K1/2)
            greska += K1;

        zaglavR = greska >= 0 ? greska : -greska;
        commande_rotation = greska * Gp_T - brzina * Gd_T;

        PWML = commande_distance - commande_rotation;
        PWMD = commande_distance + commande_rotation;
		
		// saturacija
   	if(PWMD <= -3200)
            PWMD = -3200;
        else if(PWMD >= 3200)
            PWMD = 3200;
   	if(PWML <= -3200)
            PWML = -3200;
   	else if(PWML >= 3200)
            PWML = 3200;
		
        //izbor smera:
        if(PWML >= 0)
        {
                LEVI_NAPRED;
                LeviPWM(PWML);
        }
        else
        {
                LEVI_NAZAD;
                LeviPWM(-PWML);
        }
        if (PWMD >= 0)
        {
                DESNI_NAPRED;
                DesniPWM(PWMD);
        }
        else
        {
                DESNI_NAZAD;
                DesniPWM(-PWMD);
        }
    }	//KRAJ REGULACIJE

    IFS0bits.T1IF = 0;    /* Clear Timer interrupt flag */
}


void ResetDriver()
{
    //inicijalizacija parametara:
    positionR = positionL=0;
    L = orientation = 0;
    vR = vL = 0;
}

void LeviPWM(unsigned int PWM)
{
    P1DC1 = PWM;
}

void DesniPWM(unsigned int PWM)
{
    P2DC1 = PWM;
}


int main(void)
{
	
    /* Configure Oscillator to operate the device at 30Mhz
       Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
       Fosc= 7.37*(32)/(2*2)=58.96Mhz for Fosc, Fcy = 29.48Mhz */

    /* Configure PLL prescaler, PLL postscaler, PLL divisor */
    //PLLFBDbits.PLLDIV=38;   /* M = PLLFBD + 2 */ // izlazna frekvencija = 30Mhz
    //Fin=8MHz, Fcy=30MHz 
	// Configure PLL prescaler, PLL postscaler, PLL divisor
	PLLFBD = 28; 				// M=40    ---> PLLFBD + 2 = M
	CLKDIVbits.PLLPOST = 0; 	// N2=2    ---> 2x(PLLPOST + 2) = N2
	CLKDIVbits.PLLPRE = 0; 	// N1=2    ---> PLLPRE + 2 = N1

	//new oscillator selection
	__builtin_write_OSCCONH(0b011);  				//0b011 ---> XT with PLL
	//enable oscillator source switch
	__builtin_write_OSCCONL (OSCCONL | (1<<0)); 	//OSWEN 

	//wait for PLL lock -> wait to new settings become available
	while (OSCCONbits.COSC != 0b011); 
	//wait for PLL lock
	while (OSCCONbits.LOCK != 0b1); 
    
    AD1PCFGL = 0xFFFF;// all PORT Digital

   
    RPINR18bits.U1RXR = 0;		//UART1 RX na RP0- pin 4
    RPOR0bits.RP1R = 3;			//UART1 TX na RP1- pin 5
    RPINR14bits.QEA1R = 2;		//QEI1A na RP2
    RPINR14bits.QEB1R = 3;		//QEI1B na RP3

    RPINR16bits.QEA2R = 4;		//QEI2A na RP4
    RPINR16bits.QEB2R = 7;		//QEI2B na RP7
    
    CAN_init(DRIVER_IDENTIFICATOR); // inicijalizacija CAN BUS- a-> argument je adresa drajvera

    int tmp;
    char komanda, v, smer;
    int Xc, Yc, ugao;
    
    NewLine();

    PortInit();
    //UARTinit();
    TimerInit();
    QEIinit();
    PWMinit();
   // CloseMCPWM();

    resetDriver();

    setSpeed(0x80);
    setSpeedAccel(K2);	//K2 je za 1m/s /bilo je 2
    int tmpX, tmpY, tmpO;
    unsigned char rxBuffer[8];
    while(1)
    {
        if(getStatus() == STATUS_MOVING)
            CAN_getLastMessage(rxBuffer);
        else
            CAN_read(rxBuffer);

        komanda = rxBuffer[0];

        switch(komanda)
        {
            // zadavanje pozicije
            case 'I':
                tmpX = rxBuffer[1] << 8;
                tmpX |= rxBuffer[2];

                tmpY = rxBuffer[3] << 8;
                tmpY |= rxBuffer[4];

                tmpO = rxBuffer[5] << 8;
                tmpO |= rxBuffer[6];

                setPosition(tmpX, tmpY, tmpO);

                break;

            // citanje pozicije i statusa
            case 'P':
                sendStatusAndPosition();

                break;

            //zadavanje max. brzine (default K2/2)
            case 'V':
                tmp = rxBuffer[1];
                setSpeed(tmp);

                break;

            //kretanje pravo [mm]
            case 'D':
                tmp = rxBuffer[1] << 8;
                tmp |= rxBuffer[2];
                v = rxBuffer[3];

                PWMinit();
                kretanje_pravo(tmp, v);

                break;

            //relativni ugao [stepen]
            case 'T':
                tmp = rxBuffer[1] << 8;
                tmp |= rxBuffer[2];

                PWMinit();
                okret(tmp);

                break;

            //apsolutni ugao [stepen]
            case 'A':
                tmp = rxBuffer[1] << 8;
                tmp |= rxBuffer[2];

                PWMinit();
                apsolutni_ugao(tmp);

                break;

            //idi u tacku (Xc, Yc) [mm]
            case 'G':
                tmpX = rxBuffer[1] << 8;
                tmpX |= rxBuffer[2];
                tmpY = rxBuffer[3] << 8;
                tmpY |= rxBuffer[4];
                v = rxBuffer[5];
                smer = rxBuffer[6];

                PWMinit();
                gotoXY(tmpX, tmpY, v, smer);

                break;

            //kurva
            case 'Q':
                tmpX = rxBuffer[1] << 8;
                tmpX |= rxBuffer[2];
                tmpY = rxBuffer[3] << 8;
                tmpY |= rxBuffer[4];
                tmpO = rxBuffer[5] << 8;
                tmpO |= rxBuffer[6];
                smer = rxBuffer[7];

                PWMinit();
                kurva(tmpX, tmpY, tmpO, smer);

                break;

             //ukopaj se u mestu
            case 'S':
                stop();

                break;

            //stani i ugasi PWM
            case 's':
                stop();
                CloseMCPWM();

                break;

            case 'N':
                incSend();

                break;

            default:
                forceStatus(STATUS_ERROR);
                break;
        }
    }

    return 0;
}
