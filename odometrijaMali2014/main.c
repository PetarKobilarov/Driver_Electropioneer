#define FCY 30000000ULL

#include "regulacija.h"
#include "can.h"
#include "pwm.h"
#include <p33FJ128MC802.h>
#include <libpic30.h>

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

void TimerInit(void)
{
    IEC0bits.T1IE = 0;      /* Disable the Timer1 interrupt */
    T1CONbits.TON = 0;      /* Disable timer1 */
    IFS0bits.T1IF = 0;      /* Clear Timer interrupt flag */

    T1CONbits.TGATE = 0;
    T1CONbits.TCKPS = 0;
    T1CONbits.TCS = 0;

    TMR1 = 0;
    PR1 = 30000;    // ide na 1ms

    IPC0bits.T1IP = 2; // prioritet prekida == 2
    IFS0bits.T1IF = 0;// Clear Timer1 Interrupt Flag
    IEC0bits.T1IE = 1;// Enable Timer1 interrupt
    T1CONbits.TON = 1;
}

void QEIinit()
{
    //konfigurisi registre:
    QEI1CONbits.POSRES=0;       //index impuls ne resetuje brojac
    QEI1CONbits.TQCS=1;         //brojac broji impulse sa QEA ulaza
    QEI1CONbits.UPDN_SRC=1;     //za to vreme QEB odredjuje smer brojanja
    QEI1CONbits.QEIM=4;         //Quadrature Encoder Interface enabled (x4 mode) with index pulse reset of position counter
    QEI1CONbits.TQCKPS=0;
  //  QEI1CONbits.SWPAB=1;

    MAX1CNT=0000;
    POS1CNT=0;

    //konfigurisi registre:
    QEI2CONbits.POSRES=0;       //index impuls ne resetuje brojac
    QEI2CONbits.TQCS=1;         //brojac broji impulse sa QEA ulaza
    QEI2CONbits.UPDN_SRC=1;     //za to vreme QEB odredjuje smer brojanja
    QEI2CONbits.QEIM=4;         //Quadrature Encoder Interface enabled (x4 mode) with index pulse reset of position counter
    QEI2CONbits.TQCKPS=0;
  //  QEI2CONbits.SWPAB=1;

    MAX2CNT=0000;
    POS2CNT=0;
}

void PortInit()
{

    TRISAbits.TRISA4=0;

    TRISBbits.TRISB8=0;
    TRISBbits.TRISB9=0;
    TRISBbits.TRISB10=0;
    TRISBbits.TRISB11=0;
    TRISBbits.TRISB12=0;
    TRISBbits.TRISB13=0;
    TRISBbits.TRISB14=0;
    TRISBbits.TRISB15=0;

    LATAbits.LATA4 = 0;

    LATBbits.LATB8 = 0;
    LATBbits.LATB9 = 0;
    LATBbits.LATB10 = 0;
    LATBbits.LATB11 = 0;
    LATBbits.LATB12 = 0;
    LATBbits.LATB13 = 0;
    LATBbits.LATB14 = 0;
    LATBbits.LATB15 = 0;
}


int main(void)
{
    int tmpX, tmpY, tmp, tmpO;
    char komanda, v, smer;

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

    __builtin_write_OSCCONL(OSCCON & 0xDF);

    RPINR14bits.QEA1R = 2;		//QEI1A na RP2
    RPINR14bits.QEB1R = 3;		//QEI1B na RP3

    RPINR16bits.QEA2R = 4;		//QEI2A na RP4
    RPINR16bits.QEB2R = 7;		//QEI2B na RP7

    CAN_init(DRIVER_IDENTIFICATOR); // inicijalizacija CAN BUS- a-> argument je adresa drajvera

    __builtin_write_OSCCONL(OSCCON | (1<<6));

    //INTCON1bits.NSTDIS = 1; // zabranjeni ugnjezdeni prekidi
    
    PortInit();
    TimerInit();
    QEIinit();
    CloseMCPWM();
    PWMinit();

    resetDriver();

    setSpeed(0x80);
    setSpeedAccel(K2);	//K2 je za 1m/s /bilo je 2
    setSpeed(50);
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
                kretanje_pravo(tmp, 0);

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
