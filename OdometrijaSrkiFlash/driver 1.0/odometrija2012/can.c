#define FCY	30000000ULL

#include <libpic30.h>
#include <xc.h>
#include "can.h"


static unsigned int buff[4][8]  __attribute__((space(dma), alligned(128)));
static volatile unsigned char rxBuffer[RX_BUFFER_SIZE][8];
static volatile unsigned char receivedData[8];
static volatile unsigned char rxCounter = 0;
static volatile unsigned char rxRdIndex = 0;
static volatile unsigned char rxWrIndex = 0;

unsigned char CAN_checkRX(void)
{
    return rxCounter;
}


void CAN_write(unsigned char *data, unsigned int targetAddress, unsigned char priority)
{	
	while(C1TR01CONbits.TXREQ0 == 0b1);
	__delay_us(200);
	C1TR01CONbits.TXEN0 = 1;
	C1TR01CONbits.TX0PRI = priority; //
	buff[0][0] = targetAddress << 2;
	buff[0][1] = 0;
	buff[0][2] = 0x0F; // podesavanje DLC
	
	buff[0][3] = (*(data)) | (*(data + 1) << 8); 	 //pisem prva dva bajta poruke
	buff[0][4] = (*(data + 2)) | (*(data + 3) << 8); // pisem prva dva bajta poruke
	buff[0][5] = (*(data + 4)) | (*(data + 5) << 8); // pisem prva dva bajta poruke
	buff[0][6] = (*(data + 6)) | (*(data + 7) << 8); // pisem prva dva bajta poruke
	
	while (C1TR01CONbits.TXREQ0 == 0b1);
	C1TR01CONbits.TXREQ0 = 1;	
}

void CAN_read(unsigned char *buf)
{
	unsigned char i;
	while(rxCounter == 0);

        if(++rxRdIndex == RX_BUFFER_SIZE)
            rxRdIndex = 0;
	
	for(i = 0; i < 8; i++)
		*(buf + i) = rxBuffer[rxRdIndex][i];

        SRbits.IPL = 7;
	rxCounter--;
        SRbits.IPL = 0;
}

void CAN_getLastMessage(unsigned char *buf)
{
    unsigned char i;
    for(i = 0; i < 8; i++)
		*(buf + i) = rxBuffer[rxRdIndex][i];
}

void __attribute__((interrupt, auto_psv)) _DMA1Interrupt(void) // rxInterrupt
{
  /* rxBuffer[rxWrIndex][0] = buff[1][3];
   rxBuffer[rxWrIndex][1] = buff[1][3] >> 8; 
   rxBuffer[rxWrIndex][2] = buff[1][4] >> 8;
   rxBuffer[rxWrIndex][3] = buff[1][4];
   rxBuffer[rxWrIndex][4] = buff[1][5] >> 8;
   rxBuffer[rxWrIndex][5] = buff[1][5];
   rxBuffer[rxWrIndex][6] = buff[1][6] >> 8;
   rxBuffer[rxWrIndex][7] = buff[1][6];*/

    if(++rxWrIndex == RX_BUFFER_SIZE)
   	rxWrIndex = 0;
   
   rxBuffer[rxWrIndex][0] = buff[1][3];
   rxBuffer[rxWrIndex][1] = buff[1][3] >> 8; 
   rxBuffer[rxWrIndex][2] = buff[1][4];
   rxBuffer[rxWrIndex][3] = buff[1][4] >> 8;
   rxBuffer[rxWrIndex][4] = buff[1][5];
   rxBuffer[rxWrIndex][5] = buff[1][5] >> 8;
   rxBuffer[rxWrIndex][6] = buff[1][6];
   rxBuffer[rxWrIndex][7] = buff[1][6] >> 8;
  
   rxCounter++;
   
   IFS0bits.DMA1IF = 0; // Clear the DMA1 Interrupt Flag;
   C1RXFUL1bits.RXFUL1 = 0;
}

void __attribute__((interrupt, auto_psv)) _DMA2Interrupt(void)	//txInterrupt
{
	
   IFS1bits.DMA2IF = 0; // Clear the DMA1 Interrupt Flag;
}

void __attribute__((interrupt, auto_psv)) _C1Interrupt(void)
{
	switch(C1VECbits.ICODE)
	{
		case 0b0000001:
		{
			C1RXFUL1bits.RXFUL1 = 0;
			
			break;
		}
		default:;
	}
	
	C1INTFbits.RBIF = 0b0;
}

void CAN_init(unsigned int ide)
{
//	RPOR7bits.RP15R = 0b10000; // CAN TX -> RP15
	RPOR3bits.RP6R = 0b10000;
	RPINR26bits.C1RXR = 5; // CAN RX -> RP14

	C1CTRL1bits.REQOP = 0b100; // set configuration mode
	while(C1CTRL1bits.OPMODE != 0b100); // sve dok ne udje u configuration mode
	
	/*     PODESAVANJE BAUD RATE    */
		
	// po defaultu je 0, ali nema veze. Bit kontrolise kojem setu registara se pristupa- da bih pristupio
	//kontrolnim registrima MORA BITI RESETOVAN!
	C1CTRL1bits.WIN = 0;

	C1CTRL1bits.CANCKS = 1; // Fcan = FCY --> Fcan <= 40MHz
	C1CTRL1bits.CSIDL = 0; // kada udje u IDLE mod da CAN nastavi sa radom
	
	// za 100kbps staviti 14
	// za 250kbps staviti 2
	C1CFG1bits.BRP = 2; // podesavanje tq = (2 * 1 * 1)/Fcan-> za Fcy = 4Mhz, za Fcy = 30Mhz samo delim sa 15 i dobijam isto
	C1CFG1bits.SJW = 0; // Synchronization Jump Width , 1 Tq
	C1CFG2bits.SEG1PH = 3;  // Phase 1 Segment bits 4 - bilo 8
	C1CFG2bits.SEG2PH =  2;// Phase 2 Segment bits 3- bilo 6
	C1CFG2bits.SEG2PHTS = 0b1;//Phase segment 2 Time selecet bit
	C1CFG2bits.PRSEG = 0b001;    // Propagation Time Segment bits 2- bilo 5
	C1CFG2bits.SAM = 0; // can bus- sampled once, not three times
		
	// n = 20
	//can busa baud rate - 100kbps
	//Ftq = 2 000 000 Hz
	//Tq = 1/Ftq = 0.5 us
	

	/*     PODESAVANJE FILTER I MASK REGISTARA    */
	
	
	C1CTRL1bits.WIN = 1; // OMOGUCAVAM pristup filter i mask registrima
	
	// svaka poruka koja se primi uporedjuje svoj identifier field sa filter vrednostima
	// CiRXFnSID sadrze filtere, ako primljena poruka, tj njen identifier ima neku od ovih vrednosti
	// poruka ce biti smestena u bafer, u suprotnom cvor odbija poruku
	//C1RXF0SIDbits.SID = 0b011;  // bafer 0
	C1FEN1bits.FLTEN = 0b01; // ukljucujem filter 0
	C1RXM0SIDbits.SID = 0b11111111111; // maska za filter-> gledam svih 11 bitova -> POSTOJE TRI REGISTRA ZA TRI MASKE
	C1RXM0SIDbits.MIDE = 1; //Match only message types (standard or extended address) that correspond to EXIDE bit in filter
	C1FMSKSEL1bits.F0MSK = 0b0;// KORISTI SE ACCEPTANCE FILTER 0 ZA IZBOR MASKE-> POSTOJE TRI REGISTRA, I OVDE BIRAM KOJI KORISTIM!
	C1RXF0SIDbits.EXIDE = 0; // Match only messages with standard identifier addresses	
	/* Acceptance Filter 0 to use Message Buffer 1 to store message */
	C1BUFPNT1bits.F0BP = 0x1;
    //C1RXF0SIDbits.SID = 0b00000000011;	// IDENTIFIKATOR CVORA
	C1RXF0SIDbits.SID = ide;
	
	C1CTRL1bits.WIN = 0;
	
	// 4 buffers in DMA RAM
	C1FCTRLbits.DMABS = 0; 
	// FIFO starts from TX/RX Buffer 2
	C1FCTRLbits.FSA   = 2;
	
	// Buffer 0 is a transmit buffer
	C1TR01CONbits.TXEN0 = 1; 
	//High Priority
	C1TR01CONbits.TX0PRI = 0b11; 
	// Buffer 1 is a receive buffer
	C1TR01CONbits.TXEN1 = 0;
	
//	C1INTEbits.RBIE = 1; // UKLJUCIVANJE INTERRUPTA ZA RX
//	C1INTEbitsTBIE = 1;  // UKLJUCIVANJE INTERRUPTA ZA RX

	/*		PODESAVANJE DMA 	*/
	
//	DMA1CONbits.SIZE = 1; // SIZE = BYTE
	DMA1CONbits.DIR = 0; // kanal 1 bice CAN RX
	DMA1CONbits.HALF = 0; // interrupt se generise kada se prebaci ceo podatak- ako je == 1 onda kada se prebaci pola podatka
	DMA1CONbits.AMODE = 0b10; // Peripheral Indirect Addressing mode
	DMA1CONbits.MODE = 0; //Continuous, Ping-Pong modes disabled
	DMA1REQbits.IRQSEL = 0x22; // ECAN1 RX
	DMA1PAD = (volatile unsigned int) &C1RXD; // Point DMA to ECAN1 buffers
	DMA1STA = __builtin_dmaoffset(buff); // // u buff ce DMA smestati ono sto dolazi iz ECAN RX-a
	DMA1STB = __builtin_dmaoffset(buff);
	// ovaj registar se programira tako da omoguci n+1 prenosa! zato je 7
	DMA1CNT = 7; // kada se prebaci 8 bajtova generise se interrupt --> pise 7?
	// Enable DMA Channel 1 interrupt 
	IEC0bits.DMA1IE = 1; 
	DMA1CONbits.CHEN = 1; // ukljucujem kanal
	
//	DMA2CONbits.SIZE = 1; // SIZE = BYTE
	DMA2CONbits.DIR = 1; // kanal 1 bice CAN RX
	DMA2CONbits.HALF = 0; // interrupt se generise kada se prebaci ceo podatak- ako je == 1 onda kada se prebaci pola podatka
	DMA2CONbits.AMODE = 0b10; // Peripheral Indirect Addressing mode
	DMA2CONbits.MODE = 0; //Continuous, Ping-Pong modes disabled
	DMA2REQbits.IRQSEL = 0x46; // ECAN1 TX
	DMA2PAD = (volatile unsigned int) &C1TXD; // Point DMA to ECAN1 buffers
	DMA2STA = __builtin_dmaoffset(buff); // // u buff ce DMA smestati ono sto ide u ECAN RX-a
	DMA2CNT = 7; // kada se prebaci 8 bajtova generise se interrupt --> pise 7?
	// Enable DMA Channel 2 interrupt 
	IEC1bits.DMA2IE = 1;
	DMA2CONbits.CHEN = 1; // ukljucujem kanal
	
	
	C1CTRL1bits.REQOP = 0b000;
	/* Wait for the ECAN module to enter into Loopback Operating Mode */
	// moze se koristiti LOOPBACK ZA TESTIRANJE! -> OBEZBEDJUJE DUMMY ACK, TAKO DA NEMA POTREBE ZA VISE CVOROVA!
	while(C1CTRL1bits.OPMODE != 0b000); 	
}
