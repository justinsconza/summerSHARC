/*****************************************************************************
 * dmaTest.c
 *****************************************************************************/

#include <Cdef21489.h>
#include <signal.h>
#include <stdio.h>
#include "coeffs.h"
#include <math.h>

 
// Check SRU Routings for errors.
#define SRUDEBUG
#include <SRU.h>

#define SELECT_SPI_SLAVE(select) (*pSPIFLG &= ~(DS0EN<<8))
#define DESELECT_SPI_SLAVE(select) (*pSPIFLG |= (DS0EN<<8))

// Addresses
#define AK4396_ADDR    (0x00)

#define AK4396_CTRL1   (0x00)
#define AK4396_CTRL2   (0x01)
#define AK4396_CTRL3   (0x02)
#define AK4396_LCH_ATT (0x03)
#define AK4396_RCH_ATT (0x04)

// Reset CTRL1 setting
#define AK4396_CTRL1_RST   (0x06)

// DAC register settings
#define AK4396_CTRL1_DEF   (0x07) 
#define AK4396_CTRL2_DEF   (0x02)
#define AK4396_CTRL3_DEF   (0x00)
#define AK4396_LCH_ATT_DEF (0xFF)
#define AK4396_RCH_ATT_DEF (0xFF)

#define BUFFER_LENGTH 512

// for masking out possible address offset when only interested in pointer relative positions			
#define BUFFER_MASK 0x000001FF    

#define DELAY_LENGTH 12000

// Configure the PLL for a core-clock of 266MHz and SDCLK of 133MHz
extern void initPLL_SDRAM(void);

// local setup functions
void initSRU(void);
void initSPI(unsigned int SPI_Flag);
void configureAK4396Register(unsigned int address, unsigned int data);
void initDMA(void);
void initSPDIF(void);
void clearDAIpins(void);

// dsp functions
void delayWithFeedback(int delaySpeed);
void firFilter(void);

void delay(int times);

// SPORT0 receive buffer a - also used for transmission
int rx0a_buf[BUFFER_LENGTH] = {0};

// buffer for storing floats	
double float_buffer[BUFFER_LENGTH] = {0.0};

// SPORT1 transmit dummy buffer - for making sure tx is behind rx
//int tx1a_buf_dummy[BUFFER_LENGTH/2] = {0};

// SPORT1 transmit buffer	
int tx1a_buf[BUFFER_LENGTH] = {0};

/* 
TCB format:       ECx (length of destination buffer),
				  EMx (destination buffer step size),
				  EIx (destination buffer index (initialized to start address)),
				  GPx ("general purpose"),
				  CPx ("Chain Point register"; points to last address (IIx) of
			   								   next TCB to jump to
				                               upon completion of this TCB.),
				  Cx  (length of source buffer),
				  IMx (source buffer step size),
				  IIx (source buffer index (initialized to start address)) 
*/

int rx0a_tcb[8]  = {0,0,0,0, 0, BUFFER_LENGTH, 1, 0};				// SPORT0 receive a tcb from SPDIF
int tx1a_tcb[8]  = {0,0,0,0, 0, BUFFER_LENGTH, 1, 0};				// SPORT1 transmit a tcb to DAC

int tx1a_delay_tcb[8]  = {0, 0, 0, 0, 0, BUFFER_LENGTH/2, 1, 0};	// SPORT1 transmit a tcb to DAC

int counter = 0;
int dsp = 0;
int delay_ptr = 0;
double delay_buffer[2*DELAY_LENGTH] = {0};

void main(void) {

	initPLL_SDRAM();

	initSPI(DS0EN);
	initSRU();

	configureAK4396Register(AK4396_CTRL2, AK4396_CTRL2_DEF);
	delay(10);
	
	//Set the reset so that the device is ready to initialize registers.
	configureAK4396Register(AK4396_CTRL1, AK4396_CTRL1_RST);
	delay(10);
        	
    configureAK4396Register(AK4396_CTRL1, AK4396_CTRL1_DEF);
	delay(10);
	
	configureAK4396Register(AK4396_CTRL3, AK4396_CTRL3_DEF);
	delay(10);
	
	configureAK4396Register(AK4396_LCH_ATT, AK4396_LCH_ATT_DEF);
	delay(10);
	
	configureAK4396Register(AK4396_RCH_ATT, AK4396_RCH_ATT_DEF);
	delay(10);

	initDMA();

	initSPDIF();

	while(1){
		firFilter();
		//delayWithFeedback(4);
	}  
}

void initSPDIF()
{
    // Use default setting of SPDIF
    *pDIRCTL=0x0;
}

void initSRU() 
{
	clearDAIpins();
	
	// use pin 12 on the board for SPDIF in
	// this is the pin second from the power,
	// in the not-ground row
	SRU(LOW, DAI_PB12_I);
	SRU(LOW, PBEN12_I);
	SRU(DAI_PB12_O, DIR_I);

	//Power off the DAC
	SRU2(HIGH, DPI_PBEN04_I);
	SRU2(LOW, DPI_PB04_I);
	
	delay(10);
	 
	/* ---------- DAC ----------------- */
    //Attach Main Clocks from SPDIF receiver

	//MCLK
	SRU(DIR_TDMCLK_O, DAI_PB05_I);
	SRU(HIGH,PBEN05_I);
	
	//BICK
	SRU(DIR_CLK_O, DAI_PB06_I);
	SRU(HIGH,PBEN06_I);
	
	//LRCK
	SRU(DIR_FS_O, DAI_PB03_I);
	SRU(HIGH,PBEN03_I);
	
	//CSN
	SRU2(SPI_FLG0_O, DPI_PB07_I);
    SRU2(SPI_FLG0_PBEN_O, DPI_PBEN07_I);

	//Set MOSI/CDT1 to output
	SRU2(SPI_MOSI_O, DPI_PB01_I);
	SRU2(HIGH, DPI_PBEN01_I);
	
	//Send SPI clock to DPI 3
	SRU2(SPI_CLK_O, DPI_PB03_I);
	SRU2(SPI_CLK_PBEN_O, DPI_PBEN03_I);
	
	//Power back on the DAC
	SRU2(HIGH, DPI_PB04_I);

	delay(10);  

	/* ---------- SPORT -------------- */
	SRU(DAI_PB06_O, SPORT0_CLK_I);
	SRU(DAI_PB06_O, SPORT1_CLK_I);
	SRU(DAI_PB03_O, SPORT0_FS_I);
	SRU(DAI_PB03_O, SPORT1_FS_I);

	// SPORT0 receives from SPDIF 
	SRU(DIR_DAT_O, SPORT0_DA_I);

	// SPORT1 outputs to the DAC
	SRU(SPORT1_DA_O, DAI_PB04_I);
	SRU(HIGH, PBEN04_I);

	
	/* ---------- DEBUG -------------- */
	
	// LRCLK to debug, pin 11
	SRU(DAI_PB03_O, DAI_PB11_I);
	SRU(HIGH, PBEN11_I);

	// MOSI to debug
    SRU2(SPI_MOSI_O, DPI_PB09_I);
    SRU2(HIGH, DPI_PBEN09_I);

}

void initSPI(unsigned int SPI_Flag)
{
	// Configure the SPI Control registers
    // First clear a few registers
    *pSPICTL  =(TXFLSH | RXFLSH) ; //Clear TXSPI and RXSPI
    *pSPIFLG = 0; //Clear the slave select

    //BAUDR is bits 15-1 and 0 is reserved
    *pSPIBAUD = 25;   //SPICLK baud rate = PCLK / (4*BAUDR)
    //PCLK = 200MHz and 200MHz/100 = 2 MHz? - double check this...

    // Setup the SPI Flag register using the Flag specified in the call
    *pSPIFLG = (0xF00|SPI_Flag);

    // Now setup the SPI Control register
    *pSPICTL = (SPIEN | SPIMS | WL16 | MSBF | TIMOD1 | CLKPL|GM);
    
    // SPIEN - SPI System Enable
    // SPIMS - Master Slave Mode Bit - 1 indicates we are a master device
    // WL16 - Word length is 16 bits
    // MSBF - MSB sent/received first
    // (NOT) CPHASE - SPICLK starts toggling at the middle of 1st data bit
    // CLKPL - Clock Polarity - Active low SPICLK (SPICLK high is the idle state)
    // (NOT) SMLS - Seamless Transfer disabled - delay before the next word starts, done to ensure frame on osciliscope, but check if you can change this.
    // GM - Get Data. When RXSPI is full, get more data which overwrites previous data.
}

void configureAK4396Register(unsigned int address, unsigned int data)
{
    unsigned short message = 0;

    SELECT_SPI_SLAVE(DS0EN);

    /// MSB                                LSB
    // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]

    //Read/Write (stay at 1, write only, as per diagram).
    message |= 0x2000; //message[3]
    
    //Register Address
    message |= (address << 8);
        
    message |= data;
    
    *pTXSPI = message;

    //Wait for the SPI to indicate that it has finished.
    while ((*pSPISTAT & TXS))

    //Wait for the SPI to indicate that it has finished.
    while (!(*pSPISTAT & SPIFE))
    
	delay(10);
    
    DESELECT_SPI_SLAVE(DS0EN);

}

void initDMA() {

	*pSPMCTL0 = 0; // ******* ONLY SET ONCE 
	*pSPMCTL1 = 0;
	
	*pSPCTL0 = 0;
	*pSPCTL1 = 0;

	// where rx goes next (itself)
	rx0a_tcb[4] = ((int) rx0a_tcb  + 7) & 0x7FFFF | (1<<19);
	// where rx fills
	rx0a_tcb[7] = (unsigned int) rx0a_buf;

	// where SPORT0 fills first (and always)
	*pCPSP0A = ((int) rx0a_tcb  + 7) & 0x7FFFF | (1<<19);

	// where tx goes next (itself)
	tx1a_tcb[4] = ((int) tx1a_tcb  + 7) & 0x7FFFF | (1<<19);
	// where tx gets data
	tx1a_tcb[7] = (unsigned int) tx1a_buf;

	// where tx dummy goes next (regular tx)
	//tx1a_delay_tcb[4] = ((int) tx1a_tcb  + 7) & 0x7FFFF | (1<<19);
	// where tx dummy gets data
	//tx1a_delay_tcb[7] = (unsigned int) tx1a_buf_dummy;

	// where SPORT1 gets from first
	//*pCPSP1A = ((int) tx1a_delay_tcb  + 7) & 0x7FFFF | (1<<19);
	*pCPSP1A = ((int) tx1a_tcb  + 7) & 0x7FFFF | (1<<19);


	// SPORT0 as receiver
	*pSPCTL0 = 	SPEN_A | 	// enable channel A
				SDEN_A |	// enable channel A DMA
				SCHEN_A | 	// enable channel A DMA chaining
				OPMODE |	// enable I2S mode 
				L_FIRST | 	// I2S sends left first
				SLEN24; 	// 24-bit word length

				
	// SPORT1 as transmitter
	*pSPCTL1 = 	SPEN_A |	// enable channel A
				SDEN_A |	// enable channel A DMA
				SCHEN_A |	// enable channel A DMA chaining
				OPMODE | 	// enable I2S mode 
				L_FIRST | 	// I2S sends left first
				SLEN24 | 	// 24-bit word length
				SPTRAN;		// set as transmitter
				 
				 
				
}

void delay(int times)
{
	int i;

    for(i = times; i > 0; --i)
    	asm("nop;");
}

void clearDAIpins(void)
{

//  Tie the pin buffer inputs LOW for all DAI pins.  Even though
//  these pins are inputs to the SHARC, tying unused pin buffer inputs
//  LOW is "good coding style" to eliminate the possibility of
//  termination artifacts internal to the IC.  Note that signal
//  integrity is degraded only with a few specific SRU combinations.
//  In practice, this occurs VERY rarely, and these connections are
//  typically unnecessary.  This is GROUP D

    SRU(LOW, DAI_PB01_I);
    SRU(LOW, DAI_PB02_I);
    SRU(LOW, DAI_PB03_I);
    SRU(LOW, DAI_PB04_I);
    SRU(LOW, DAI_PB05_I);
    SRU(LOW, DAI_PB06_I);
    SRU(LOW, DAI_PB07_I);
    SRU(LOW, DAI_PB08_I);
    SRU(LOW, DAI_PB09_I);
    SRU(LOW, DAI_PB10_I);
    SRU(LOW, DAI_PB11_I);
    SRU(LOW, DAI_PB12_I);
    SRU(LOW, DAI_PB13_I);
    SRU(LOW, DAI_PB14_I);
    SRU(LOW, DAI_PB15_I);
    SRU(LOW, DAI_PB16_I);
    SRU(LOW, DAI_PB17_I);
    SRU(LOW, DAI_PB18_I);
    SRU(LOW, DAI_PB19_I);
    SRU(LOW, DAI_PB20_I);

//  Tie the pin buffer enable inputs LOW for all DAI pins so
//  that they are always input pins.  This is GROUP F.

    SRU(LOW, PBEN01_I);
    SRU(LOW, PBEN02_I);
    SRU(LOW, PBEN03_I);
    SRU(LOW, PBEN04_I);
    SRU(LOW, PBEN05_I);
    SRU(LOW, PBEN06_I);
    SRU(LOW, PBEN07_I);
    SRU(LOW, PBEN08_I);
    SRU(LOW, PBEN09_I);
    SRU(LOW, PBEN10_I);
    SRU(LOW, PBEN11_I);
    SRU(LOW, PBEN12_I);
    SRU(LOW, PBEN13_I);
    SRU(LOW, PBEN14_I);
    SRU(LOW, PBEN15_I);
    SRU(LOW, PBEN16_I);
    SRU(LOW, PBEN17_I);
    SRU(LOW, PBEN18_I);
    SRU(LOW, PBEN19_I);
    SRU(LOW, PBEN20_I);
}

void delayWithFeedback(int delaySpeed) 
{
	while( ( ((int)rx0a_buf + dsp) & BUFFER_MASK ) != ( *pIISP0A & BUFFER_MASK ) ) {

		counter = (counter + 1) % delaySpeed;
		
		// delay_ptr is putting what rx just took in into the delay_buffer.
		// once the delay length is satisfied, dsp pointer is adding to the receive buffer what rx
		// already put there PLUS what's just ahead of where delay_ptr is now. this way, 
		// the desired delay time is satisfied constantly.
		
		
		// ----------------- feedback w/ ints and floats ---------- //

		if (counter == 0) {

			if (rx0a_buf[dsp] & 0x00800000)			// is rx negative?
				rx0a_buf[dsp] |= 0xFF000000;		// sign-extend it

			// get current input as a float into float buffer
			float_buffer[dsp] = (float)rx0a_buf[dsp];

			// write current input + attenuated delay into delay buffer at delay_ptr
			delay_buffer[delay_ptr] = 0.5*delay_buffer[delay_ptr] + float_buffer[dsp];

			// increment delay_ptr - now points to oldest spot in delay buffer
			delay_ptr = (delay_ptr + 1)%DELAY_LENGTH;

			// put the oldest delay value (from 1 delay ago) into receive float buffer
			float_buffer[dsp] = delay_buffer[ delay_ptr ];

			// get the float back into int format for tx to play it back coming up
			tx1a_buf[dsp] = (int)float_buffer[dsp];
		}

		// make sure it's 24 bit for 24 bit i2s
		tx1a_buf[dsp] &= 0x00FFFFFF;			

		// increment the buffer_ptr
    	dsp = (dsp + 1)%BUFFER_LENGTH;                  

	}
    return;
}

void firFilter() {

	double acc = 0.0;

	// make sure dsp is behind rx
	while( ( ((int)rx0a_buf + dsp) & BUFFER_MASK ) != ( *pIISP0A & BUFFER_MASK ) ) {

		int i = 0;

		if (rx0a_buf[dsp] & 0x00800000)			// is rx negative?
			rx0a_buf[dsp] |= 0xFF000000;		// sign-extend it

		// get input as a float
		float_buffer[dsp] = (double)rx0a_buf[dsp];

		//printf("float_buffer[%d] = %f\n", dsp, float_buffer[dsp]);
		
		// convolution of input (going bakcwards) and coeffs (going forward)
		for (i=0; i<FILTER_LENGTH; i++) {
			acc += float_buffer[ (dsp - i + BUFFER_LENGTH) % BUFFER_LENGTH ] * coeffs[i];
		}
		
		// put the output (acc) into the tx buffer
		
		//The filter is working but there's a deadzone 
		//in the output signal.  Don't understand why.
		
		tx1a_buf[dsp] = (int)acc;
		//tx1a_buf[dsp] = rx0a_buf[dsp];

		// make sure it's 24 bit for 24 bit i2s
		tx1a_buf[dsp] &= 0x00FFFFFF;					

		// increment the buffer_ptr
		dsp = (dsp + 1)%BUFFER_LENGTH;  

		return;              

	}	
}
























































