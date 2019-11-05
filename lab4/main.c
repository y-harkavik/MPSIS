#include <msp430.h>

typedef unsigned char uint8_t;

// For all commands, CD signal must = 0
#define SET_COLUMN_ADDRESS_MSB        0x10  //Set SRAM col. addr. before write, last 4 bits =
                                            // ca4-ca7
#define SET_COLUMN_ADDRESS_LSB        0x00  //Set SRAM col. addr. before write, last 4 bits =
                                            // ca0-ca3
#define SET_POWER_CONTROL             0x2F  //Set Power control - booster, regulator, and follower
                                            // on
#define SET_SCROLL_LINE               0x40  //Scroll image up by SL rows (SL = last 5 bits),
                                            // range:0-63
#define SET_PAGE_ADDRESS              0xB0  //Set SRAM page addr (pa = last 4 bits), range:0-8
#define SET_VLCD_RESISTOR_RATIO       0x27  //Set internal resistor ratio Rb/Ra to adjust contrast
#define SET_ELECTRONIC_VOLUME_MSB     0x81  //Set Electronic Volume "PM" to adjust contrast
#define SET_ELECTRONIC_VOLUME_LSB     0x0F  //Set Electronic Volume "PM" to adjust contrast (PM =
                                            // last 5 bits)
#define SET_ALL_PIXEL_ON              0xA4  //Disable all pixel on (last bit 1 to turn on all pixels
                                            // - does not affect memory)
#define SET_INVERSE_DISPLAY           0xA6  //Inverse display off (last bit 1 to invert display -
                                            // does not affect memory)
#define SET_DISPLAY_ENABLE            0xAF  //Enable display (exit sleep mode & restore power)
#define SET_SEG_DIRECTION             0xA0  //Mirror SEG (column) mapping (set bit0 to mirror
                                            // display)
#define SET_COM_DIRECTION             0xC0  //Mirror COM (row) mapping (set bit3 to mirror display)
#define SYSTEM_RESET                  0xE2  //Reset the system. Control regs reset, memory not
                                            // affected
#define NOP                           0xE3  //No operation
#define SET_LCD_BIAS_RATIO            0xA2  //Set voltage bias ratio (BR = bit0)
#define SET_CURSOR_UPDATE_MODE        0xE0  //Column address will increment with write operation
                                            // (but no wrap around)
#define RESET_CURSOR_UPDATE_MODE      0xEE  //Return cursor to column address from before cursor
                                            // update mode was set
#define SET_ADV_PROGRAM_CONTROL0_MSB  0xFA  //Set temp. compensation curve to -0.11%/C
#define SET_ADV_PROGRAM_CONTROL0_LSB  0x90

// Pins from MSP430 connected to LCD
#define CD              BIT6
#define CS              BIT4
#define RST             BIT7
#define BACKLT          BIT6
#define SPI_SIMO        BIT1
#define SPI_CLK         BIT3

// Ports
#define CD_RST_DIR      P5DIR
#define CD_RST_OUT      P5OUT
#define CS_BACKLT_DIR   P7DIR
#define CS_BACKLT_OUT   P7OUT
#define CS_BACKLT_SEL   P7SEL
#define SPI_SEL         P4SEL
#define SPI_DIR         P4DIR

uint8_t currentPage = 0, currentColumn = 0;

// Dog102-6 Initialization Commands
uint8_t Dogs102x6_initMacro[] = {
    SET_SCROLL_LINE,
    SET_SEG_DIRECTION,
    SET_COM_DIRECTION,
    SET_ALL_PIXEL_ON,
    SET_INVERSE_DISPLAY,
    SET_LCD_BIAS_RATIO,
    SET_POWER_CONTROL,
    SET_VLCD_RESISTOR_RATIO,
    SET_ELECTRONIC_VOLUME_MSB,
    SET_ELECTRONIC_VOLUME_LSB,
    SET_ADV_PROGRAM_CONTROL0_MSB,
    SET_ADV_PROGRAM_CONTROL0_LSB,
    SET_DISPLAY_ENABLE,
    SET_PAGE_ADDRESS,
    SET_COLUMN_ADDRESS_MSB,
    SET_COLUMN_ADDRESS_LSB
};

uint8_t MODE_COMMANDS[2][1] = { {SET_SEG_DIRECTION}, {SET_SEG_DIRECTION | 1} };

int CURRENT_ORIENTATION = 0; // 0 - default, 1 - mirror horizontal
int COLUMN_START_ADDRESS = 31; // 0 - default(31), 1 - mirror horizontal(0)
int CURRENT_NUMBER = -5417;
int SUM_NUMBER = +981;
uint8_t symbols[12][11] = {
		{0x20, 0x20, 0x20, 0x20, 0x20, 0xF8, 0x20, 0x20, 0x20, 0x20, 0x20}, // plus
		{0x00, 0x00, 0x00, 0x00, 0xF8, 0xF8, 0xF8, 0x00, 0x00, 0x00, 0x00}, // minus
		{0xF8, 0xF8, 0xD8, 0xD8, 0xD8, 0xD8, 0xD8, 0xD8, 0xD8, 0xF8, 0xF8}, // num0
		{0xF8, 0xF8, 0x30, 0x30, 0x30, 0x30, 0xF0, 0xF0, 0x70, 0x70, 0x30}, // num1
		{0xF8, 0xF8, 0xC0, 0xC0, 0xC0, 0xF8, 0xF8, 0x18, 0x18, 0xF8, 0xF8}, // num2
		{0xF8, 0xF8, 0x18, 0x18, 0x18, 0xF8, 0xF8, 0x18, 0x18, 0xF8, 0xF8}, // num3
		{0x18, 0x18, 0x18, 0x18, 0xF8, 0xF8, 0xD8, 0xD8, 0xD8, 0xD8, 0xD8}, // num4
		{0xF8, 0xF8, 0x18, 0x18, 0x18, 0xF8, 0xF8, 0xC0, 0xC0, 0xF8, 0xF8}, // num5
		{0xF8, 0xF8, 0xD8, 0xD8, 0xD8, 0xF8, 0xF8, 0xC0, 0xC0, 0xF8, 0xF8}, // num6
		{0xC0, 0xC0, 0xE0, 0x70, 0x38, 0x18, 0x18, 0x18, 0x18, 0xF8, 0xF8}, // num7
		{0xF8, 0xF8, 0xD8, 0xD8, 0xD8, 0xF8, 0xD8, 0xD8, 0xD8, 0xF8, 0xF8}, // num8
		{0xF8, 0xF8, 0x18, 0x18, 0xF8, 0xF8, 0xD8, 0xD8, 0xD8, 0xF8, 0xF8} // num9
};

int lenHelper(int number);
int abs(int number);
int pow(int base, int exponent);
void printNumber(void);

void Dogs102x6_clearScreen(void);
void Dogs102x6_setAddress(uint8_t pa, uint8_t ca);
void Dogs102x6_writeData(uint8_t *sData, uint8_t i);
void Dogs102x6_writeCommand(uint8_t *sCmd, uint8_t i);
void Dogs102x6_backlightInit(void);
void Dogs102x6_init(void);

#pragma vector = PORT1_VECTOR
__interrupt void buttonS1(void)
{
	volatile int i = 0;

	for(i=0; i<2000; i++);

	if((P1IN & BIT7) == 0) {
		CURRENT_NUMBER += SUM_NUMBER;

		Dogs102x6_clearScreen();

		printNumber();
	}

	P1IFG = 0;
}

#pragma vector = PORT2_VECTOR
__interrupt void buttonS2(void)
{
	volatile int i = 0;

	for(i=0; i<1000; i++);

	if((P2IN & BIT2) == 0){
		if(CURRENT_ORIENTATION == 0) {
			COLUMN_START_ADDRESS = 0;
			CURRENT_ORIENTATION = 1;
		} else {
			COLUMN_START_ADDRESS = 31;
			CURRENT_ORIENTATION = 0;
		}

		Dogs102x6_writeCommand(MODE_COMMANDS[CURRENT_ORIENTATION], 1);
		Dogs102x6_clearScreen();
		printNumber();

		for(i=0; i<2000; i++);
	}

	P2IFG = 0;
}

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	
    P1DIR &= ~BIT7;
	P1OUT |= BIT7;
	P1REN |= BIT7;
	P1IE |= BIT7;
	P1IES |= BIT7;
	P1IFG = 0;

    P2DIR &= ~BIT2;
	P2OUT |= BIT2;
	P2REN |= BIT2;
	P2IE |= BIT2;
	P2IES |= BIT2;
	P2IFG = 0;

    Dogs102x6_init();
    Dogs102x6_backlightInit();
    Dogs102x6_clearScreen();
    printNumber();

    __bis_SR_register(GIE);

	return 0;
}

void printNumber(void) {
	int nDigits = lenHelper(CURRENT_NUMBER);
	int number = CURRENT_NUMBER;

    Dogs102x6_setAddress(0, COLUMN_START_ADDRESS);
    Dogs102x6_writeData(number > 0 ? symbols[0] : symbols[1], 11);

	int i = 0;
    int divider = pow(10, nDigits - 1);

    number = abs(number);

    for(i = 1; i <= nDigits; i++) {
    	int digit = number / divider;

        Dogs102x6_setAddress(i, COLUMN_START_ADDRESS);
    	Dogs102x6_writeData(symbols[digit + 2], 11);

    	number = number % divider;
    	divider /= 10;
    }
}

int lenHelper(int number) {
	number = abs(number);

	if(number >= 10000) return 5;
	if(number >= 1000) return 4;
	if(number >= 100) return 3;
	if(number >= 10) return 2;

	return 1;
}

int abs(int number) {
	return number > 0 ? number : number * (-1);
}

int pow(int base, int exponent) {
	int i = 0;
	int result = base;

    for(i = 0; i < exponent - 1; i++) {
    	result *= base;
    }

    return result;
}

void Dogs102x6_clearScreen(void)
{
    uint8_t LcdData[] = {0x00};
    uint8_t p, c;

    // 8 total pages in LCD controller memory
    for (p = 0; p < 8; p++)
    {
        Dogs102x6_setAddress(p, 0);
        // 132 total columns in LCD controller memory
        for (c = 0; c < 132; c++)
        {
            Dogs102x6_writeData(LcdData, 1);
        }
    }
}

void Dogs102x6_setAddress(uint8_t pa, uint8_t ca)
{
    uint8_t cmd[1];

    // Page boundary check
    if (pa > 7)
    {
        pa = 7;
    }

    // Column boundary check
    if (ca > 101)
    {
        ca = 101;
    }

    // Page Address Command = Page Address Initial Command + Page Address
    cmd[0] = SET_PAGE_ADDRESS + (7 - pa);
    uint8_t H = 0x00;
    uint8_t L = 0x00;
    uint8_t ColumnAddress[] = { SET_COLUMN_ADDRESS_MSB, SET_COLUMN_ADDRESS_LSB };

    currentPage = pa;
    currentColumn = ca;

    // Separate Command Address to low and high
    L = (ca & 0x0F);
    H = (ca & 0xF0);
    H = (H >> 4);
    // Column Address CommandLSB = Column Address Initial Command
    //                             + Column Address bits 0..3
    ColumnAddress[0] = SET_COLUMN_ADDRESS_LSB + L;
    // Column Address CommandMSB = Column Address Initial Command
    //                             + Column Address bits 4..7
    ColumnAddress[1] = SET_COLUMN_ADDRESS_MSB + H;

    // Set page address
    Dogs102x6_writeCommand(cmd, 1);
    // Set column address
    Dogs102x6_writeCommand(ColumnAddress, 2);
}

void Dogs102x6_writeData(uint8_t *sData, uint8_t i)
{
    // CS Low
    P7OUT &= ~CS;
    //CD High
    P5OUT |= CD;

    while (i)
    {
        currentColumn++;

        // Boundary check
        if (currentColumn > 101)
        {
            currentColumn = 101;
        }

        // USCI_B1 TX buffer ready?
        while (!(UCB1IFG & UCTXIFG)) ;

        // Transmit data and increment pointer
        UCB1TXBUF = *sData++;

        // Decrement the Byte counter
        i--;
    }

    // Wait for all TX/RX to finish
    while (UCB1STAT & UCBUSY) ;

    // Dummy read to empty RX buffer and clear any overrun conditions
    UCB1RXBUF;

    // CS High
    P7OUT |= CS;
}

void Dogs102x6_writeCommand(uint8_t *sCmd, uint8_t i)
{
    // CS Low
    P7OUT &= ~CS;

    // CD Low
    P5OUT &= ~CD;
    while (i)
    {
        // USCI_B1 TX buffer ready?
        while (!(UCB1IFG & UCTXIFG)) ;

        // Transmit data
        UCB1TXBUF = *sCmd;

        // Increment the pointer on the array
        sCmd++;

        // Decrement the Byte counter
        i--;
    }

    // Wait for all TX/RX to finish
    while (UCB1STAT & UCBUSY) ;

    // Dummy read to empty RX buffer and clear any overrun conditions
    UCB1RXBUF;

    // CS High
    P7OUT |= CS;
}

void Dogs102x6_backlightInit(void)
{
    CS_BACKLT_DIR |= BACKLT;
    CS_BACKLT_OUT |= BACKLT;
    CS_BACKLT_SEL &= ~BACKLT;
}

void Dogs102x6_init(void)
{
    // Port initialization for LCD operation
    CD_RST_DIR |= RST;
    // Reset is active low
    CD_RST_OUT &= RST;
    // Reset is active low
    CD_RST_OUT |= RST;
    // Chip select for LCD
    CS_BACKLT_DIR |= CS;
    // CS is active low
    CS_BACKLT_OUT &= ~CS;
    // Command/Data for LCD
    CD_RST_DIR |= CD;
    // CD Low for command
    CD_RST_OUT &= ~CD;

    // P4.1 option select SIMO
    SPI_SEL |= SPI_SIMO;
    SPI_DIR |= SPI_SIMO;
    // P4.3 option select CLK
    SPI_SEL |= SPI_CLK;
    SPI_DIR |= SPI_CLK;

    // Initialize USCI_B1 for SPI Master operation
    // Put state machine in reset
    UCB1CTL1 |= UCSWRST;
    //3-pin, 8-bit SPI master
    UCB1CTL0 = UCCKPH + UCMSB + UCMST + UCMODE_0 + UCSYNC;
    // Clock phase - data captured first edge, change second edge
    // MSB
    // Use SMCLK, keep RESET
    UCB1CTL1 = UCSSEL_2 + UCSWRST;
    UCB1BR0 = 0x02;
    UCB1BR1 = 0;
    // Release USCI state machine
    UCB1CTL1 &= ~UCSWRST;
    UCB1IFG &= ~UCRXIFG;

    Dogs102x6_writeCommand(Dogs102x6_initMacro, 13);

    // Deselect chip
    CS_BACKLT_OUT |= CS;
}
