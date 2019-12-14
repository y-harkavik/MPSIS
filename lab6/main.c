#include <msp430.h>
#include <math.h>
#include "CTS_Layer.h"

/*================================================================== LCD DEFINITION ==================================================================*/

#define SET_COLUMN_ADDRESS_LSB        0x00
#define SET_COLUMN_ADDRESS_MSB        0x10
#define SET_PAGE_ADDRESS              0xB0

#define SET_SEG_DIRECTION             0xA0
#define SET_COM_DIRECTION             0xC0

#define SET_POWER_CONTROL             0x2F // Управление питанием. PC[0] – усилитель, PC[1] — регулятор, PC[2] — повторитель. 0 — отключено, 1 — включено
#define SET_SCROLL_LINE               0x40 // Установка начальной линии скроллинга SL=0..63
#define SET_VLCD_RESISTOR_RATIO       0x27 // Установка уровня внутреннего резисторного делителя PC = [0..7].Используется для управления контрастом.
#define SET_ELECTRONIC_VOLUME_MSB     0x81 // Регулировка контраста. Двухбайтная команда. PM[5..0] PM = 0..63.
#define SET_ELECTRONIC_VOLUME_LSB     0x0F
#define SET_ALL_PIXEL_ON              0xA4 // Включение всех пикселей. 0 – отображение содержимого памяти, 1 – все пиксели включены (содержимое памяти сохраняется).
#define SET_INVERSE_DISPLAY           0xA6 // Включение инверсного режима. 0 — нормальное отображение содержимого памяти, 1 — инверсное.
#define SET_DISPLAY_ENABLE            0xAF // Отключение экрана. 0 — экран отключен, 1 — включен.
#define SET_LCD_BIAS_RATIO            0xA2 // Смещение напряжения делителя: 0 – 1/9, 1 – 1/7.
#define SET_ADV_PROGRAM_CONTROL0_MSB  0xFA // Расширенное управление. ТС — температурная компенсация 0 = -0.05, 1 = -0.11 % / °С;
#define SET_ADV_PROGRAM_CONTROL0_LSB  0x90 // WC – циклический сдвиг столбцов 0 = нет, 1 = есть; WP –циклический сдвиг страниц 0 = нет, 1 = есть.

#define CD              BIT6
#define CS              BIT4

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

uint8_t symbols[12][11] = {
		{0x20, 0x20, 0x20, 0x20, 0x20, 0xF8, 0x20, 0x20, 0x20, 0x20, 0x20}, // plus
		{0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00}, // minus
		{0xF8, 0xF8, 0xD8, 0xD8, 0xD8, 0xD8, 0xD8, 0xD8, 0xD8, 0xF8, 0xF8}, // num0
		{0xF8, 0xF8, 0x30, 0x30, 0x30, 0x30, 0xF0, 0xF0, 0x70, 0x70, 0x30}, // num1
		{0xF8, 0xF8, 0xC0, 0xC0, 0xC0, 0xF8, 0xF8, 0x18, 0x18, 0xF8, 0xF8}, // num2
		{0xF8, 0xF8, 0x18, 0x18, 0x18, 0xF8, 0xF8, 0x18, 0x18, 0xF8, 0xF8}, // num3
		{0x18, 0x18, 0x18, 0x18, 0xF8, 0xF8, 0xD8, 0xD8, 0xD8, 0xD8, 0xD8}, // num4
		{0xF8, 0xF8, 0x18, 0x18, 0x18, 0xF8, 0xF8, 0xC0, 0xC0, 0xF8, 0xF8}, // num5
		{0xF8, 0xF8, 0xD8, 0xD8, 0xD8, 0xF8, 0xF8, 0xC0, 0xC0, 0xF8, 0xF8}, // num6
		{0xC0, 0xC0, 0xE0, 0x70, 0x38, 0x18, 0x18, 0x18, 0x18, 0xF8, 0xF8}, // num7
		{0xF8, 0xF8, 0xD8, 0xD8, 0xD8, 0xF8, 0xD8, 0xD8, 0xD8, 0xF8, 0xF8}, // num8
		{0xF8, 0xF8, 0x18, 0x18, 0xF8, 0xF8, 0xD8, 0xD8, 0xD8, 0xF8, 0xF8}  // num9
};

int COLUMN_START_ADDRESS = 30;

void Dogs102x6_clearScreen(void);
void Dogs102x6_setAddress(uint8_t pa, uint8_t ca);
void Dogs102x6_writeData(uint8_t* sData, uint8_t i);
void Dogs102x6_writeCommand(uint8_t* sCmd, uint8_t i);
void Dogs102x6_backlightInit(void);
void Dogs102x6_init(void);

/*================================================================== END LCD DEFINITION  ==================================================================*/

/*================================================================== PAD DEFINITION ==================================================================*/

#define NUM_KEYS    5
#define LED4        BIT1
#define LED5        BIT2
#define LED6        BIT3
#define LED7        BIT4
#define LED8        BIT5

const uint8_t LED_PADx[NUM_KEYS] = { LED4, LED5, LED6, LED7, LED8 };
const struct Element* ELEMENT_PADx[NUM_KEYS] = { &element0, &element1, &element2, &element3, &element4 };

struct Element* pressedKey;

/*================================================================== END PAD DEFINITION ==================================================================*/

int getNumberLength(long int number);
void printNumber(long int number);
void SetVcoreUp(uint16_t level);
void ADC12A_init();
void Potentiometer_init();


#pragma vector=ADC12_VECTOR
__interrupt void ADC12_ISR(void) {
	Dogs102x6_clearScreen();
	printNumber(ADC12MEM0 * (1.5 / 4096) * 1000);
}

int main(void) {
	WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

	P1DIR |= BIT1 | BIT2 | BIT3 | BIT4 | BIT5;
	P1OUT &= ~(BIT1 | BIT2 | BIT3 | BIT4 | BIT5);

	Dogs102x6_init();
	Dogs102x6_backlightInit();
	Dogs102x6_clearScreen();

	ADC12A_init();
	Potentiometer_init();

	 //*  Set DCO to 25Mhz and SMCLK to DCO. Taken from MSP430F55xx_UCS_10.c code
	 //*  example.

	 // Increase Vcore setting to level3 to support fsystem=25MHz
	 // NOTE: Change core voltage one level at a time..
	SetVcoreUp(0x01);
	SetVcoreUp(0x02);
	SetVcoreUp(0x03);

	UCSCTL3 = SELREF_2;                       // Set DCO FLL reference = REFO
	UCSCTL4 |= SELA_2;                        // Set ACLK = REFO

	__bis_SR_register(SCG0);                  // Disable the FLL control loop
	UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
	UCSCTL1 = DCORSEL_7;                      // Select DCO range 50MHz operation
	UCSCTL2 = FLLD_1 + 762;                   // Set DCO Multiplier for 25MHz
											  // (N + 1) * FLLRef = Fdco
											  // (762 + 1) * 32768 = 25MHz
											  // Set FLL Div = fDCOCLK/2
	__bic_SR_register(SCG0);                  // Enable the FLL control loop

	// Worst-case settling time for the DCO when the DCO range bits have been
	// changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
	// UG for optimization.
	// 32 x 32 x 25 MHz / 32,768 Hz ~ 780k MCLK cycles for DCO to settle
	__delay_cycles(782000);
	// Loop until XT1,XT2 & DCO stabilizes - In this case only DCO has to stabilize
	do
	{
		UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
		// Clear XT2,XT1,DCO fault flags
		SFRIFG1 &= ~OFIFG;                      // Clear fault flags
	} while (SFRIFG1 & OFIFG);                   // Test oscillator fault flag


	TI_CAPT_Init_Baseline(&slider);
	TI_CAPT_Update_Baseline(&slider, 5);

	__bis_SR_register(GIE);

	while (1) {
		P1OUT &= ~LED_PADx[1];
		pressedKey = (struct Element*)TI_CAPT_Buttons(&slider);

		if (pressedKey == ELEMENT_PADx[4]) {
			P1OUT |= LED_PADx[1];
			ADC12CTL0 &= ~ADC12SC;
			ADC12CTL0 |= ADC12SC;
		}

		__delay_cycles(100000);
	}
}
/*
 *  ======== SetVcorUp(uint16_t) ========
 *  Taken from MSP430F55xx_UCS_10.c code example.
 */
void SetVcoreUp(uint16_t level)
{
	// Open PMM registers for write
	PMMCTL0_H = PMMPW_H;
	// Set SVS/SVM high side new level
	SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
	// Set SVM low side to new level
	SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
	// Wait till SVM is settled
	while ((PMMIFG & SVSMLDLYIFG) == 0);
	// Clear already set flags
	PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
	// Set VCore to new level
	PMMCTL0_L = PMMCOREV0 * level;
	// Wait till new level reached
	if ((PMMIFG & SVMLIFG))
		while ((PMMIFG & SVMLVLRIFG) == 0);
	// Set SVS/SVM low side to new level
	SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
	// Lock PMM registers for write access
	PMMCTL0_H = 0x00;
}

void ADC12A_init() {
	ADC12CTL0 = ADC12SHT0_15 | ADC12ON;
	ADC12CTL1 = ADC12CSTARTADD_0 | ADC12SHS_0 | ADC12SHP | ADC12SSEL_3 |  ADC12CONSEQ_2;
	ADC12MCTL0 = ADC12EOS | ADC12INCH_5;
	ADC12IE |= ADC12IE0;
	ADC12CTL0 |= ADC12ENC;
}

void Potentiometer_init() {
	// setup A5 pin
	P6DIR &= ~BIT5;
	P6SEL |= BIT5;
	// apply high voltage to resistor
	P8DIR |= BIT0;
	P8SEL &= ~BIT0;
	P8OUT |= BIT0;
}

/*================================================================== LCD IMPLEMENTATION ==================================================================*/

int getNumberLength(long int number) {
	int length = 0;
	number = fabsl(number);

	if (number == 0) {
		return 1;
	}

	while (number) {
		number /= 10;
		length++;
	}

	return length;
}

void printNumber(long int number) {
	int nDigits = getNumberLength(number);

	Dogs102x6_setAddress(0, COLUMN_START_ADDRESS);
	Dogs102x6_writeData(number > 0 ? symbols[0] : symbols[1], 11);

	int i = 0;
	long int divider = pow(10, nDigits - 1);

	number = fabsl(number);

	for (i = 1; i <= nDigits; i++) {
		int digit = number / divider;

		Dogs102x6_setAddress(i, COLUMN_START_ADDRESS);
		Dogs102x6_writeData(symbols[digit + 2], 11);

		number = number % divider;
		divider /= 10;
	}
}

void Dogs102x6_clearScreen(void)
{
	uint8_t LcdData[] = { 0x00 };
	uint8_t p, c;

	for (p = 0; p < 8; p++)
	{
		Dogs102x6_setAddress(p, 0);

		for (c = 0; c < 132; c++)
		{
			Dogs102x6_writeData(LcdData, 1);
		}
	}
}

void Dogs102x6_setAddress(uint8_t pa, uint8_t ca)
{
	uint8_t cmd[1];

	if (pa > 7)
	{
		pa = 7;
	}

	if (ca > 101)
	{
		ca = 101;
	}

	cmd[0] = SET_PAGE_ADDRESS + (7 - pa);
	uint8_t H = 0x00;
	uint8_t L = 0x00;
	uint8_t ColumnAddress[] = { SET_COLUMN_ADDRESS_MSB, SET_COLUMN_ADDRESS_LSB };

	L = (ca & 0x0F);
	H = (ca & 0xF0);
	H = (H >> 4);

	ColumnAddress[0] = SET_COLUMN_ADDRESS_LSB + L;
	ColumnAddress[1] = SET_COLUMN_ADDRESS_MSB + H;

	Dogs102x6_writeCommand(cmd, 1);
	Dogs102x6_writeCommand(ColumnAddress, 2);
}

void Dogs102x6_writeData(uint8_t* sData, uint8_t i)
{
	P7OUT &= ~CS;
	P5OUT |= CD;

	while (i)
	{
		while (!(UCB1IFG & UCTXIFG));

		UCB1TXBUF = *sData;

		sData++;
		i--;
	}

	while (UCB1STAT & UCBUSY);

	UCB1RXBUF;

	P7OUT |= CS;
}

void Dogs102x6_writeCommand(uint8_t* sCmd, uint8_t i)
{
	P7OUT &= ~CS;
	P5OUT &= ~CD;

	while (i)
	{
		while (!(UCB1IFG & UCTXIFG));

		UCB1TXBUF = *sCmd;

		sCmd++;
		i--;
	}

	while (UCB1STAT & UCBUSY);

	UCB1RXBUF;

	P7OUT |= CS;
}

void Dogs102x6_backlightInit(void)
{
	P7DIR |= BIT6;
	P7OUT |= BIT6;
	P7SEL &= ~BIT6;
}

void Dogs102x6_init(void)
{
	P5DIR |= BIT7;
	P5OUT &= BIT7;
	P5OUT |= BIT7;

	P7DIR |= CS;

	P5DIR |= CD;
	P5OUT &= ~CD;

	P4SEL |= BIT1;
	P4DIR |= BIT1;

	P4SEL |= BIT3;
	P4DIR |= BIT3;

	UCB1CTL1 = UCSSEL_2 + UCSWRST;
	UCB1BR0 = 0x02;
	UCB1BR1 = 0;

	UCB1CTL0 = UCCKPH + UCMSB + UCMST + UCMODE_0 + UCSYNC;

	UCB1CTL1 &= ~UCSWRST;
	UCB1IFG &= ~UCRXIFG;

	Dogs102x6_writeCommand(Dogs102x6_initMacro, 13);
}

/*================================================================== END LCD IMPLEMENTATION ==================================================================*/