/*
 * File:   main.cpp
 * Author: Henrik Thulin
 *
 * Created 16 may 2014, 20:26
 *
 * xc32-g++ optimizer should be set to 1 when using the bit-banging routine
 * When using the SPI routine optimizer can be set to any value
 *
 */

#pragma config POSCMOD=XT // Primary Oscillator Configuration bits
#pragma config FSOSCEN=OFF // Secondary Oscillator Enable bit
#pragma config FNOSC=PRIPLL // Oscillator Selection bits
#pragma config OSCIOFNC=1 // CLKO Enable Configuration bit
#pragma config FPLLODIV=DIV_1 // Default PLL Output Divisor bits
#pragma config FPLLMUL=MUL_20 // PLL Multiplier bits
#pragma config FPLLIDIV=DIV_1 // PLL Input Divider bits
#pragma config FPBDIV=DIV_1 // Peripheral Bus Clock Divisor Default Value bits
#pragma config FWDTEN=OFF
#pragma config CP=OFF
#pragma config BWP=OFF
#pragma config PWP=OFF

#define SYS_FREQ    (80000000L)
#define GetPeripheralClock()  (FYC/(1 << OSCCONbits.PBDIV))
#define GetInstructionClock()  (FYC)


// For 16x16 RGB leds
#define ROWS 16
#define COLS 16

// Settings used when bitbanging. B5 is here used as output pin
#define LED_PORT IOPORT_B
#define LED_PIN BIT_5
#define LED_HIGH mPORTBSetBits(LED_PIN);
#define LED_LOW mPORTBClearBits(LED_PIN);

#define SPI_HIGH 0b11100000000000000000000000000000 // This will be sent for highs
#define SPI_LOW 0b10000000000000000000000000000000 // This will be sent for lows
#define SPI_BITSPERBIT 4 // Equals the numbers of bits defined above

#define GAMMACORRECTION_ON // Enables gamma currection
#define SPIMODE_ON // Enables SPI mode. If removed, bit-banging will be used
#define SPIDMA_ON // Enables DMA mode for SPI

#include <plib.h>
#include <math.h>

#ifdef SPIMODE_ON
int spidataSize = ceil((double) (SPI_BITSPERBIT * COLS * ROWS * 24) / (double) 32); //(int)((COLS * ROWS) * (32 / SPI_BITSPERBIT));

unsigned int* spidata = (unsigned int*) malloc(spidataSize);
#endif

unsigned int bitmapBuffer[ROWS*COLS];

#ifdef GAMMACORRECTION_ON
unsigned char ledTweak[256];

void makeGamma(unsigned int& color) {

    color = (ledTweak[(color >> 16) & 0xff] << 16) |
            (ledTweak[(color >> 8) & 0xff] << 8) |
            (ledTweak[(color) & 0xff]);
}
#endif


#ifdef SPIMODE_ON
// Initializes the SPI module. SPI here configured for port RB13. Effective bit-rate to the WS2812 will be ~700 kHz
// If you have problems try a 1 kOhm resistor in series

unsigned char SendSPI2(unsigned char c) {

    SPI2BUF = c;
    while (!SPI2STATbits.SPIRBF) Nop();
    return SPI2BUF;
}

unsigned char GetResponse() {

    unsigned char c;

    do {
        c = SendSPI2(0xff);
    } while (c == 0xff);

    Nop();
    return c;
}

unsigned char SendSpi2(unsigned char cmd, unsigned int data, unsigned char crc) {

    SendSPI2(cmd);
    SendSPI2((data >> 24) & 0x000000ff);
    SendSPI2((data >> 16) & 0x000000ff);
    SendSPI2((data >> 8) & 0x000000ff);
    SendSPI2((data >> 0) & 0x000000ff);
    SendSPI2(crc);

    return 0;
}

void GetSdSector(unsigned int sector) {

    unsigned char chrarray[512];

    PORTBbits.RB9 = 1;

    for (int i = 0; i < 20; i++) SendSPI2(0xff);

    PORTBbits.RB9 = 0;

    SendSpi2(0x40, 0x00000000, 0x95);

    if (GetResponse() != 0x01) return;

    PORTBbits.RB9 = 1;

    PORTBbits.RB9 = 0;

    SendSpi2(0x48, 0x1AA, 0x87);
    if (GetResponse() != 0x01) return;

    PORTBbits.RB9 = 1;

    do {

        PORTBbits.RB9 = 0;

        SendSpi2(0x41, 0x40000000, 0xFF);

        bool match = false;

        if (GetResponse() == 0x00) {

            PORTBbits.RB9 = 1;
            break;
        }

        PORTBbits.RB9 = 1;

    } while (true);


    PORTBbits.RB9 = 0;

    SendSpi2(0x51, sector, 0xff);

    GetResponse();

    for (int i = 0; i < 6; i++) SendSPI2(0x00);

    for (int i = 0; i < 512; i++) {

        chrarray[i] = SendSPI2(0x00);
    }

    PORTBbits.RB9 = 1;

    for (int i = 0; i < 512; i++) {

        if (chrarray[i] == 0x90)
            Nop();
    }
}

void InitUart() {

    CloseUART1();
    U1RXR = 0b0010; // U1RX = A4
    RPB3R = 0b0001; // U1TX = B3

    PORTSetPinsDigitalIn(IOPORT_A, BIT_4);
    PORTSetPinsDigitalOut(IOPORT_B, BIT_3);

    /* IEC1bits.U1RXIE = 1; //Enables RX interrupt.
     IFS1bits.U1RXIF = 0; //Clears RX interrupt flag.
     IPC8bits.U1IP = 7;
     IPC8bits.U1IS = 3;*/


    unsigned int UxBRG = 16;

    OpenUART1(UART_EN | UART_NO_PAR_8BIT | UART_1STOPBIT | UART_DIS_ABAUD, UART_RX_OVERRUN_CLEAR | UART_ADR_DETECT_DIS | UART_TX_ENABLE | UART_RX_ENABLE, UxBRG);

    U1MODEbits.BRGH = 1;

    while (true) {
        while (BusyUART1()) Nop();
        WriteUART1(0xaa);
        WriteTimer1(0);
        while (ReadTimer1() < 0xffff) Nop();
    }
}

void InitSPI() {

    AD1CON1bits.ADON = 0;


    OpenSPI1(SPI_MODE32_ON | SPI_SMP_ON | SPI_CKE_ON | MASTER_ENABLE_ON | SEC_PRESCAL_1_1 | PRI_PRESCAL_4_1, SPI_ENABLE);
    RPB13R = 0b0011; // SET RB13 = SDO1

    PORTSetPinsDigitalOut(IOPORT_B, BIT_11); // SDO2
    PORTSetPinsDigitalIn(IOPORT_B, BIT_2); // SDI2
    PORTSetPinsDigitalOut(IOPORT_B, BIT_9); // CS2

    PORTBbits.RB9 = 1;

    OpenSPI2(SPI_MODE8_ON | SPI_CKE_OFF | CLK_POL_ACTIVE_LOW | MASTER_ENABLE_ON | SEC_PRESCAL_1_1 | PRI_PRESCAL_64_1, SPI_ENABLE);

    RPB11R = 0b0100; // SET RB11 = SDO2
    SDI2R = 0b0100; // SET RPB2 = SDI2

    WriteTimer1(0);
    while (ReadTimer1() < 0xfff) Nop();


    do {

        GetSdSector(0x0000);
        GetSdSector(0x2000);
    } while (1);



}
#endif

void InitUART() {

    CloseUART1();
    RPA0R = 0b0001;
    PORTSetPinsDigitalOut(IOPORT_A, BIT_0);
    OpenUART1(UART_EN | UART_NO_PAR_8BIT | UART_1STOPBIT | UART_DIS_ABAUD, UART_RX_ENABLE | UART_RX_OVERRUN_CLEAR | UART_ADR_DETECT_DIS | UART_TX_ENABLE, 51);
}

#ifdef SPIDMA_ON

void InitDMA() {

    DmaChnOpen(DMA_CHANNEL1, DMA_CHN_PRI3, DMA_OPEN_AUTO);

    DmaChnSetEvFlags(DMA_CHANNEL1, DMA_EV_BLOCK_DONE);

    DCH1ECONbits.CHSIRQ = _SPI1_TX_IRQ; // Channel Transfer Start IRQ bits
    DCH1ECONbits.SIRQEN = 1; // Channel Start IRQ Enable bit, 1 =Start channel cell transfer if an interrupt matching CHSIRQ occurs
    DCH1CONbits.CHAEN = 0; // Channel Automatic Enable bit, 0 = Channel is disabled on block transfer complete

    DmaChnSetTxfer(DMA_CHANNEL1, (void*) &spidata[0], (void*) &SPI1BUF, spidataSize * sizeof (*spidata), 4, 1);
}
#endif

#ifdef SPIMODE_ON

// Sends bitmap buffer to the LEDS using SPI

void sendBitmapBuffer() {

    char spipos = 0;
    unsigned int *p, *buf;
    unsigned int pixel;
    buf = bitmapBuffer;

    p = spidata;
    *p = 0;

    for (int pos = COLS * ROWS - 1; pos >= 0; pos--, buf++) {

        pixel = *buf;

#ifdef GAMMACORRECTION_ON
        makeGamma(pixel);
#endif

        for (char bitpos = 23; bitpos >= 0; bitpos--) {

            if ((pixel & (0x00000001 << bitpos)) > 0)
                *p |= (SPI_HIGH >> spipos);
            else
                *p |= (SPI_LOW >> spipos);

            spipos += SPI_BITSPERBIT;

            if (spipos > 31) {
                *(++p) = 0;
                spipos = spipos - 32;
            }
        }
    }

#ifdef SPIDMA_ON
    DCH1CONbits.CHEN = 1;
#else
    putsSPI1(spidataSize, spidata);
#endif

}
#else

// Old bitbanging routine

void bitbangBitmapPixel(unsigned int x) {

    char i = 24;

    do {
        if ((x >> --i) & 1) {
            LED_HIGH
            Nop();
            Nop();
            Nop();
            Nop();
            Nop();
            LED_LOW
        } else {
            LED_HIGH
            Nop();
            Nop();
            LED_LOW
            Nop();
            Nop();
            Nop();
            Nop();
        }
    } while (i > 0);
}

void sendBitmapBuffer() {

    for (int i = 0; i < COLS * ROWS; i++)
        bitbangBitmapPixel(bitmapBuffer[i]);
}
#endif

// Returns bitmap buffer position for X and Y

short getPixelMappingPosition(short x, short y) {

    //Since I have 8x8 leds connected upper left -> lower left -> upper right -> lower right this will calculate correct position
    if (x < 8)
        return y * 8 + x;
    else
        return (y + 15)*8 + x;

    // Use this if leds are connected left to right and downwards

    return y * COLS + x;
}

// returns alpha, green, red, blue that'll work with the WS2811, but for simplicity is called ARGB

unsigned int getARGB(unsigned char alpha, unsigned char r, unsigned char g, unsigned char b) {

    return (alpha << 24) | (g << 16) | (r << 8) | b;
}

// converts alpha, red, green and blue to alpha, green, red and blue

unsigned int getARGB(unsigned int i) {

    return (i & 0xff000000) | ((i & 0x00ff0000) >> 8) | ((i & 0x0000ff00) << 8) | (i & 0x000000ff);
}

unsigned int* getPixel(char x, char y) {

    return &bitmapBuffer[getPixelMappingPosition(x, y)];
}

// Blends the color with the bitmap buffer

unsigned int alphaBlend(char x, char y, unsigned int newColor) {

    if (newColor >> 24 == 0xff) return newColor;

    unsigned int* oldColor = getPixel(x, y);

    unsigned int alpha = (newColor >> 24) & 0xff;

    unsigned int red = (((newColor >> 8) & 0xff) * alpha / 255) + (((((*oldColor >> 8) & 0xff)) * (255 - alpha)) / 255);
    unsigned int green = (((newColor >> 16) & 0xff) * alpha / 255) + (((((*oldColor >> 16) & 0xff)) * (255 - alpha)) / 255);
    unsigned int blue = (((newColor) & 0xff) * alpha / 255) + (((((*oldColor) & 0xff)) * (255 - alpha)) / 255);

    return (green << 16 | red << 8 | blue);
}

// Sets alpha value of color

void setAlpha(unsigned char alpha, unsigned int * color) {

    *color = (*color & 0x00ffffff) | (alpha << 24);
}

// Returns alpha value of color

unsigned int getAlpha(unsigned char *alpha, unsigned int color) {

    return (color & 0x00ffffff) | (*alpha << 24);
}

// Sends pixel to the bitmap buffer

void setPixel(short x, short y, unsigned int color) {

    if (x < 0 || y < 0 || x >= COLS || y >= ROWS) return; // pixels outside the boundaries are ignored

    bitmapBuffer[getPixelMappingPosition(x, y)] = alphaBlend(x, y, color);
}

// Fills the buffer with specified color

void setBackground(unsigned int color) {

    for (unsigned int i = 0; i < COLS * ROWS; i++)
        bitmapBuffer[i] = color;
}

// Creates a solid rectangle

void fillRectangle(char x1, char y1, char x2, char y2, unsigned int color) {

    for (int x = x1; x <= x2; x++)
        for (int y = y1; y <= y2; y++)
            setPixel(x, y, color);
}

// Draws a line

void drawLine(char x1, char y1, char x2, char y2, unsigned int color) {

    short xSteps = x2 > x1 ? (x2 - x1) : (x1 - x2);
    short ySteps = y2 > y1 ? (y2 - y1) : (y1 - y2);

    float xPosInc = (float) (x2 - x1) / (xSteps > ySteps ? xSteps : ySteps);
    float yPosInc = (float) (y2 - y1) / (xSteps > ySteps ? xSteps : ySteps);

    float xpos = x1;
    float ypos = y1;

    for (int i = 0; i < (xSteps > ySteps ? xSteps : ySteps); i++) {

        setPixel(round(xpos), round(ypos), color);

        xpos += xPosInc;
        ypos += yPosInc;
    }
}

// Draws the outlines of a rectangle

void drawRectangle(char x1, char y1, char x2, char y2, unsigned int color) {

    drawLine(x1, y1, x2, y1, color); // top
    drawLine(x1, y2, x2, y2, color); // bottom
    drawLine(x1, y1, x1, y2, color); // left
    drawLine(x2, y1, x2, y2, color); // right
}

void drawSquareRotate(float centerX, float centerY, float size, float rotate, unsigned int color) {

    float rot = -0.7855 + (((float) rotate / 360) * 6.284);

    char x1 = round(centerX + size * cos(rot));
    char y1 = round(centerY + size * sin(rot));

    char x2 = round(centerX + size * cos(rot + (float) 1.571));
    char y2 = round(centerY + size * sin(rot + (float) 1.571));

    char x3 = round(centerX + size * cos(rot + (float) 3.142));
    char y3 = round(centerY + size * sin(rot + (float) 3.142));

    char x4 = round(centerX + size * cos(rot + (float) 4.713));
    char y4 = round(centerY + size * sin(rot + (float) 4.713));

    drawLine(x1, y1, x2, y2, color);
    drawLine(x2, y2, x3, y3, color);
    drawLine(x3, y3, x4, y4, color);
    drawLine(x4, y4, x1, y1, color);
}

// Not optimized but does the job

void drawCircle(char x, char y, char radius, unsigned int color) {

    for (float angle = 0; angle < 3.142 * 2; angle += 0.1)
        setPixel(x + round(radius * cos(angle)), y + round(radius * sin(angle)), color);
}

int main() {

    SYSTEMConfigPerformance(80000000L);
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    mJTAGPortEnable(DEBUG_JTAGPORT_OFF);
    OpenTimer1(T1_ON | T1_PS_1_256, 0xffffffff);

    InitUart();

#ifdef SPIMODE_ON
    InitSPI();
#else
    PORTSetPinsDigitalOut(LED_PORT, LED_PIN);
#endif


#ifdef SPIDMA_ON
    InitDMA();
    InitUART();
#endif


#ifdef GAMMACORRECTION_ON
    // Calculates gamma values
    for (float f = 0; f < 256; f++) {
        ledTweak[(unsigned char) f] = (unsigned char) round(
                (f * (f / 256) + 1)
                );
    }

    ledTweak[0] = 0;
#endif

    setBackground(0xff000000);

    while (1) {
        for (short i = 0; i < 90; i += 3) {

            drawSquareRotate(7.5, 7.5, 9 - (i / 10), i, getARGB(100, 0, 0, 255));
            drawSquareRotate(7.5, 7.5, i / 10, 90 - i, getARGB(100, 0, 255, 0));


            sendBitmapBuffer();

            fillRectangle(0, 0, 15, 15, getARGB(0x20, 0, 0, 0));

            while (ReadTimer1() < 0x3ff) Nop();
            WriteTimer1(0);
        }
    }

    return 0;
}