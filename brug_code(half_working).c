/*
    PA6 28  in1
    PA7 29  in2
    PA3 25  led1 knipperlicht
    PA2 24  led2 knipperlicht
    PA1 23  led3 knipperlicht
    PA0 22  led4 knipperlicht

    PF0 A0  knop1 open
    PF1 A1  knop2 dicht
    PF2 A2  knop3 onder
    PF3 A3  knop4 boven

    PE4 2   slagboom1
    PE4 5   slagboom2

    PL3 46  dc motor

    PC4 33  led5 boot groen 1
    PC5 32  led6 boot rood 1
    PC6 31  led7 boot groen 2
    PC7 30  led8 boot rood 1

 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define delaySl 10
#define delay 50

#define min     999
#define mid     2999
#define max     4999

#define MotorSpeed      40      //lager kan zorgen dat hij geen belasting aan kan
#define FREQ3           39999   //20ms servo
#define FREQ4           7811   //1hz  leds
#define FREQ5           40000   //20ms motor


static inline void TimersInit(void)
{
    //Timer 3, slagbomen, mode 14, prescaler 8
    TCCR3A |= (1<<COM3A1) | (1<<COM3B1) | (1<<WGM31);
    TCCR3B |= (1<<WGM33) | (1<<WGM32) | (1<<CS31);
    ICR3 = FREQ3;
    DDRE |= (1 << PE4); //pin 2
    DDRE |= (1 << PE3); //pin 5

    //Timer 4, leds, mode 14, prescaler 1024
    TCCR4A |= (1<<WGM41);
    TCCR4B |= (1<<WGM43) | (1<<WGM42); // ik zet de prescaler later in de code
    TIMSK4 |= (1<<TOIE4);
    ICR4 = FREQ4;
    sei();

    //Timer 5, DC motor, mode 14, prescaler 8
    TCCR5A |= (1<<COM5A1) | (1<<WGM51);
    TCCR5B |= (1<<WGM53) | (1<<WGM52) | (1<<CS51 );
    ICR5 = FREQ5;
    DDRL |= (1 << PL3); //pin 46
}

ISR(TIMER4_OVF_vect)
{
    PORTA ^= (1 << PA0);
    PORTA ^= (1 << PA1);
    PORTA ^= (1 << PA2);
    PORTA ^= (1 << PA3);
}

static inline void PinsInit(void)
{
    DDRA |= (1 << PA0);     //22 led1
    DDRA |= (1 << PA1);     //23 led2
    DDRA |= (1 << PA2);     //24 led3
    DDRA |= (1 << PA3);     //35 led4
    DDRA |= (1 << PA6);     //28 in1
    DDRA |= (1 << PA7);     //29 in2

    DDRC |= (1 << PC4);     //33  led5
    DDRC |= (1 << PC5);     //32  led6
    DDRC |= (1 << PC6);     //31  led7
    DDRC |= (1 << PC7);     //30  led8

    DDRF &= ~(1 << PF0);    //A0 knop1
    DDRF &= ~(1 << PF1);    //A1 knop2
    DDRF &= ~(1 << PF2);    //A2 knop3
    DDRF &= ~(1 << PF3);    //A3 knop4
}

static inline void slagbomen(uint8_t open)
{
    if(open)
    {
        for (int i = max ; i >= mid ; i-=10)
        {
            OCR3B = i;
            OCR3A = i;
            _delay_ms (delaySl);
        }
    }
    else
    {
        for (int i = mid ; i <= max ; i+=10)
        {
            OCR3B = i;
            OCR3A = i;
            _delay_ms (delaySl);
        }
    }
}

static inline void motorStand(uint8_t aan, uint8_t uit)
{
    if (aan)
    {
        while((PINF & (1 << PF3)) == 0)
        {
            PORTA |= (1 << PA6);
            OCR5A = ((100/MotorSpeed)*FREQ5);
        }
        PORTC |= (1 << PC4);
        PORTC |= (1 << PC6);
    }
    else if (uit)
    {
        while((PINF & (1 << PF2)) ==
        {
            PORTA |= (1 << PA7);
            OCR5A = ((100/MotorSpeed)*FREQ5);
        }
        PORTC |= (1 << PC5);
        PORTC |= (1 << PC7);
    }
    PORTA &= ~(1 << PA6);
    PORTA &= ~(1 << PA7);
    OCR5A = 0;
}

static inline void knipperled(uint8_t aan)
{
    if(aan)
    {
        PORTA |= (1 << PA0);
        PORTA |= (1 << PA2);
        PORTA &= ~(1 << PA1);
        PORTA &= ~(1 << PA3);

        TCCR4B |= (1<<CS42);    //timer aan
        TCCR4B |= (1<<CS40);
    }
    else
    {
        PORTA &= ~(1 << PA0);
        PORTA &= ~(1 << PA1);
        PORTA &= ~(1 << PA2);
        PORTA &= ~(1 << PA3);

        TCCR4B &= ~(1<<CS42);   //timer uit
        TCCR4B &= ~(1<<CS40);
    }
}

static inline void veiligheid(uint8_t open)
{
    if(open)
    {
        knipperled(1);
        slagbomen(1);

    }
    else
    {
        slagbomen(0);
        knipperled(0);
    }
}


int main(void)
{
    TimersInit();
    PinsInit();

    while(1) {
        motorStand(1, 0);
        veiligheid(1);
        _delay_ms(3000);

    }
}
