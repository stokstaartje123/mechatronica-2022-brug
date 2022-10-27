#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define delaySl 5
#define delay 50

#define min1     949
#define mid1     2999
#define max1     4999

#define min2     949
#define mid2     2999
#define max2     4999

#define MotorSpeed      80      //percentage tijd aan motor
#define FREQ3           39999   //20ms servo
#define FREQ4           7811    //1hz  leds
#define FREQ5           39999   //20ms motor

//----------VinitsV----------//
static inline void TimersInit(void)
{
    //Timer 3, slagbomen, mode 14, prescaler 8
    TCCR3A |= (1<<COM3A1) | (1<<COM3B1) | (1<<WGM31);
    TCCR3B |= (1<<WGM33) | (1<<WGM32) | (1<<CS31);
    ICR3 = FREQ3;
    DDRE |= (1 << PE4); //pin 2
    DDRE |= (1 << PE3); //pin 5

    //Timer 5, DC motor, mode 14, prescaler 8
    TCCR5A |= (1<<COM5A1) | (1<<WGM51);
    TCCR5B |= (1<<WGM53) | (1<<WGM52) | (1<<CS51 );
    ICR5 = FREQ5;
    DDRL |= (1 << PL3); //pin 46

    //Timer 4, leds, mode 14, prescaler 1024
    TCCR4A |= (1<<WGM41);
    TCCR4B |= (1<<WGM43) | (1<<WGM42);  //de prescaler staat later in de code
    TIMSK4 |= (1<<TOIE4);
    ICR4 = FREQ4;
    sei();
}

ISR(TIMER4_OVF_vect)
{
    PORTA ^= (1 << PA2);
    PORTA ^= (1 << PA3);
}

static inline void PinsInit(void)
{
    DDRA |= (1 << PA2);     //22 knipperset1
    DDRA |= (1 << PA3);     //23 knipperset2
    DDRA |= (1 << PA6);     //28 in1
    DDRA |= (1 << PA7);     //29 in2

    DDRC |= (1 << PC4);     //33  led3 setgroen
    DDRC |= (1 << PC5);     //32  led4 setrood
}
//---------^inits^----------//

static inline void slagboom1(uint8_t dicht)
{
    if(dicht)
    {
        TCCR3B |= (1<<CS31);
        for (int i = mid1 ; i >= min1 ; i-=10)
            {
                OCR3A = i;
                _delay_ms (delaySl);
            }
        TCCR3B &= ~(1<<CS31);
    }
    else
    {
        TCCR3B |= (1<<CS31);
        for (int i = min1 ; i <= mid1 ; i+=10)
        {
            OCR3A = i;
            _delay_ms (delaySl);
        }
        TCCR3B &= ~(1<<CS31);
    }
}

static inline void slagboom2(uint8_t dicht)
{
    if(dicht)
    {
        TCCR3B |= (1<<CS31);
        for (int i = mid2 ; i >= min2 ; i-=10)
            {
                OCR3B = i;
                _delay_ms (delaySl);
            }
        TCCR3B &= ~(1<<CS31);
    }
    else
    {
        TCCR3B |= (1<<CS31);
        for (int i = min2 ; i <= mid2 ; i+=10)
        {
            OCR3B = i;
            _delay_ms (delaySl);
        }
        TCCR3B &= ~(1<<CS31);
    }
}

static inline void motorStand(uint8_t open, uint8_t dicht)
{
    if (open)
    {
        PORTA |= (1 << PA6);
        OCR5A = ((MotorSpeed*FREQ5)/100);
    }
    else if (dicht)
    {
        PORTA |= (1 << PA7);
        OCR5A = ((MotorSpeed*FREQ5)/100);
    }
    PORTA &= ~(1 << PA6); //alles van motor uit
    PORTA &= ~(1 << PA7);
    OCR5A = 0;
}

static inline void knipperled(uint8_t aan)
{
    if(aan)
    {
        PORTA |= (1 << PA3);    //begin positie leds
        PORTA &= ~(1 << PA2);

        TCCR4B |= (1<<CS42) | (1<<CS40);    //timer aan
    }
    else
    {
        PORTA &= ~(1 << PA3);
        PORTA &= ~(1 << PA2);

        TCCR4B &= ~(1<<CS42);
        TCCR4B &= ~(1<<CS40); //timer uit
    }
}

static inline void bootleds(uint8_t doorvaard)
{
    if(doorvaard)
    {
        PORTC |= (1<<PC4);
        PORTC &= ~(1<<PC5);
    }
    else
    {
        PORTC |= (1<<PC5);
        PORTC &= ~(1<<PC4);
    }
}

static inline void veiligheid(uint8_t open)
{
    if(open)
    {
        knipperled(1);
        _delay_ms(1000);
        slagboom1(1);
        slagboom2(1);

    }
    else
    {
        slagboom1(0);
        slagboom2(0);
        _delay_ms(1000);
        knipperled(0);
    }
}

int main(void)
{
    TimersInit();
    PinsInit();

    OCR3A = mid1;
    OCR3B = mid2;

    while(1)
    {
        veiligheid(1);
        motorStand(1,0);
        _delay_ms(2000);

        bootleds(1);
        motorStand(0,0);
        _delay_ms(4000);

        bootleds(0);
        motorStand(0,1);
        _delay_ms(2000);

        motorStand(0,0);
        veiligheid(0);
        _delay_ms(6000);
    }
}
