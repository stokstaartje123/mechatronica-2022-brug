#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define delaySl 5
#define delay 50

#define min1     949
#define mid1     2999
#define max1     4999

#define FREQ3           39999   //20ms servo
#define FREQ4           62499    //leds

//----------VinitsV----------//
static inline void Timer3Init(void)
{
    //Timer 3, slagbomen, mode 14, prescaler 8
    TCCR3A |= (1<<COM3A1) | (1<<COM3B1) | (1<<WGM31);
    TCCR3B |= (1<<WGM33) | (1<<WGM32) | (1<<CS31);
    ICR3 = FREQ3;
    DDRE |= (1 << PE4); //pin 2
    DDRE |= (1 << PE3); //pin 5
}

static inline void Timer4Init(void)
{
    //Timer 4, leds, mode 12, prescaler 1
    TCCR4A |= (1<<COM4A1) | (1<<COM4B1) | (1<<WGM41);
    TCCR4B |= (1<<WGM43) | (1<<WGM42);// | (1<<CS42);
    ICR4 = FREQ4;
    OCR4A = 62499/2;
    OCR4B = 62499/2;
    DDRH |= (1 << PH3); //6 knipperset1
    DDRH |= (1 << PH4); //7 knipperset2
}

static inline void PinsInit(void)
{
    DDRA |= (1 << PA6);     //28 in1
    DDRA |= (1 << PA7);     //29 in2
    DDRH |= (1 << PH6);     //9  conformatie led

    DDRC |= (1 << PC4);     //33  led3 setgroen
    DDRC |= (1 << PC5);     //32  led4 setrood

    DDRF &= ~(1 << PF0);    //A0  sensor 1
    //PORTF |= (1 << PF0);    //A0  pull-up resistor
    DDRF &= ~(1 << PF1);    //A1  sensor 2
    PORTF |= (1 << PF1);    //A1  pull-up resistor

    DDRF &= ~(1 << PF2);    //A2  knop 1 onder
    PORTF |= (1 << PF2);    //A2  pull-up resistor
    DDRF &= ~(1 << PF3);    //A3  knop 2 boven
    PORTF |= (1 << PF3);    //A3  pull-up resistor

    DDRB &= ~(1 << PB5);    //11  resume knop
    PORTB |= (1 << PB5);    //11  pull-up resistor

    DDRH &= ~(1 << PH5);    //8  confirm knop
    PORTH |= (1 << PH5);    //8  pull-up resistor

    PCICR |= (1<< PCIE0);   //10 noodknop + interrupt
    PCMSK0 |= (1<<PCINT4);
    DDRB |= (1 << PB4);
}
//---------^inits^----------//

ISR(PCINT0_vect)
{
    PORTA &= ~(1 << PA6);
    PORTA &= ~(1 << PA7);
    while((PINB & (1 << PB5)));
}

static inline void slagboom1(uint8_t dicht)
{
    if(dicht)
    {
        TCCR3B |= (1<<CS31);
        for (int i = mid1 ; i >= min1 ; i-=10)
            {
                OCR3A = i;
                OCR3B = i;
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
        while(PINF & (1 << PF3))
        {
            PORTA |= (1 << PA6);
        }

    }
    else if (dicht)
    {
        while(PINF & (1 << PF2))
        {
            PORTA |= (1 << PA7);
        }
    }
    else
    {
        PORTA &= ~(1 << PA6);
        PORTA &= ~(1 << PA7);
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
        TCCR4B |= (1<<CS42);
        _delay_ms(1500);
        slagboom1(1);

    }
    else
    {
        slagboom1(0);
        _delay_ms(1500);
        TCCR4B &= ~(1<<CS42);
    }
}

static inline void accepteren(uint8_t i)
{
    if(i)
    {
        PORTH |= (1 << PH6);
        while((PINH & (1 << PH5)));
        PORTH &= ~(1 << PH6);
    }
}

int main(void){

    Timer3Init();
    Timer4Init();
    PinsInit();
    bootleds(0);
    sei();

    OCR3A = mid1;
    OCR3B = mid1;

    while(1)
    {
        if((PINF & (1 << PF0)) == 0)
           {
            accepteren(1);

            veiligheid(1);
            motorStand(1,0);

            motorStand(0,0);
            bootleds(1);

            //wachten totdat het voorbij de tweede is
            while((PINF & (1 << PF1)));
            _delay_ms(50);
            while((PINF & (1 << PF1)) == 0);

            accepteren(1);

            bootleds(0);
            motorStand(0,1);

            motorStand(0,0);
            veiligheid(0);
           }

        if((PINF & (1 << PF1)) == 0)
           {
            accepteren(1);

            veiligheid(1);
            motorStand(1,0);

            motorStand(0,0);
            bootleds(1);

            //wachten totdat het voorbij de tweede is
            while((PINF & (1 << PF0)));
            _delay_ms(50);
            while((PINF & (1 << PF0)) == 0);

            accepteren(1);

            bootleds(0);
            motorStand(0,1);

            motorStand(0,0);
            veiligheid(0);
           }
    }
}
