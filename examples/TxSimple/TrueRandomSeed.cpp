// TrueRandomSeed.ino
// This example sketch shows how to provide a truly random seed value to the built in
// library pseudo random number generator. This ensures that your sketch will be
// using a different sequence of random numbers every time it runs. Unlike the
// usually suggested randomSeed(analogRead(0)) this method will provide a much more
// uniform and varied seed value. For more information about the basic technique used
// here to produce a random number or if you need more than one such number you can
// find a library, Entropy from the following web site along with documentation of how
// the library has been tested to provide TRUE random numbers on a variety of AVR
// chips and arduino environments.
//
// https://sites.google.com/site/astudyofentropy/project-definition/
// timer-jitter-entropy-sources/entropy-library
//
// Copyright 2014 by Walter Anderson, wandrson01 at gmail dot com
//

#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/atomic.h>
// The following addresses a problem in version 1.0.5 and earlier of the Arduino IDE
// that prevents randomSeed from working properly.
// https://github.com/arduino/Arduino/issues/575
#define randomSeed(s) srandom(s)

volatile uint32_t seed; // These two variables can be reused in your program after the
volatile int8_t nrot; // function CreateTrulyRandomSeed() executes in the setup() function

uint32_t createTrulyRandomSeed()
{
    seed = 0;
    nrot = 8; // Must be at least 4, but more increased the uniformity of the produced
    // seeds entropy.
    // The following five lines of code turn on the watch dog timer interrupt to create
    // the seed value
    cli();
    MCUSR = 0;
    _WD_CONTROL_REG |= (1<<_WD_CHANGE_BIT) | (1<<WDE);
    _WD_CONTROL_REG = (1<<WDIE);
    sei();
 
    while (nrot > 0); // wait here until seed is created
 
    // The following five lines turn off the watch dog timer interrupt
    cli();
    MCUSR = 0;
    _WD_CONTROL_REG |= (1<<_WD_CHANGE_BIT) | (0<<WDE);
    _WD_CONTROL_REG = (0<< WDIE);
    sei();
    return seed;
}

ISR(WDT_vect)
{
    nrot--;
    seed = seed << 8;
    seed = seed ^ TCNT1L;
}
