#include <avr/sleep.h>
#include <avr/interrupt.h>

// int switchPin = 3;
int sensePin = 2;
int mosfetPin = 0;
int playPin = 1;
int txPin = 3;
int rxPin = 4;

// Setup runs 1 time when the microcontroller first boots up.
void setup()
{
    // Setup the pins.
    pinMode(sensePin, INPUT_PULLUP);
    pinMode(mosfetPin, OUTPUT);
    pinMode(playPin, OUTPUT);

    // Temporary
    pinMode(txPin, INPUT);
    pinMode(rxPin, INPUT);
    // Temporary

    digitalWrite(playPin, HIGH);
    digitalWrite(mosfetPin, HIGH);

    ADCSRA &= ~_BV(ADEN);                   // ADC off (saves power)
}

// Loop runs over and over, but only when the microcontroller is awake.
void loop()
{
    // This function will put the microcontroller into a lower power sleep state
    // and it will stop processing code, waiting for the button to be pressed.
    sleep();

    // After MTCH101 triggered, its output pin wakes up the microcontroller, this function will start DFPlayer.
    playMusic();
}

ISR(PCINT0_vect)
{
    // This is the interrupt service routine, the code that will run when the microcontroller
    // wakes from sleep due to the button press.  We don't need to do anything here but having
    // this function is required.
}

void sleep()
{
    GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
    PCMSK |= _BV(PCINT2);                   // Use PB2 as interrupt pin
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
    sei();                                  // Enable interrupts
    sleep_cpu();                            // Put the microcontroller to sleep

    // Code execution stops here since the microcontroller is now asleep.  When the button connected to PB3
    // is pressed the microcontroller will awake, run the code in the interrupt service routine (ISR),
    // and then continue executing the code starting from here.

    cli();                                  // Disable interrupts

    PCMSK &= ~_BV(PCINT2);                  // Turn off PB3 as interrupt pin
    pinMode(sensePin, INPUT_PULLUP);
    sleep_disable();                        // Clear SE bit
    sei();                                  // Enable interrupts
}

void playMusic(void)
{   
    digitalWrite(playPin, HIGH);  // Set playPin HIGH to prepare for trigger
    digitalWrite(mosfetPin, LOW);  // Turn on DFPlayer
    // delay(500); // possible delay to wait DFPlayer to power up
    // turn of amplifier, if power down save works, turn off after playin and turn on after next delay
    delay(2000); // Wait for a bit for DFPlayer start
    // turn on amplifier

    // play the song
    digitalWrite(playPin, LOW);
    delay(200);
    digitalWrite(playPin, HIGH);

    int playTimer = 100;  // after last detect timer (x100ms)
    int playCounter = 0;  // after last detect timer counter
    
    while(playCounter < playTimer){
      delay(100);
      if(digitalRead(sensePin) == LOW)
        playCounter = 0;
      else
        playCounter++;
    }

    digitalWrite(mosfetPin, HIGH);  // Turn off DFPlayer
    digitalWrite(playPin, LOW); // Set playPin low for power consumption
}
