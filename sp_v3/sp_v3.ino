#include <avr/sleep.h>
#include <avr/interrupt.h>
#include "LemonSerial.h"

// int switchPin = 3;
int sensePin = 2;
int mosfetPin = 0;
int playPin = 1;
int txPin = 3;
int rxPin = 4;
LemonSerial mySerial(rxPin, txPin);

// Setup runs 1 time when the microcontroller first boots up.
void setup()
{
    // Setup the pins.
    pinMode(sensePin, INPUT_PULLUP);
    pinMode(mosfetPin, OUTPUT);
    pinMode(playPin, INPUT); // Necessary for serial comm

    
    // Setup serial
    mySerial.begin(9700);  // calibrated
    //delay(1000);
    //!!!! pinMode(rxPin, INPUT);
    
    /*
    // test
    pinMode(playPin, INPUT);
    while(1){
    mySerial.println("AT+LED=ON\r\n");
    delay(1000);
    mySerial.println("AT+LED=OFF\r\n");
    delay(1000);
    }
    */

    //digitalWrite(playPin, HIGH);
    digitalWrite(mosfetPin, LOW);  // Turn on DFPlayer
    delay(2000);
    mySerial.println("AT+LED=OFF\r\n");
    delay(100);   
    mySerial.println("AT+VOL=25\r\n");
    delay(100);
    
    /*
    for (int i=0; i<5; i++){
      mySerial.println("AT+LED=ON\r\n");
      delay(500);  
      mySerial.println("AT+LED=OFF\r\n");
      delay(500);
    }
    */
    
    // digitalWrite(playPin, HIGH);
    digitalWrite(mosfetPin, HIGH);  // Turn on DFPlayer

    ADCSRA &= ~_BV(ADEN);                   // ADC off (saves power)
}

// Loop runs over and over, but only when the microcontroller is awake.
void loop()
{
    // This function will put the microcontroller into a lower power sleep state
    // and it will stop processing code, waiting for the button to be pressed.
    sleep();

    // After the button is pressed and wakes up the microcontroller, this function will start DFPlayer.
    playMusic();
}

ISR(PCINT0_vect)
{
    // This is the interrupt service routine, the code that will run when the microcontroller
    // wakes from sleep due to the button press.  We don't need to do anything here but having
    // this function is required.
    cli();                                  // Disable interrupts
    GIMSK &= ~_BV(PCIE);                    // Disable Pin Change Interrupts
    PCMSK &= ~_BV(PCINT2);                  // Turn off PB2 as interrupt pin
    GIFR |= _BV(PCIF);                      // Clear Pin Change Interrupt Flag by writing 1
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
    GIMSK &= ~_BV(PCIE);                    // Disable Pin Change Interrupts
    PCMSK &= ~_BV(PCINT2);                  // Turn off PB2 as interrupt pin
    GIFR |= _BV(PCIF);                      // Clear Pin Change Interrupt Flag by writing 1
    
    sleep_disable();                        // Clear SE bit
    sei();                                  // Enable interrupts
}

void playMusic(void)
{   
    digitalWrite(sensePin, HIGH);   // Set pull up resistor for sense pin
    // digitalWrite(playPin, HIGH);  // Set playPin HIGH to prepare for trigger
    digitalWrite(mosfetPin, LOW);  // Turn on DFPlayer
    delay(2000); // Wait for a bit for DFPlayer start

    /*
    // play the song
    digitalWrite(playPin, LOW);
    delay(200);
    digitalWrite(playPin, HIGH);
    */
    
    // play the song
    mySerial.println("AT+PLAYMODE=4\r\n");
    delay(100);  
    mySerial.println("AT+PLAY=PP\r\n");
    
    int playLimit = 100;  // after last detect play time limit (x100ms)
    int playTimer = 0;  // after last detect play timer
    long totalPlayLimit = 5000; // total play time limit (x100ms)
    long totalPlayTimer = 0; // total play timer
    while(playTimer < playLimit  &&  totalPlayTimer < totalPlayLimit){
      delay(100);
      totalPlayTimer++;
      if(digitalRead(sensePin) == LOW)
        playTimer = 0;
      else
        playTimer++;
    }
    
    digitalWrite(mosfetPin, HIGH);  // Turn off DFPlayer
    // digitalWrite(playPin, LOW); // Set playPin low for power consumption
    delay(5000); // Wait before next input
}
