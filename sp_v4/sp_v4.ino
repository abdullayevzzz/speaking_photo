#include <avr/sleep.h>
#include <avr/interrupt.h>
#include "LemonSerial.h"

// Tracks for Nausica of the Valley Wind
#define NUMBER_OF_TRACKS 22
#define NUMBER_OF_FAVORITES 13
#define FAVORITE_BIAS 60 // How much do you favor your favorites (in percentage)
// Length of each track in seconds. Name of the tracks are "str(index+1).mp3"
uint8_t tracks[NUMBER_OF_TRACKS] = {96, 60, 61, 151, 68, 104, 77, 146, 41, 53, 32, 71, 49, 71, 87, 110, 107, 63, 43, 127, 123, 52};
uint8_t favoriteTracksIndices[NUMBER_OF_FAVORITES] = {0, 1, 2, 2, 2, 3, 3, 4, 5, 5, 5, 7, 19};

// int switchPin = 3;
int sensePin = 2;
int mosfetPin = 0;
int playPin = 1;
int txPin = 3;
int rxPin = 4;
LemonSerial mySerial(rxPin, txPin);

char randomPlayCommand[] = "AT+PLAYFILE=/00.mp3\r\n\0";
uint32_t trackLength = 0;

void setup()
{   
    // Setup the pins.
    pinMode(sensePin, INPUT_PULLUP);
    pinMode(mosfetPin, OUTPUT);
    pinMode(playPin, INPUT); // Necessary for serial comm

    // Setup serial
    mySerial.begin(9675);  // calibrated

    digitalWrite(mosfetPin, LOW);  // Turn on DFPlayer
    delay(2000);
    
    mySerial.println("AT+LED=ON\r\n");
    delay(1000);
    mySerial.println("AT+LED=OFF\r\n");
    
    delay(100);
    digitalWrite(mosfetPin, HIGH);  // Turn off DFPlayer
    
    ADCSRA &= ~_BV(ADEN); // ADC off (saves power)

    unsigned int randomCounter = 0;
    while(digitalRead(sensePin) == HIGH){
      randomCounter++;
      delay(1);
    }  // Wait for the first input to set random seed
    randomSeed(randomCounter);

    delay(5000);  // time to hang the frame)
}


// Loop runs over and over, but only when the microcontroller is awake.
void loop()
{
    // This function will choose next random song and modify randomPlayCommand
    prepareNextTrack();
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

void prepareNextTrack(void){
    uint8_t trackIndex;
    // Decide whether to pick from favorite tracks or from all tracks
    if (random(100) <= FAVORITE_BIAS) { //FAVORITE_BIAS% of chance to choose from favorites
        // Pick a random index from the favoriteTracksIndices array
        trackIndex = favoriteTracksIndices[random(NUMBER_OF_FAVORITES)];
    } else {
        // Pick a random track number from all tracks
        trackIndex = random(NUMBER_OF_TRACKS);
    }

    // Calculate the tens and ones place of the track number
    uint8_t tens = (trackIndex + 1) / 10;
    uint8_t ones = (trackIndex + 1) % 10;
    randomPlayCommand[13] = '0' + tens;
    randomPlayCommand[14] = '0' + ones;

    trackLength = tracks[trackIndex];
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
    digitalWrite(mosfetPin, LOW);  // Turn on DFPlayer
    delay(2000); // Wait for a bit for DFPlayer start
    
    // configure DFPlayer
    mySerial.println("AT+VOL=6\r\n");
    delay(100);
    mySerial.println("AT+PLAYMODE=3\r\n");  // play one song and pause
    delay(100);
    mySerial.println("AT+AMP=ON\r\n");  // turn on amplifier
    delay(100);

    // play the selected song
    mySerial.println(randomPlayCommand); // play the random song
    delay(1000 * trackLength);  // wait until the end of the song
    
    mySerial.println("AT+AMP=OFF\r\n");  // turn of amplifier
    delay(100);
    digitalWrite(mosfetPin, HIGH);  // Turn off DFPlayer
    delay(5000); // Wait before next input
}
