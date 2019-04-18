#include <OneWire.h>


//#include<SoftwareSerial.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/wdt.h> //Needed to enable/disable watch dog timer
#include <Adafruit_NeoPixel.h>

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN        PB1 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 1 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


#define PERIPHERALS_OFF 0
#define PERIPHERALS_ON 1
#define UNSAFE_HIGH_TEMPERATURE 60
#define SAFE_HIGH_TEMPERATURE 55
#define SHUTDOWN_TEMPERATURE 40
#define ONEWIRE_BUSS 0
#define RED_LED PB1
#define DESCENDING true
#define ASCENDING false

#define MOSFET_SWITCH PB4
//#define GREEN_LED PB3
#define SLEEPAFTER 10000


uint8_t currentTemperature = 0;
uint8_t lightState = HIGH;
boolean crossedHighThreshold = false;


OneWire TemperatureSensor(ONEWIRE_BUSS);

//#define RX PB5
//#define TX PB2
//SoftwareSerial mySerial(RX, TX); // RX, TX




boolean canSleepAgain = true;
void setup(void) {
  ADCSRA &= ~(1 << ADEN); //Disable ADC, saves ~230uA  
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)  

  //Power down various bits of hardware to lower power usage
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); //Power down everything, wake up from WDT
  sleep_enable();

  pixels.setPixelColor(0, pixels.Color(64, 0, 64));

  pixels.show();   // Send the updated pixel colors to the hardware.
  delay(1000);

  pixels.setPixelColor(0, pixels.Color(0, 0, 0));

  pixels.show();   // Send the updated pixel colors to the hardware.


}
int i;

uint8_t measuredTemperatureAtWakeUp = 0;


boolean justWokeUp = true;
boolean wokeUpByError = false;

unsigned long int wokeUpAt = millis();

void loop(void)
{
  if (canSleepAgain)
  {
    canSleepAgain = false;
    sleep();
    justWokeUp = true;
  }

  if (justWokeUp)
  {
    // User should expect a Purple Color Pulse, if it is Red, the power has gone below the threshold of the BLUE LED and the device needs charging
    pixels.setPixelColor(0, pixels.Color(64, 0, 64));  
    pixels.show();
    delay(250);
    
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();
     
    justWokeUp = false;
    wokeUpByError = false;
    wokeUpAt = millis();
    measuredTemperatureAtWakeUp = 0;
  }

  currentTemperature = getTemperature();

  if (measuredTemperatureAtWakeUp == 0)
  {
    measuredTemperatureAtWakeUp = currentTemperature; //Save the temperature that was measured at wakeup.
  }

  //It is possible that the wakeup may be inadvertant, so we should check if the temperature is increasing since wakeup
  if ( (millis() - wokeUpAt) > SLEEPAFTER && (wokeUpByError == false) && (currentTemperature < SHUTDOWN_TEMPERATURE) ) // See if SLEEPAFTER milliseconds have passed since last wakeup.
  {
    int temperatureDifference = abs(currentTemperature - measuredTemperatureAtWakeUp); // Is there a difference in temperature since wakeup
    if (temperatureDifference > 5) // We want to see if the tempearature changed by 5C in last 10 seconds.
    {
      wokeUpByError = false; // the temperature has changed, hence the user wishes to measure the temperature
    }
    else
    {
      wokeUpByError = true; //Temperature has not changed by much.
    }
  }

  

  if (wokeUpByError && (currentTemperature < SHUTDOWN_TEMPERATURE))
  {
    canSleepAgain = true;
    justWokeUp = false;
    pixels.setPixelColor(0, pixels.Color(150, 150, 0));
    pixels.show();
    delay(100);
    
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();
    return;
  }



  if (currentTemperature > UNSAFE_HIGH_TEMPERATURE)
  {
    
    pixels.setPixelColor(0, pixels.Color(150, 0, 0));
    pixels.show();   // Send the updated pixel colors to the hardware.
    canSleepAgain = false;
    crossedHighThreshold = true;
  }

  else if (currentTemperature <= UNSAFE_HIGH_TEMPERATURE && currentTemperature > SAFE_HIGH_TEMPERATURE &&  crossedHighThreshold)
  {
    
    pixels.setPixelColor(0, pixels.Color(0, 0, 150));
    pixels.show();   // Send the updated pixel colors to the hardware.
    
    canSleepAgain = false;
  }

  else if (currentTemperature <= SAFE_HIGH_TEMPERATURE && currentTemperature > SHUTDOWN_TEMPERATURE && crossedHighThreshold)
  {   
    pixels.setPixelColor(0, pixels.Color(0, 150, 0));
    pixels.show();   // Send the updated pixel colors to the hardware.
    canSleepAgain = false;
  }
  else if (currentTemperature <= SHUTDOWN_TEMPERATURE && crossedHighThreshold)
  {
    canSleepAgain = true;
    crossedHighThreshold = false;
  }
  delay(125);// Keep the state of the LED Output for this long
  setPeripheralState(PERIPHERALS_OFF);// Enable the mosfet to switch off All Devices including LEDS and Sensors. This also turns off the LED
  setup_watchdog(6); //Setup watchdog to go off after 1sec
  sleep_mode(); //Go to sleep by Watchdog! Wake up 1sec later
  setPeripheralState(PERIPHERALS_ON); //Turn on the Peripherals
  init_peripherals();// Initialise Peripherals that have shut down
  


}

void sleep() {

  GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
  PCMSK |= _BV(PCINT2);                   // Use PB2 as interrupt pin
  ADCSRA &= ~_BV(ADEN);                   // ADC off
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // replaces above statement

  sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
  sei();                                  // Enable interrupts
  sleep_cpu();                            // sleep

  cli();                                  // Disable interrupts
  PCMSK &= ~_BV(PCINT2);                  // Turn off PB2 as interrupt pin
  sleep_disable();                        // Clear SE bit
  sei();                                  // Enable interrupts




}

ISR(PCINT0_vect) {

}

ISR(WDT_vect) {
  
 
}

uint8_t getTemperature()
{
  uint8_t data[9];
  uint16_t raw;
  uint8_t t;
  TemperatureSensor.reset(); // reset one wire buss
  TemperatureSensor.skip(); // select only device
  TemperatureSensor.write(0x44); // start conversion
  //TemperatureSensor.setResolution()
  //delay(1000); // wait for the conversion
  TemperatureSensor.reset();
  TemperatureSensor.skip();
  TemperatureSensor.write(0xBE); // Read Scratchpad

  for ( i = 0; i < 9; i++) { // 9 bytes

    data[i] = TemperatureSensor.read();

  }

  raw = (data[1] << 8) | data[0];

  t =  (uint8_t)(raw >> 4);
  
  // disable the Temperature Sensor 
  pinMode(ONEWIRE_BUSS, INPUT);//This will Float the Sensor pin, essentially pulling it up. The sensor goes into low power mode.
  return t;

}



//Sets the watchdog timer to wake us up, but not reset
//0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
//6=1sec, 7=2sec, 8=4sec, 9=8sec
//From: http://interface.khm.de/index.php/lab/experiments/sleep_watchdog_battery/
void setup_watchdog(int timerPrescaler) {

  if (timerPrescaler > 9 ) timerPrescaler = 9; //Limit incoming amount to legal settings

  byte bb = timerPrescaler & 7; 
  if (timerPrescaler > 7) bb |= (1<<5); //Set the special 5th bit if necessary

  //This order of commands is important and cannot be combined
  MCUSR &= ~(1<<WDRF); //Clear the watch dog reset
  WDTCR |= (1<<WDCE) | (1<<WDE); //Set WD_change enable, set WD enable
  WDTCR = bb; //Set new watchdog timeout value
  WDTCR |= _BV(WDIE); //Set the interrupt enable, this will keep unit from resetting after each int
}

void setPeripheralState(boolean state)
{
  pinMode(MOSFET_SWITCH, state);
}

void init_peripherals()
{
  pixels.begin(); 
}
