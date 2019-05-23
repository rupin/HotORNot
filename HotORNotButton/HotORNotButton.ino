#include <SendOnlySoftwareSerial.h>

#include <OneWire.h>


//#include<SoftwareSerial.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/wdt.h> //Needed to enable/disable watch dog timer
#include <Adafruit_NeoPixel.h>


// How many NeoPixels are attached to the Arduino?


// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.



#define PERIPHERALS_OFF 0
#define PERIPHERALS_ON 1
#define UNSAFE_HIGH_TEMPERATURE 60
#define SAFE_HIGH_TEMPERATURE 55
#define SHUTDOWN_TEMPERATURE 40
#define TEMPERATURE_SENSOR PB0

#define WS2812LED PB1
#define NUMPIXELS 2

#define DESCENDING true
#define ASCENDING false

#define MOSFET_GATE PB4

#define SLEEPAFTER 10000


Adafruit_NeoPixel pixels(NUMPIXELS, WS2812LED, NEO_GRB + NEO_KHZ800);


#define PURPLE pixels.Color(30, 0, 30)
#define GREEN pixels.Color(0, 30, 0)
#define RED pixels.Color(30, 0, 0)
#define BLUE pixels.Color(0, 0, 30)
#define YELLOW pixels.Color(30, 30, 0)
#define NO_COLOR pixels.Color(0, 0, 0)

uint8_t currentTemperature = 0;
uint8_t lightState = HIGH;
boolean crossedHighThreshold = false;

uint8_t watchdogCounter = 0;

OneWire TemperatureSensor(TEMPERATURE_SENSOR);


#define TX PB3
SendOnlySoftwareSerial mySerial (TX);  // Tx pin




boolean canSleepAgain = true;
void setup(void) {
  ADCSRA &= ~(1 << ADEN); //Disable ADC, saves ~230uA
  initPeripherals();

  mySerial.begin(9600);
  //Power down various bits of hardware to lower power usage
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); //Power down everything, wake up from WDT
  sleep_enable();
  setTemperatureIndicationColor(GREEN);
  delay(1000);
  setTemperatureIndicationColor(NO_COLOR);



}
int i;

uint8_t measuredTemperatureAtWakeUp = 0;


boolean justWokeUp = true;
boolean wokeUpByError = false;



void loop(void)
{
  putToDeepSleep(canSleepAgain);

  if (justWokeUp)
  {

    initPeripherals();

    // User should expect a Purple Color Pulse, if it is Red, the power has gone below the threshold of the BLUE LED and the device needs charging
    setTemperatureIndicationColor(PURPLE);
    delay(250);
    setTemperatureIndicationColor(NO_COLOR);

    justWokeUp = false;
    wokeUpByError = false;
    measuredTemperatureAtWakeUp = 0;
  }


  currentTemperature = getTemperature();

  if (measuredTemperatureAtWakeUp == 0)
  {
    measuredTemperatureAtWakeUp = currentTemperature; //Save the temperature that was measured at wakeup.
  }
  mySerial.print("MeasuredTemperature at Wakeup: ");
  mySerial.println(measuredTemperatureAtWakeUp);

  mySerial.print("Current Temperature: ");
  mySerial.println(currentTemperature);



  //It is possible that the wakeup may be inadvertant, so we should check if the temperature is increasing since wakeup
  if (watchdogCounter > 3 && (wokeUpByError == false) /*&& (currentTemperature < SHUTDOWN_TEMPERATURE) */) // This will happen when Watchdog timer comes back after 8 seconds
  {

    int temperatureDifference = abs(currentTemperature - measuredTemperatureAtWakeUp); // Is there a difference in temperature since wakeup
    mySerial.print("Temperature Difference: ");
    //mySerial.println(temperatureDifference);
    if (temperatureDifference > 5) // We want to see if the tempearature changed by 5C in last 8 seconds.
    {
      wokeUpByError = false; // the temperature has changed, hence the user wishes to measure the temperature
    }
    else
    {
      wokeUpByError = true; //Temperature has not changed by much.
    }
  }

  /*
    if(wokeUpByError)
    {
      //mySerial.println("Woke Up by Error");
    }
    else
    {
      //mySerial.println("Woke Up correctly");
    }
  */


  if (wokeUpByError && (currentTemperature < SHUTDOWN_TEMPERATURE))
  {
    canSleepAgain = true;
    justWokeUp = false;
    setTemperatureIndicationColor(GREEN);
    delay(250);
    setTemperatureIndicationColor(NO_COLOR);
    return;
  }





  if (currentTemperature > UNSAFE_HIGH_TEMPERATURE)
  {
    setTemperatureIndicationColor(RED);

    canSleepAgain = false;
    crossedHighThreshold = true;
  }

  else if (currentTemperature <= UNSAFE_HIGH_TEMPERATURE && currentTemperature > SAFE_HIGH_TEMPERATURE &&  crossedHighThreshold)
  {
    setTemperatureIndicationColor(YELLOW);
    canSleepAgain = false;
  }

  else if (currentTemperature <= SAFE_HIGH_TEMPERATURE && currentTemperature > SHUTDOWN_TEMPERATURE && crossedHighThreshold)
  {
    setTemperatureIndicationColor(GREEN);
    canSleepAgain = false;
  }
  else if (currentTemperature <= SHUTDOWN_TEMPERATURE && crossedHighThreshold)
  {
    canSleepAgain = true;
    crossedHighThreshold = false;
  }
  delay(125);// Keep the state of the LED Output for this long

  setTemperatureIndicationColor(NO_COLOR);


  //setPeripheralState(PERIPHERALS_OFF);// Enable the mosfet to switch off All Devices including LEDS and Sensors. This also turns off the LED
  //putToDeepSleep(canSleepAgain);

  if (canSleepAgain == false) //If Deep sleep is not enabled, then enable WDT Sleep
  {
    setup_watchdog(7); //Setup watchdog to go off after 2sec
    sleep_mode(); //Go to sleep by Watchdog! Wake up 1sec later
  }





}

void setTemperatureIndicationColor(uint32_t pixelColor)
{
  pixels.setPixelColor(0, pixelColor);
  pixels.setPixelColor(1, pixelColor);
  pixels.show();
}

void sleep() {

  GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
  PCMSK |= _BV(PCINT2);                   // Use PB2 as interrupt pin
  ADCSRA &= ~_BV(ADEN);                   // ADC off
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // replaces above statement
  disablePeripherals();
  sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
  sei();                                  // Enable interrupts
  sleep_cpu();                            // sleep

  cli();                                  // Disable interrupts
  PCMSK &= ~_BV(PCINT2);                  // Turn off PB2 as interrupt pin
  sleep_disable();                        // Clear SE bit
  sei();                                  // Enable interrupts
  initPeripherals();




}

ISR(PCINT0_vect) {
  //mySerial.println("Waking from Deep Sleep");
}

ISR(WDT_vect) {
  //mySerial.println("Waking from WDT Sleep");
  watchdogCounter++;


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
  pinMode(TEMPERATURE_SENSOR, INPUT);//This will Float the Sensor pin, essentially pulling it up. The sensor goes into low power mode.
  return t;

}



//Sets the watchdog timer to wake us up, but not reset
//0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
//6=1sec, 7=2sec, 8=4sec, 9=8sec
//From: http://interface.khm.de/index.php/lab/experiments/sleep_watchdog_battery/
void setup_watchdog(int timerPrescaler) {
  wdt_reset();
  if (timerPrescaler > 9 ) timerPrescaler = 9; //Limit incoming amount to legal settings

  byte bb = timerPrescaler & 7;
  if (timerPrescaler > 7) bb |= (1 << 5); //Set the special 5th bit if necessary

  //This order of commands is important and cannot be combined
  MCUSR &= ~(1 << WDRF); //Clear the watch dog reset
  WDTCR |= (1 << WDCE) | (1 << WDE); //Set WD_change enable, set WD enable
  WDTCR = bb; //Set new watchdog timeout value
  WDTCR |= _BV(WDIE); //Set the interrupt enable, this will keep unit from resetting after each int
}

void enableWDT()
{
  //wdt_reset();
}
void disableWDT()
{
  wdt_disable();
}

void setPeripheralState(boolean state)
{

  pinMode(MOSFET_GATE, state);
}

void initPeripherals()
{
  pinMode(MOSFET_GATE, OUTPUT);
  digitalWrite(MOSFET_GATE, HIGH);
  pixels.begin();
}
void disablePeripherals()
{
  pixels.clear();
  digitalWrite(MOSFET_GATE, LOW);
  pinMode(MOSFET_GATE, INPUT);


}

void putToDeepSleep(boolean sleepFlag)
{
  if (sleepFlag)
  {
    //mySerial.println("Deep Sleep");
    mySerial.flush();
    canSleepAgain = false;

    disableWDT();
    disablePeripherals();// turn off All Peripherals
    sleep();// Device Goes to sleep here, and when woken up, executes from here
    justWokeUp = true;
    watchdogCounter = 0;
    initPeripherals();

  }


}
