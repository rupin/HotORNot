#include <OneWire.h>


#include<SoftwareSerial.h>
#include <avr/sleep.h> //Needed for sleep_mode
#include <avr/wdt.h> //Needed to enable/disable watch dog timer


#define UNSAFE_HIGH_TEMPERATURE 60
#define SAFE_HIGH_TEMPERATURE 55
#define SHUTDOWN_TEMPERATURE 40
#define ONEWIRE_BUSS 0
#define RED_LED PB1
#define DESCENDING true
#define ASCENDING false

#define AMBER_LED PB4
#define GREEN_LED PB3

uint8_t lastTemperature = 0;
uint8_t currentTemperature = 0;
int temperatureDifference = 0;
uint8_t lightState = HIGH;
boolean temperatureDirection = false;


OneWire TemperatureSensor(ONEWIRE_BUSS);

#define RX PB5
#define TX PB2
SoftwareSerial mySerial(RX, TX); // RX, TX

uint8_t data[9];
uint16_t raw;
uint8_t t;
int temperatureGradient = ASCENDING;

void setup(void) {
  ADCSRA &= ~(1 << ADEN); //Disable ADC, saves ~230uA
  pinMode(RED_LED, OUTPUT);
  pinMode(AMBER_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  digitalWrite(RED_LED, LOW);
  digitalWrite(AMBER_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  mySerial.begin(9600);
  //mySerial.println("Hello, world?");

  //Power down various bits of hardware to lower power usage  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); //Power down everything, wake up from WDT
  sleep_enable();


}
int i;
int watchdog_counter = 11;
void loop(void)
{

  ADCSRA &= ~(1<<ADEN); //Disable ADC, saves ~230uA
  TemperatureSensor.reset(); // reset one wire, puts the bus low, and the sensor in Low Power mode.
  setup_watchdog(6); //Setup watchdog to go off after 1sec
  sleep_mode(); //Go to sleep! Wake up 1sec later and check water
  if (watchdog_counter > 1)
  {

    watchdog_counter=0; 
    wdt_disable(); //Turn off the WDT!!
    TemperatureSensor.reset(); // reset one wire buss
    TemperatureSensor.skip(); // select only device
    TemperatureSensor.write(0x44); // start conversion
    TemperatureSensor.setResolution()
    //delay(1000); // wait for the conversion
    TemperatureSensor.reset();
    TemperatureSensor.skip();

    TemperatureSensor.write(0xBE); // Read Scratchpad

    for ( i = 0; i < 9; i++) { // 9 bytes

      data[i] = TemperatureSensor.read();

    }

    raw = (data[1] << 8) | data[0];
    //currentTemperature = (uint8_t)raw / 16.0; //divide by 16
    currentTemperature = (uint8_t)(raw >> 4);



    temperatureDifference = currentTemperature - lastTemperature;

    if (temperatureDifference > 0)
    {
      temperatureDirection = ASCENDING;
    }
    else if (temperatureDifference < 0)
    {
      temperatureDirection = DESCENDING;
    }
    else
    {
      temperatureDirection = DESCENDING;
    }


    mySerial.print(currentTemperature);
    mySerial.print("\t");
    mySerial.print(lastTemperature);
    mySerial.print("\t");
    mySerial.println(temperatureDirection);

    

    if (temperatureDirection == DESCENDING)
    {

      //lightState = !lightState;


      if (currentTemperature > UNSAFE_HIGH_TEMPERATURE)
      {
        digitalWrite(RED_LED, lightState);
        digitalWrite(AMBER_LED, LOW);
        digitalWrite(GREEN_LED, LOW);
      }
      else if (currentTemperature < SAFE_HIGH_TEMPERATURE)
      {
        digitalWrite(RED_LED, LOW);
        digitalWrite(AMBER_LED, LOW);
        digitalWrite(GREEN_LED, lightState);
      }
      else
      {
        digitalWrite(RED_LED, LOW);
        digitalWrite(AMBER_LED, lightState);
        digitalWrite(GREEN_LED, LOW);
      }

      delay(100);
      digitalWrite(RED_LED, LOW);
      digitalWrite(AMBER_LED, LOW);
      digitalWrite(GREEN_LED, LOW);

    }
    lastTemperature = currentTemperature;



  }

}

ISR(WDT_vect) {
  mySerial.println("Waking");
  watchdog_counter++;
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
