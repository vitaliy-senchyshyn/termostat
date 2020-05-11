#define LOG64_ENABLED

#include <Log64.h>
#include <OneWire.h>

#define LED_PIN 2
#define COOLER_PIN 3
#define HEATER_PIN 4
#define IN_PIN A5
#define INNER_TEMP_PIN 10
#define OUTER_TEMP_PIN 11

#define HEATING_TEMP 1.0
#define COOLING_TEMP 4.5
#define HYST_TEMP 0.5

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

typedef enum status_t {
  STATUS_SUCCESS = 0,
  STATUS_FAILURE,
  STATUS_INIT_FAILED,
  STATUS_NOT_FOUND,
  STATUS_CRC_CHECK_FAIL,
  STATUS_UNKNOWN_ITEM
} status_t;


class Ds18TempSensor {
  private:
    enum {
      DS_ADDR_SIZE = 8,
      DS_DATA_SIZE = 12,
      DS_DELAY = 1000  // ms
    };
 
    enum {
      DS18S20 = 0x10,
      DS18B20 = 0x28,
      DS1822 = 0x22
    };

    enum {
      DS_STATE_START,
      DS_STATE_INIT_DONE,
      DS_STATE_START_CONVERSION,
      DS_STATE_DELAY,
      DS_STATE_READ_AND_PROCESS
    };
    
    byte addr[DS_ADDR_SIZE];
    byte data[DS_DATA_SIZE];
    byte type;
    OneWire ds;
    float temp;
    uint8_t state;
    uint8_t powerMode;
    unsigned long timeVal;
     
  public:
    Ds18TempSensor(uint8_t pin, uint8_t power = 0): 
        ds(pin), powerMode(power), temp(-150.0), state(DS_STATE_START), timeVal(0) 
    {
    }
    
    status_t init() {
      if (!ds.search(addr)) {
        LOG64_SET("No more addresses.");
        LOG64_NEW_LINE;
        ds.reset_search();
        delay(250);
        return STATUS_NOT_FOUND;
      }

      if (OneWire::crc8(addr, 7) != addr[7]) {
        LOG64_SET("CRC is not valid!");
        LOG64_NEW_LINE;
        return STATUS_CRC_CHECK_FAIL;
      }

      switch (addr[0]) {
        case DS18S20:
        case DS18B20:
        case DS1822:
          break;
        default:
          LOG64_SET("Device is not a DS18x20 family device.");
          LOG64_NEW_LINE;
          return STATUS_UNKNOWN_ITEM;        
      }

      state = DS_STATE_INIT_DONE;
          
      return STATUS_SUCCESS;
    }

    float readTemp(void) {
      return temp;
    }
    
    void loop () {
      switch (state) {
        case DS_STATE_INIT_DONE:
          //LOG64_SET("DS_STATE_INIT_DONE"); LOG64_NEW_LINE;
          state = DS_STATE_START_CONVERSION;
          break;
        case DS_STATE_START_CONVERSION:
          //LOG64_SET("DS_STATE_START_CONVERSION"); LOG64_NEW_LINE;
          ds.reset();
          ds.select(addr);
          ds.write(0x44, powerMode);
          timeVal = millis();
          state = DS_STATE_DELAY;
          break;
        case DS_STATE_DELAY:
          // Go to next state if timeout expired, otherwise stay in the current one. 
          if ((millis() - timeVal) >= DS_DELAY) {
            //LOG64_SET("DS_STATE_DELAY"); LOG64_NEW_LINE;
            state = DS_STATE_READ_AND_PROCESS; 
          }
          break;
        case DS_STATE_READ_AND_PROCESS: {
          //LOG64_SET("DS_STATE_READ_AND_PROCESS"); LOG64_NEW_LINE;
          ds.reset();
          ds.select(addr);    
          ds.write(0xBE);         // Read Scratchpad
          
          for (int8_t i = 0; i < 9; i++) {           // we need 9 bytes
            data[i] = ds.read();
          }
        
          // Convert the data to actual temperature
          // because the result is a 16 bit signed integer, it should
          // be stored to an "int16_t" type, which is always 16 bits
          // even when compiled on a 32 bit processor.
          int16_t raw = (data[1] << 8) | data[0];
          if (addr[0] == DS18S20) {
            raw = raw << 3; // 9 bit resolution default
            if (data[7] == 0x10) {
              // "count remain" gives full 12 bit resolution
              raw = (raw & 0xFFF0) + 12 - data[6];
            }
          } else {
            byte cfg = (data[4] & 0x60);
            // at lower res, the low bits are undefined, so let's zero them
            if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
            else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
            else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
            //// default is 12 bit resolution, 750 ms conversion time
          }
          temp = (float)raw / 16.0;
          //LOG64_SET("Temp= "); LOG64_SET(temp); LOG64_NEW_LINE;
          state = DS_STATE_START_CONVERSION;
          break;
        }         
        default:
          break;
      }
    }
    
    void dump() {
      if (state == DS_STATE_START) {
        Serial.println("Device is not initialized!");
        return;
      }
      
      Serial.print("ROM =");
      for(uint8_t i = 0; i < DS_ADDR_SIZE; i++) {
        Serial.write(' ');
        Serial.print(addr[i], HEX);
      }

      switch (addr[0]) {
        case DS18S20:
          Serial.println("  Chip = DS18S20");
          break;
        case DS18B20:
          Serial.println("  Chip = DS18B20");
          break;
        case DS1822:
          Serial.println("  Chip = DS1822");
          break;
        default:
          Serial.println("Device is not a DS18x20 family device.");
          return;        
      }
   
      Serial.print("Temperature = ");
      Serial.println(temp);
      Serial.println("");
      Serial.println("");
    } 
};




class Termostat {
  private:
    enum {
      TM_COOLER_ON = 0,
      TM_COOLER_OFF = 1,
      TM_HEATER_ON = 0,
      TM_HEATER_OFF = 1,      
      TM_LED_ON = 1,
      TM_LED_OFF = 0       
    };
    
    enum {
      TM_STATE_INIT,
      TM_STATE_NORM,
      TM_STATE_TO_COLD,
      TM_STATE_HEATING,
      TM_STATE_TO_HOT,
      TM_STATE_COOLING,
      TM_STATE_ERROR
    };

    // Heating temperature
    float hTemp;
    // Cooling temperature
    float cTemp;
    // Temperature hysteresis
    float hTemp;
    uint8_t state;
    Ds18TempSensor innerSensor;
    Ds18TempSensor outerSensor;
     
  public:
    Termostat(float heatingTemp = HEATING_TEMP,
              float collingTemp = COOLING_TEMP,
              float hystTemp = HYST_TEMP): 
        hTemp(heatingTemp),
        cTemp(collingTemp),
        hTemp(hystTemp),
        state(TM_STATE_INIT),
        innerSensor(INNER_TEMP_PIN),
        outerSensor(OUTER_TEMP_PIN)  
    {
    }
    
    status_t init() {
      status_t status = STATUS_SUCCESS;
      
      status = innerSensor.init();
      if (status != STATUS_SUCCESS) {
        LOG64_SET("Could not initialize inner temperature sensor.");
        LOG64_NEW_LINE;
        state = TM_STATE_ERROR;
        return status;
      }
      
      status = outerSensor.init();
      if (status != STATUS_SUCCESS) {
        LOG64_SET("Could not initialize outer temperature sensor.");
        LOG64_NEW_LINE;
        state = TM_STATE_ERROR;
        return status;
      }

      digitalWrite(COOLER_PIN, TM_COOLER_OFF); 
      digitalWrite(HEATER_PIN, TM_HEATER_OFF);

      state = TM_STATE_NORM;
          
      return STATUS_SUCCESS;
    }
    
    void loop () {
      // TODO: Read temperature from sensors here
      
      switch (state) {
        case TM_STATE_NORM:
          //LOG64_SET("TM_STATE_NORM"); LOG64_NEW_LINE;
          //state = DS_STATE_START_CONVERSION;

          // Cooler and heater have to be disabled when 
          // temperature is normal.
          if (digitalRead(COOLER_PIN) != TM_COOLER_OFF) {
            digitalWrite(COOLER_PIN, TM_COOLER_OFF); 
          }

          if (digitalRead(HEATER_PIN) != TM_HEATER_OFF) {
            digitalWrite(HEATER_PIN, TM_HEATER_OFF); 
          }

          // TODO: Add conditions to move to proper state here
          break;
        case TM_STATE_TO_COLD:
          //LOG64_SET("TM_STATE_TO_COLD"); LOG64_NEW_LINE;
          state = TM_STATE_HEATING;
          break;
        case TM_STATE_HEATING:
          //LOG64_SET("TM_STATE_HEATING"); LOG64_NEW_LINE;
          break;
        case TM_STATE_TO_HOT: {
          //LOG64_SET("TM_STATE_TO_HOT"); LOG64_NEW_LINE;
          break;
        }
        case TM_STATE_COOLING:
          //LOG64_SET("TM_STATE_COOLING"); LOG64_NEW_LINE;
          break;
        case TM_STATE_ERROR:
          //LOG64_SET("TM_STATE_ERROR"); LOG64_NEW_LINE;
          break;
        default:
          break;
      }
    }
    
    void dump() {
      if (state == DS_STATE_START) {
        Serial.println("Device is not initialized!");
        return;
      }
      
      Serial.print("ROM =");
      for(uint8_t i = 0; i < DS_ADDR_SIZE; i++) {
        Serial.write(' ');
        Serial.print(addr[i], HEX);
      }

      switch (addr[0]) {
        case DS18S20:
          Serial.println("  Chip = DS18S20");
          break;
        case DS18B20:
          Serial.println("  Chip = DS18B20");
          break;
        case DS1822:
          Serial.println("  Chip = DS1822");
          break;
        default:
          Serial.println("Device is not a DS18x20 family device.");
          return;        
      }
   
      Serial.println("");
      Serial.println("");
    } 
};



Ds18TempSensor innerTempSensor(INNER_TEMP_PIN);
Ds18TempSensor outerTempSensor(OUTER_TEMP_PIN);

void setup() { 
  pinMode(LED_PIN, OUTPUT);
  pinMode(IN_PIN, INPUT_PULLUP);  
  digitalWrite(LED_PIN, HIGH);

  LOG64_INIT();

  //Serial.begin(9600);

  innerTempSensor.init();
  outerTempSensor.init();
}


void loop() {
  innerTempSensor.loop();
  outerTempSensor.loop();
  
  float temp = innerTempSensor.readTemp();

  boolean in_pin_val = digitalRead(IN_PIN);
  if (temp > 23.5){
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }

  if (!in_pin_val){
    innerTempSensor.dump();
    outerTempSensor.dump();
  }
}
