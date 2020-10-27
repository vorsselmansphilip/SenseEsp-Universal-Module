#include <Arduino.h>

//#define SERIAL_DEBUG_DISABLED

#include "sensors/digital_input.h"
#include "sensors/analog_input.h"

#include "sensesp_app.h"
#include "transforms/frequency.h"
#include "signalk/signalk_output.h"
#include "sensors/ina219.h"
#include "transforms/linear.h"
#include "sensors/onewire_temperature.h"


// SensESP builds upon the ReactESP framework. Every ReactESP application
// defines an "app" object vs defining a "main()" method.
ReactESP app([]() {
#ifndef SERIAL_DEBUG_DISABLED
  Serial.begin(115200);

  // A small delay and one debugI() are required so that
  // the serial output displays everything
  delay(100);
  Debug.setSerialEnabled(true);
  #endif
  delay(100);
  debugI("Serial debug enabled");

  sensesp_app = new SensESPApp();

  // The "SignalK path" identifies the output of the sensor to the SignalK network.
  // If you have multiple sensors connected to your microcontoller (ESP), each one of them
  // will (probably) have its own SignalK path variable. For example, if you have two
  // propulsion engines, and you want the RPM of each of them to go to SignalK, you might
  // have sk_path_portEngine = "propulsion.port.revolutions"
  // and sk_path_portEngine = "propulsion.port.revolutions"
  // In this example, there is only one propulsion engine, and its RPM is the only thing
  // being reported to SignalK.
  String sSensorName = sensesp_app->get_hostname() + ".";
  const char* cSensorName = sSensorName.c_str();
  
  //char cSensorName[sSensorName.length() +1];
  //sSensorName.toCharArray(cSensorName,sSensorName.length()+1);
  String sSkPath = sensesp_app->get_hostname() + "." + "propulsion.port.revolutions";

  const char* sk_path = sSkPath.c_str();



  //const char SensorName = SensESPApp::get_hostname();


  // The "Configuration path" is combined with "/config" to formulate a URL
  // used by the RESTful API for retrieving or setting configuration data.
  // It is ALSO used to specify a path to the SPIFFS file system
  // where configuration data is saved on the microcontroller.  It should
  // ALWAYS start with a forward slash if specified. If left blank,
  // that indicates this sensor or transform does not have any
  // configuration to save.
  // Note that if you want to be able to change the sk_path at runtime,
  // you will need to specify a configuration path.
  // As with the SignalK path, if you have multiple configurable sensors
  // connected to the microcontroller, you will have a configuration path
  // for each of them, such as config_path_portEngine = "/sensors/portEngine/rpm"
  // and config_path_portEngine = "/sensor/portEngine/rpm".
  const char* config_path = "/input/D5/counter";

  // These two are necessary until a method is created to synthesize them. Everything
  // after "/sensors" in each of these ("/engine_rpm/calibrate" and "/engine_rpm/sk")
  // is simply a label to display what you're configuring in the Configuration UI. 
  const char* config_path_calibrate = "/input/D5/counter/calibrate";
  const char* config_path_skpath = "/input/D5/counter/sk";


//////////
// connect a RPM meter. A DigitalInputCounter implements an interrupt
// to count pulses and reports the readings every read_delay ms
// (500 in the example). A Frequency
// transform takes a number of pulses and converts that into
// a frequency. The sample multiplier converts the 97 tooth
// tach output into Hz, SK native units.
const float multiplier = 1.0 / 97.0;
const uint read_delay = 500;


  // Wire it all up by connecting the producer directly to the consumer
  // ESP8266 pins are specified as DX
  // ESP32 pins are specified as just the X in GPIOX
  auto* pSensor = new DigitalInputCounter(D5, INPUT_PULLUP, RISING, read_delay);

  pSensor->connectTo(new Frequency(multiplier, config_path_calibrate))  // connect the output of pSensor to the input of Frequency()
        ->connectTo(new SKOutputNumber(sk_path, config_path_skpath));   // connect the output of Frequency() to a SignalK Output as a number



  // Create an INA219, which represents the physical sensor.
  // 0x40 is the default address. Chips can be modified to use 0x41 (shown here), 0x44, or 0x45.
  // The default volt and amp ranges are 32V and 2A (cal32_2). Here, 32v and 1A is specified with cal32_1.
  auto* pINA219 = new INA219(0x40, cal32_1);


  // Create an INA219value, which is used to read a specific value from the INA219, and send its output
  // to SignalK as a number (float). This one is for the bus voltage.
  sSkPath = sensesp_app->get_hostname() + "." + "i2c.ina219.40.bus.voltage";

  auto* pINA219busVoltage = new INA219value(pINA219, bus_voltage, read_delay, "/i2c/ina219/40/busvoltage");
      
      pINA219busVoltage->connectTo(new SKOutputNumber(sSkPath.c_str()));

  // Do the same for the shunt voltage.
  sSkPath = sensesp_app->get_hostname() + "." + "i2c.ina219.40.shunt.voltage";
  auto* pINA219shuntVoltage = new INA219value(pINA219, shunt_voltage, read_delay, "/i2c/ina219/40/shuntvoltage");
      
      pINA219shuntVoltage->connectTo(new SKOutputNumber(sSkPath.c_str()));

  // Do the same for the current (amperage).
  sSkPath = sensesp_app->get_hostname() + "." + "i2c.ina219.40.current";
  auto* pINA219current = new INA219value(pINA219, current, read_delay, "/i2c/ina219/40/current");
      
      pINA219current->connectTo(new SKOutputNumber(sSkPath.c_str()));   

  // Do the same for the power (watts).
  sSkPath = sensesp_app->get_hostname() + "." + "i2c.ina219.40.power";
  auto* pINA219power = new INA219value(pINA219, power, read_delay, "/i2c/ina219/40/power");
      
      pINA219power->connectTo(new SKOutputNumber(sSkPath.c_str()));  

  // Do the same for the load voltage.
  auto* pINA219loadVoltage = new INA219value(pINA219, load_voltage, read_delay, "/i2c/ina219/40/loadvoltage");
  sSkPath = sensesp_app->get_hostname() + "." + "i2c.ina219.40.load.voltage";    
      pINA219loadVoltage->connectTo(new SKOutputNumber(sSkPath.c_str()));     

  
    // Create an INA219, which represents the physical sensor.
  // 0x40 is the default address. Chips can be modified to use 0x41 (shown here), 0x44, or 0x45.
  // The default volt and amp ranges are 32V and 2A (cal32_2). Here, 32v and 1A is specified with cal32_1.
  auto* pINA219_41 = new INA219(0x41, cal32_1);


  // Create an INA219value, which is used to read a specific value from the INA219, and send its output
  // to SignalK as a number (float). This one is for the bus voltage.
  sSkPath = sensesp_app->get_hostname() + "." + "i2c.ina219.41.bus.voltage";

  auto* pINA219busVoltage_41 = new INA219value(pINA219_41, bus_voltage, read_delay, "/i2c/ina219/41/busvoltage");
      
      pINA219busVoltage_41->connectTo(new SKOutputNumber(sSkPath.c_str()));

  // Do the same for the shunt voltage.
  sSkPath = sensesp_app->get_hostname() + "." + "i2c.ina219.41.shunt.voltage";
  auto* pINA219shuntVoltage_41 = new INA219value(pINA219_41, shunt_voltage, read_delay, "/i2c/ina219/41/shuntvoltage");
      
      pINA219shuntVoltage_41->connectTo(new SKOutputNumber(sSkPath.c_str()));

  // Do the same for the current (amperage).
  sSkPath = sensesp_app->get_hostname() + "." + "i2c.ina219.41.current";
  auto* pINA219current_41 = new INA219value(pINA219, current, read_delay, "/i2c/ina219/41/current");
      
      pINA219current_41->connectTo(new SKOutputNumber(sSkPath.c_str()));   

  // Do the same for the power (watts).
  sSkPath = sensesp_app->get_hostname() + "." + "i2c.ina219.41.power";
  auto* pINA219power_41 = new INA219value(pINA219_41, power, read_delay, "/i2c/ina219/41/power");
      
      pINA219power_41->connectTo(new SKOutputNumber(sSkPath.c_str()));  

  // Do the same for the load voltage.
  auto* pINA219loadVoltage_41 = new INA219value(pINA219_41, load_voltage, read_delay, "/i2c/ina219/41/loadvoltage");
  sSkPath = sensesp_app->get_hostname() + "." + "i2c.ina219.41.load.voltage";    
      pINA219loadVoltage_41->connectTo(new SKOutputNumber(sSkPath.c_str()));     



  /* Find all the sensors and their unique addresses. Then, each new instance
     of OneWireTemperature will use one of those addresses. You can't specify
     which address will initially be assigned to a particular sensor, so if you
     have more than one sensor, you may have to swap the addresses around on
     the configuration page for the device. (You get to the configuration page
     by entering the IP address of the device into a browser.)
     ESP8266 pins are specified as DX
     ESP32 pins are specified as just the X in GPIOX
  */
  DallasTemperatureSensors* dts = new DallasTemperatureSensors(D7);
  
  // Start the SensESP application running. Because of everything that's been set up above,
  // it constantly monitors the interrupt pin, and every read_delay ms, it sends the 
  // calculated frequency to SignalK.
    // A Linear transform takes its input, multiplies it by the multiplier, then adds the offset,
  // to calculate its output. The MAX31856TC produces temperatures in degrees Celcius. We need to change
  // them to Kelvin for compatibility with SignalK.
 
  const float multiplierC = 1.0;
  const float offset = -273.16;
  sSkPath = sensesp_app->get_hostname() + "." + "1wire.ds18b.sender1.temperature.kelvin";
  String sSkPath2 = sensesp_app->get_hostname() + "." + "1wire.ds18b.sender1.temperature.celsius";

    auto* sensor_temp =
      new OneWireTemperature(dts, read_delay, "/1wire/ds18b/sender1");
      sensor_temp->connectTo(new SKOutputNumber(sSkPath.c_str()))
                ->connectTo(new Linear(multiplierC,offset,""))
                ->connectTo(new SKOutputNumber(sSkPath2.c_str()));   

 

    // The "SignalK path" identifies this sensor to the SignalK server. Leaving
    // this blank would indicate this particular sensor (or transform) does not
    // broadcast SignalK data.
    const char* sk_path_analog = "sensor.analog.a0";
    sSkPath = sensesp_app->get_hostname() + "." + "analog.a0";

    // The "Configuration path" is combined with "/config" to formulate a URL
    // used by the RESTful API for retrieving or setting configuration data.
    // It is ALSO used to specify a path to the SPIFFS file system
    // where configuration data is saved on the MCU board. It should
    // ALWAYS start with a forward slash if specified. If left blank,
    // that indicates this sensor or transform does not have any
    // configuration to save, or that you're not interested in doing
    // run-time configuration.
    const char* analog_in_config_path = "/analog/A0/read_delay";
    const char* linear_config_path = "/analog/A0/linear";

    // Create a sensor that is the source of our data, that will be read every 500 ms. 
    // It's a light sensor that's connected to the ESP's AnalogIn pin. 
    // The AnalogIn pin on ESP8266 is always A0, but ESP32 has many pins that can be
    // used for AnalogIn, and they're expressed here as the XX in GPIOXX.
    // When it's xx, the sensor's output (as read by analogRead()) is xx, and when
    // it's xx, the output is xx, for a range of xx.
    uint8_t pin = A0;
    

    auto* pAnalogInput = new AnalogInput(pin, read_delay, analog_in_config_path);

    // A Linear transform takes its input, multiplies it by the multiplier, then adds the offset,
    // to calculate its output. In this example, we want to see the final output presented
    // as a percentage, where dark = 0% and bright = 100%. To get a percentage, we use this formula:
    // sensor output * (100 / 730) - 16.44 = output (a percentage from 0 to 100).
    // Dark = 120 * (100 / 730) + (-16.44) = 0%
    // Bright = 850 * (100 / 730) + (-16.44) = 100%
    const float multiplier_analog =1.0; // 100% divided by 730 = 0.137 "percent per analogRead() unit"
    const float offset_analog = 0-0.0;

    // Wire up the output of the analog input to the Linear transform,
    // and then output the results to the SignalK server.
    pAnalogInput -> connectTo(new Linear(multiplier_analog, offset_analog, linear_config_path))
                 -> connectTo(new SKOutputNumber(sSkPath.c_str()));
    
    sSkPath = sensesp_app->get_hostname() + "." + "input.D0";

    auto *pSensorInput = new DigitalInputValue(D0,INPUT_PULLUP,RISING,read_delay,"/input/D0");
    pSensorInput->connectTo(new SKOutputInt(sSkPath.c_str(),""));
    


   




  sensesp_app->enable();
});

