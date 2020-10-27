#include "ina219_sender.h"

#include "sensesp.h"
//#include "i2c_tools.h"
#include <RemoteDebug.h>


// INA219Sender represents an ADAfruit (or compatible) INA219 High Side DC Current Sensor.
INA219Sender::INA219Sender(uint8_t addr, INA219SenderCAL_t calibration_setting, String config_path) :
       Sensor(config_path) {
    className = "INA219Sender";
    load_configuration();
    pAdafruitINA219Sender = new Adafruit_INA219(addr);
    pAdafruitINA219Sender->begin();
    // Default calibration in the Adafruit_INA219Sender constructor is 32V and 2A, so that's what it will be unless
    // it's set to something different in the call to this constructor:
    
    if (calibration_setting == cal32_1) {
         pAdafruitINA219Sender->setCalibration_32V_1A();
    }     
    else if (calibration_setting == cal16_400) {
         pAdafruitINA219Sender->setCalibration_16V_400mA();
    }     
}



// INA219Sendervalue reads and outputs the specified type of value of a INA219Sender sensor
INA219Sendervalue::INA219Sendervalue(INA219Sender* pINA219Sender, INA219SenderValType val_type, uint read_delay,float R_Shunt, float R1, String config_path) :
                   NumericSensor(config_path), pINA219Sender{pINA219Sender}, val_type{val_type}, read_delay{read_delay} {
      className = "INA219Sendervalue";
      load_configuration();
}


void INA219Sendervalue::enable() {
  app.onRepeat(read_delay, [this](){
      switch (val_type) { 
        case bus_voltage: output = pINA219Sender->pAdafruitINA219Sender->getBusVoltage_V();
                break;
        case shunt_voltage: output = (pINA219Sender->pAdafruitINA219Sender->getShuntVoltage_mV() / 1000); // SK wants volts, not mV
                break;
        case current: output = (pINA219Sender->pAdafruitINA219Sender->getCurrent_mA() / 1000); // SK wants amps, not mA
                break;
        case power: output = (pINA219Sender->pAdafruitINA219Sender->getPower_mW() / 1000); // SK want watts, not mW
                break; 
        case load_voltage: output = (pINA219Sender->pAdafruitINA219Sender->getBusVoltage_V() + (pINA219Sender->pAdafruitINA219Sender->getShuntVoltage_mV() / 1000));
                 break; 
         case sender_resistance: 
                  Vin = (pINA219Sender->pAdafruitINA219Sender->getBusVoltage_V());
                  //Vin = (pINA219Sender->pAdafruitINA219Sender->getBusVoltage_V() + (pINA219Sender->pAdafruitINA219Sender->getShuntVoltage_mV() / 1000));

                  Iin = (pINA219Sender->pAdafruitINA219Sender->getCurrent_mA()/1000);
                  VShunt = (pINA219Sender->pAdafruitINA219Sender->getShuntVoltage_mV()/1000);
                  char cVin[10];
                  char cIin[10];
                  char cR2[10];
                  //R_Shunt = 0.1;
                  //R1 = 10;
                  

                  dtostrf(Vin,6,6,cVin);
                  dtostrf(Iin*1000,6,6,cIin);

                  result =  ((Vin-(R1* Iin))/Iin);
                  //result = (Vin-VShunt)/Iin;
                 

                  if (Iin > 0.0){
                    output =  result;

                  } else {
                    output =  0.0;
                  }
                  

                  dtostrf(output,6,6,cR2);
                  debugI("Voltage: %s\nCurrent: %s\n Resistance R2: %s  ",cVin, cIin, cR2 );
   



                 break; 
        default: debugE("FATAL: invalid val_type parameter.");  
      }
      
      notify();
 });
}

JsonObject& INA219Sendervalue::get_configuration(JsonBuffer& buf) {
  JsonObject& root = buf.createObject();
  root["read_delay"] = read_delay;
  root["R_shunt"] = R_Shunt;
  root["R1"] = R1;
  root["value"] = output;
  return root;
  };

  static const char SCHEMA[] PROGMEM = R"###({
    "type": "object",
    "properties": {
        "read_delay": { "title": "Read delay", "type": "number", "description": "The time, in milliseconds, between each read of the input" },
        "R_shunt": { "title": "R shunt", "type": "number", "description": "value in ohm of the shuntresistor" },
        "R1": { "title": "R1", "type": "number", "description": "value in ohm of R1" },
        "value": { "title": "Last value", "type" : "number", "readOnly": true }
    }
  })###";


  String INA219Sendervalue::get_config_schema() {
  return FPSTR(SCHEMA);
}

bool INA219Sendervalue::set_configuration(const JsonObject& config) {
  String expected[] = {"read_delay"};
  for (auto str : expected) {
    if (!config.containsKey(str)) {
      return false;
    }
  }
  read_delay = config["read_delay"];
  R_Shunt = config["R_shunt"];
  R1 = config["R1"];
  
  return true;
}
