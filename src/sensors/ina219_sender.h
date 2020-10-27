#ifndef _ina219_sender_H_
#define _ina219_sender_H_

#include <Wire.h>
#include <Adafruit_INA219.h>

#include "sensor.h"

// The INA219 classes are based on the ADAfruit_INA219 library.

// These are the used to tell the constructor what to set for maximum voltage and amperage.
// The default in the Adafruit constructor is 32V and 2A, so we only need to handle the other two.
enum INA219SenderCAL_t { cal32_2, cal32_1, cal16_400 };

// INA219 represents an ADAfruit (or compatible) INA219 High Side DC Current Sensor.
// The constructor creates a pointer to the instance, and starts up the sensor. The pointer is
// passed to INA219Sendervalue, which retrieves the specified value.
// 
class INA219Sender : public Sensor {
  public:
    INA219Sender(uint8_t addr = 0x40, INA219SenderCAL_t calibration_setting = cal16_400, String config_path = "");
    Adafruit_INA219* pAdafruitINA219Sender;

};


// Pass one of these in the constructor to INA219Sendervalue() to tell which type of value you want to output
enum INA219SenderValType { bus_voltage, shunt_voltage, current, power, load_voltage,sender_resistance };

// INA219Sendervalue reads and outputs the specified value of a INA219 sensor.
class INA219Sendervalue : public NumericSensor {
  public:
    INA219Sendervalue(INA219Sender* pINA219Sender, INA219SenderValType val_type, uint read_delay = 500, float R_shunt=0.1, float R1=220,String config_path="");
    void enable() override final;
    INA219Sender* pINA219Sender;

  private:
    
    INA219SenderValType val_type;
    uint read_delay;
    float R_Shunt;
    float R1;
    float Vin;
    float VShunt;
    float Iin;
    float result;
    virtual JsonObject& get_configuration(JsonBuffer& buf) override;
    virtual bool set_configuration(const JsonObject& config) override;
    virtual String get_config_schema() override;

};

#endif
