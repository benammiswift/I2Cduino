//We intend to wrap the Wire library but check it's not already included
#ifndef TwoWire_h
#include <Wire.h> 
#endif

//Check if the library is already included
#ifndef I2CDUINO_H
#define I2CDUINO_H

#define I2CDUINO_DEBUG

//Probably not needed but it avoids squiggles in VSC
#ifndef byte
    #define byte char
#endif

//Exists in windows and I really like it so I'm defining it
#define ZeroMemory(Destination,Length) memset((Destination),0,(Length))
#define GET_BIT_SLVLIB(value, bit)   ((value >> bit) & 1)
#define SET_BIT_SLVLIB(value, bit)   (value |= (1ll << bit))
#define CLEAR_BIT_SLVLIB(value, bit)   (value &= ~(1ll << bit))
#define TOGGLE_BIT_SLVLIB(value, bit)   (value ^= (1ll << bit))


//these define if the device is a Master or a Slave, here for debugging, intended to be user controlled
///#define I2CDUINO_MASTER_DEVICE
//#define I2CDUINO_SLAVE_DEVICE

#define STANDARD_ANSWERSIZE (uint8_t) 9

enum I2CDUINO_Flags {
    DIGITAL_PINS_SYNC               = 0x01,
    DIGITAL_PINS_INPUT_FLAG_SYNC    = 0x02,
    DIGITAL_PINS_PULLUP_ENABLE_SYNC = 0x03,
    REQUEST_FLAG_DIGITAL_PINS       = 0x04,
    // Add more flags as needed
};

//Define structs to store info of a remote slave Arduino Mega
typedef union {
    int64_t value;
    uint8_t bytes[8];
} int64Union;

typedef struct {
    int64Union digitalPins;
    int64Union digitalPinsInputFlag;
    int64Union digitalPinsPullupEnable;
    uint8_t  address;
    byte requestFlag;
} I2CduinoSlaveMega;

//Common Functions
void initI2Cduino(int addr, I2CduinoSlaveMega* slaveStruct);
void zeroI2CSlaveStruct(I2CduinoSlaveMega &slave, uint8_t  address);

//Master Functions
void syncI2CSlave(I2CduinoSlaveMega &device);
void sendRequestFlag(uint8_t deviceAddress, uint8_t flag);
void slaveDigitalWrite(I2CduinoSlaveMega &device, int pin, int value);
int  slaveDigitalRead(I2CduinoSlaveMega &device, int pin);
void slavePinMode(I2CduinoSlaveMega &device, int pin, int mode);
void transmitData(uint8_t flag, int64Union& data, uint8_t address) ;

//Slave functions
void handleI2CData(int howMany);
void I2CrequestedFromHost();
extern I2CduinoSlaveMega* globalSlave;

#endif