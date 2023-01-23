#include <Arduino.h>
#include "I2Cduino.h"


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// MASTER DEVICE CODE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------


// MASTER CLASS
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

I2CduinoSlaveMega* globalSlave = NULL;

//Initialise the Wire library as the host
void initI2Cduino(int addr, I2CduinoSlaveMega* slaveStruct)
{
    //init as master
    if (addr == -1)
    {
        Wire.begin();
        return;
    }

    // init as slave with address
    Wire.begin(addr);
    globalSlave = slaveStruct;
    globalSlave->address = addr;
    Wire.onRequest(I2CrequestedFromHost);   
    return;    
}

//Resets the struct to a base state, all pins off, set to output with the internal pullup resistors disabled
void zeroI2CSlaveStruct(I2CduinoSlaveMega &slave, uint8_t  address)
{
    slave.address = address;
    ZeroMemory(slave.digitalPins.bytes, sizeof(int64_t));
    ZeroMemory(slave.digitalPinsInputFlag.bytes, sizeof(int64_t));
    ZeroMemory(slave.digitalPinsPullupEnable.bytes, sizeof(int64_t));
}

//Intended to be called within loop(), this will send your slaveMega struct to the specified slave which will then apply
//all the pin states for output pins, if pins are set to input then they are skipped.
//Then we send a flag to the target to set what data we want to request
//Then request the data and read the response, then save the digital pins state to our struct so they can be manipulated
void syncI2CSlave(I2CduinoSlaveMega &device)
{
    transmitData(DIGITAL_PINS_SYNC, device.digitalPins, device.address);
    sendRequestFlag(device.address, REQUEST_FLAG_DIGITAL_PINS);
    delay(25);
    uint8_t temp = Wire.requestFrom(device.address, STANDARD_ANSWERSIZE);
    delay(25);
    if (Wire.available() >= 1) {
        // Read the first byte as the flag
        uint8_t flag = Wire.read();

        // Check the flag and handle the data accordingly
        switch (flag) {
            case REQUEST_FLAG_DIGITAL_PINS:
                if (Wire.available() >= 8) {
                    // Read the 8 bytes as the digitalPins variable
                    for (int i = 0; i < 8; i++) {
                        device.digitalPins.bytes[i] = Wire.read();
                    }
                }
                break;
            // Add more cases for other flags here
            default:
                break;
        }
    }
}

//General purpose function to send the contents of a 64 bit int to the I2C device address with a leading byte to flag the type of data for the slave to handle it as
void transmitData(uint8_t flag, int64Union& data, uint8_t address) 
{
    // Start transmission to the specified address
    Wire.beginTransmission(address);

    // Send the flag byte first
    Wire.write(flag);

    // Send the bytes of the int64Union variable
    for (int i = 0; i < 8; i++) {
        Wire.write(data.bytes[i]);
    }

    // End the transmission
    Wire.endTransmission();
}

//Function to send a request flag byte (NOTE, IF YOU SEND A NON-REQUEST FLAG BYTE YOU WILL FUCK UP THE SLAVE)
void sendRequestFlag(uint8_t deviceAddress, uint8_t flag)
{
    Wire.beginTransmission(deviceAddress);
    Wire.write(flag);
    Wire.endTransmission();
}

//DigitalWrite() but for slave devices, this just sets the appropriate byte for the pin in the slave's struct, this will be synced on the next I2C sync call
void slaveDigitalWrite(I2CduinoSlaveMega &device, int pin, int value) {
    if (value == HIGH) {
        SET_BIT_SLVLIB(device.digitalPins.value, pin);
    } else {
        CLEAR_BIT_SLVLIB(device.digitalPins.value, pin);
    }
}

//DigitalRead() but for slave devices, this returns the state of the pin requested from the struct.
int slaveDigitalRead(I2CduinoSlaveMega &device, int pin) {
    return GET_BIT_SLVLIB(device.digitalPins.value, pin);
}

void slavePinMode(I2CduinoSlaveMega &device, int pin, int mode) {
    // Check the value of mode
    switch (mode) {
        // If mode is INPUT
        case INPUT:
            // Set the bit of the pin in digitalPinsInputFlag to 1
            SET_BIT_SLVLIB(device.digitalPinsInputFlag.value, pin);
            // Clear the bit of the pin in digitalPinsPullupEnable to 0
            CLEAR_BIT_SLVLIB(device.digitalPinsPullupEnable.value, pin);
            break;
        // If mode is OUTPUT
        case OUTPUT:
            // Clear the bit of the pin in digitalPinsInputFlag to 0
            CLEAR_BIT_SLVLIB(device.digitalPinsInputFlag.value, pin);
            // Clear the bit of the pin in digitalPinsPullupEnable to 0
            CLEAR_BIT_SLVLIB(device.digitalPinsPullupEnable.value, pin);
            break;
        // If mode is INPUT_PULLUP
        case INPUT_PULLUP:
            // Set the bit of the pin in digitalPinsInputFlag to 1
            SET_BIT_SLVLIB(device.digitalPinsInputFlag.value, pin);
            // Set the bit of the pin in digitalPinsPullupEnable to 1
            SET_BIT_SLVLIB(device.digitalPinsPullupEnable.value, pin);
            break;
        // Default case
        default:
            break;
    }
    // Send the data of digitalPinsInputFlag to the slave device
    transmitData(DIGITAL_PINS_INPUT_FLAG_SYNC, device.digitalPinsInputFlag, device.address);
    // Send the data of digitalPinsPullupEnable to the slave device
    transmitData(DIGITAL_PINS_PULLUP_ENABLE_SYNC, device.digitalPinsPullupEnable, device.address);
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// SLAVE DEVICE CODE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// SLAVE CLASS
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*
void handleI2CData()
{
    // Check if data is available
    if (Wire.available() >= 1) {
        // Read the first byte as the flag
        uint8_t flag = Wire.read();
        Serial.println("Recieved I2C data");
        // Check the flag and handle the data accordingly
        switch (flag) 
        {
            case DIGITAL_PINS_SYNC:
                // Read the next 8 bytes as the digitalPins variable
                for (int i = 0; i < 8; i++) {
                    globalSlave->digitalPins.bytes[i] = Wire.read();
                }
                for(int i=0; i<54; i++){
                    if(!GET_BIT_SLVLIB(globalSlave->digitalPinsInputFlag.value, i)){
                        // set the pin as output
                        pinMode(i, OUTPUT);
                        // set the pin to the value stored in digitalPins
                        digitalWrite(i, GET_BIT_SLVLIB(globalSlave->digitalPins.value, i));
                    }else{
                        // set the pin as input
                        pinMode(i, INPUT);
                        // read the pin state
                        int pinValue = digitalRead(i);
                        // store the pin state in the digitalPins variable
                        if(pinValue)
                            SET_BIT_SLVLIB(globalSlave->digitalPins.value, i);
                        else
                            CLEAR_BIT_SLVLIB(globalSlave->digitalPins.value, i);
                    }
                }
                break;            

            case DIGITAL_PINS_INPUT_FLAG_SYNC:
                // Read the next 8 bytes as the digitalPinsInputFlag variable
                for (int i = 0; i < 8; i++) {
                    globalSlave->digitalPinsInputFlag.bytes[i] = Wire.read();
                }
                for(int i=0; i<54; i++){
                    if(GET_BIT_SLVLIB(globalSlave->digitalPinsInputFlag.value, i)){
                        // set the pin as input
                        pinMode(i, INPUT);
                    }else{
                        // set the pin as output
                        pinMode(i, OUTPUT);
                    }
                }
                break;

                case DIGITAL_PINS_PULLUP_ENABLE_SYNC:
                // Read the next 8 bytes as the digitalPinsPullupEnable variable
                for (int i = 0; i < 8; i++) {
                    globalSlave->digitalPinsPullupEnable.bytes[i] = Wire.read();
                }
                for(int i=0; i<54; i++){
                    if(GET_BIT_SLVLIB(globalSlave->digitalPinsPullupEnable.value, i)){
                        // set the pin pullup resistor
                        digitalWrite(i,HIGH);
                        pinMode(i, INPUT_PULLUP);
                    }
                }
                break;

            case REQUEST_FLAG_DIGITAL_PINS:
                globalSlave->requestFlag = REQUEST_FLAG_DIGITAL_PINS;
                break;
        }
    }
}
*/

#define I2CDUINO_DEBUG


void handleI2CData(int howMany)
{
    Serial.println("I2C ping");
    // Check if data is available
    if (Wire.available() >= 1) {
        // Read the first byte as the flag
        uint8_t flag = Wire.read();
#ifdef I2CDUINO_DEBUG
        Serial.println("Recieved I2C data");
#endif
        // Check the flag and handle the data accordingly
        switch (flag) 
        {
            case DIGITAL_PINS_SYNC:
                // Read the next 8 bytes as the digitalPins variable
                for (int i = 0; i < 8; i++) {
                    globalSlave->digitalPins.bytes[i] = Wire.read();
                }
                for(int i=0; i<54; i++){
                    if(!GET_BIT_SLVLIB(globalSlave->digitalPinsInputFlag.value, i)){
                        // set the pin as output
                        pinMode(i, OUTPUT);
                        // set the pin to the value stored in digitalPins
                        digitalWrite(i, GET_BIT_SLVLIB(globalSlave->digitalPins.value, i));
#ifdef I2CDUINO_DEBUG
                        Serial.print("Digital Pin:");
                        Serial.print(i);
                        Serial.print(" set as Output with value:");
                        Serial.println((int) GET_BIT_SLVLIB(globalSlave->digitalPins.value, i));
#endif
                    }else{
                        // set the pin as input
                        pinMode(i, INPUT);
                        // read the pin state
                        int pinValue = digitalRead(i);
                        // store the pin state in the digitalPins variable
                        if(pinValue)
                            SET_BIT_SLVLIB(globalSlave->digitalPins.value, i);
                        else
                            CLEAR_BIT_SLVLIB(globalSlave->digitalPins.value, i);
#ifdef I2CDUINO_DEBUG
                        Serial.print("Digital Pin:");
                        Serial.print(i);
                        Serial.print(" set as Input with value:");
                        Serial.println(pinValue);
#endif
                    }
                }
                break;            

            case DIGITAL_PINS_INPUT_FLAG_SYNC:
                                // Read the next 8 bytes as the digitalPinsInputFlag variable
                for (int i = 0; i < 8; i++) {
                    globalSlave->digitalPinsInputFlag.bytes[i] = Wire.read();
                }
                for(int i=0; i<54; i++){
                    if(GET_BIT_SLVLIB(globalSlave->digitalPinsInputFlag.value, i)){
                        // set the pin as input
                        pinMode(i, INPUT);
#ifdef I2CDUINO_DEBUG
                        Serial.print("Digital Pin:");
                        Serial.print(i);
                        Serial.println(" set as Input");
#endif
                    }else{
                        // set the pin as output
                        pinMode(i, OUTPUT);
#ifdef I2CDUINO_DEBUG
                        Serial.print("Digital Pin:");
                        Serial.print(i);
                        Serial.println(" set as Output");
#endif
                    }
                }
                break;

                case DIGITAL_PINS_PULLUP_ENABLE_SYNC:
                // Read the next 8 bytes as the digitalPinsPullupEnable variable
                for (int i = 0; i < 8; i++) {
                    globalSlave->digitalPinsPullupEnable.bytes[i] = Wire.read();
                }
                for(int i=0; i<54; i++){
                    if(GET_BIT_SLVLIB(globalSlave->digitalPinsPullupEnable.value, i)){
                        // set the pin pullup resistor
                        digitalWrite(i,HIGH);
                        pinMode(i, INPUT_PULLUP);
#ifdef I2CDUINO_DEBUG
                        Serial.print("Digital Pin:");
                        Serial.print(i);
                        Serial.println(" Pullup resistor enabled");
#endif
                    }
                }
                break;

            case REQUEST_FLAG_DIGITAL_PINS:
                globalSlave->requestFlag = REQUEST_FLAG_DIGITAL_PINS;
#ifdef I2CDUINO_DEBUG
                Serial.println("Digital pins data requested");
#endif
                break;
        }
    }
}


/*
// function that executes whenever data is received from master
void handleI2CData(int howMany) {
    Serial.println("Data from Master");
  while (Wire.available()) {
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);      // print the character
  }
  Serial.println();
}
*/


void I2CrequestedFromHost()
{
    byte response[STANDARD_ANSWERSIZE];
    switch (globalSlave->requestFlag )
    {
    case REQUEST_FLAG_DIGITAL_PINS:
        response[0] = REQUEST_FLAG_DIGITAL_PINS;
        for (int i = 1; i < STANDARD_ANSWERSIZE; i++)
            response[i] = globalSlave->digitalPins.bytes[(i-1)];
        
        Wire.write(response,STANDARD_ANSWERSIZE);
        break;
    
    default:
        break;
    }
}

