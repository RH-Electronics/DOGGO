// LSCSerial.h
#ifndef LSC_SERIAL_H
#define LSC_SERIAL_H

#include <Arduino.h>

class LSCSerial {
public:
    LSCSerial(HardwareSerial &serialPort);
    void begin(uint32_t baudRate = 9600);
    
    void moveServo(uint8_t id, int16_t position, uint16_t time);
    void moveMultipleServos(uint8_t numServos, uint8_t *ids, int16_t *positions, uint16_t time);
    uint16_t getBatteryVoltage();
    uint16_t readServoPosition(uint8_t id);

    void unloadServos(uint8_t numServos, uint8_t *ids);

private:
    HardwareSerial &serial;
    void sendPacket(uint8_t length, uint8_t command, uint8_t *params, uint8_t paramSize);
};

#endif // LSC_SERIAL_H
