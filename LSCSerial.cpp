#include "LSCSerial.h"

LSCSerial::LSCSerial(HardwareSerial &serialPort) : serial(serialPort) {}

void LSCSerial::begin(uint32_t baudRate) {
    serial.begin(baudRate);
}

void LSCSerial::moveServo(uint8_t id, int16_t position, uint16_t time) {
    uint8_t params[6];
    params[0] = 1; // Количество сервоприводов
    params[1] = time & 0xFF; // Младший байт времени
    params[2] = (time >> 8) & 0xFF; // Старший байт времени
    params[3] = id; // ID сервопривода
    params[4] = position & 0xFF; // Младший байт позиции
    params[5] = (position >> 8) & 0xFF; // Старший байт позиции
    sendPacket(8, 0x03, params, 6);
}

void LSCSerial::moveMultipleServos(uint8_t numServos, uint8_t *ids, int16_t *positions, uint16_t time) {
    uint8_t params[3 + numServos * 3];
    params[0] = numServos;
    params[1] = time & 0xFF;
    params[2] = (time >> 8) & 0xFF;
    for (uint8_t i = 0; i < numServos; i++) {
        params[3 + i * 3] = ids[i];
        params[4 + i * 3] = positions[i] & 0xFF;
        params[5 + i * 3] = (positions[i] >> 8) & 0xFF;
    }
    sendPacket(5 + numServos * 3, 0x03, params, 3 + numServos * 3);
}

uint16_t LSCSerial::getBatteryVoltage() {
    while (serial.available()) serial.read(); // Очистка буфера перед запросом

    sendPacket(2, 0x0F, nullptr, 0); // Запрос напряжения
    
    unsigned long startTime = millis();
    while (serial.available() < 4) { // Ждём хотя бы 4 байта (0x55 0x55, длина, команда)
        if (millis() - startTime > 100) {
            return 0; // Ошибка - нет ответа
        }
    }

    // Ожидаем заголовок 0x55 0x55
    while (serial.read() != 0x55);
    if (serial.read() != 0x55) return 0;

    // Читаем длину (не используем)
    serial.read();

    // Читаем команду (должна быть 0x0F)
    if (serial.read() != 0x0F) return 0;

    // Ожидаем данные (2 байта)
    while (serial.available() < 2);
    
    uint8_t low = serial.read();
    uint8_t high = serial.read();

    return (high << 8) | low;
}

uint16_t LSCSerial::readServoPosition(uint8_t id) {
    while (serial.available()) serial.read(); // Очистка буфера перед запросом
    uint8_t params[2] = {1, id};
    sendPacket(4, 0x15, params, 2); // 0x15 - команда запроса позиции одного сервопривода

    unsigned long startTime = millis();
    while (serial.available() < 8) { // Ожидаем 8 байтов ответа
        if (millis() - startTime > 100) {
            Serial.println("Ошибка: таймаут ожидания ответа");
            return 0;
        }
    }
    
    uint8_t response[8]; // Исправлено: теперь читаем 8 байтов
    for (uint8_t i = 0; i < 8; i++) {
        response[i] = serial.read();
    }
    
    // Выводим полученные данные в HEX для отладки
//    Serial.print("Raw response: ");
//    for (uint8_t i = 0; i < 8; i++) {
//        Serial.print("0x");
//        Serial.print(response[i], HEX);
//        Serial.print(" ");
//    }
//    Serial.println();

    // Проверяем заголовок пакета
    if (response[0] != 0x55 || response[1] != 0x55) {
        Serial.println("Ошибка: неверный заголовок пакета");
        return 0;
    }

    // Проверяем длину пакета (должно быть 0x06)
    if (response[2] != 0x06) {
        Serial.println("Ошибка: неверная длина пакета");
        return 0;
    }

    // Проверяем команду (должна быть 0x15)
    if (response[3] != 0x15) {
        Serial.println("Ошибка: неверный код команды");
        return 0;
    }

    // Проверяем количество сервоприводов (должно быть 0x01)
    if (response[4] != 0x01) {
        Serial.println("Ошибка: количество сервоприводов не 1");
        return 0;
    }

    // Проверяем ID сервопривода
    if (response[5] != id) {
        Serial.println("Ошибка: ID сервопривода не совпадает");
        return 0;
    }

    // Извлекаем позицию сервопривода (последние два байта)
    uint16_t position = (response[7] << 8) | response[6];

    return position;
}






void LSCSerial::unloadServos(uint8_t numServos, uint8_t *ids) {
    uint8_t params[numServos + 1];
    params[0] = numServos;
    for (uint8_t i = 0; i < numServos; i++) {
        params[i + 1] = ids[i];
    }
    sendPacket(3 + numServos, 0x14, params, numServos + 1);
}

void LSCSerial::sendPacket(uint8_t length, uint8_t command, uint8_t *params, uint8_t paramSize) {
    serial.write(0x55);
    serial.write(0x55);
    serial.write(length);
    serial.write(command);
    for (uint8_t i = 0; i < paramSize; i++) {
        serial.write(params[i]);
    }
}
