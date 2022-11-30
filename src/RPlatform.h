/*
 * Библиотека для управления РОББО Платформой
 */

#ifndef RPlatform_h
#define RPlatform_h

#include "Arduino.h"

#define R_F 5
#define R_B 6
#define L_F 9
#define L_B 10
#define START_BUTTON 14

enum direction {FW, BW}; // FW - вперёд; BW - назад.

class RPlatform {
  public:
    RPlatform(); // конструктор
    String getLog();
    int readSensor(int portNumber, bool raw = false); //+ чтение показаний с датчика на позиции sensor
    bool isStartPressed(); // нажата ли кнопка старт
    void stopMotors(bool fullStop = true); //+ остановка двигателей
    void setDirection(direction dir);
    void setDirection(direction dirL, direction dirR);
    void setPower(int power); // одинаковая скорость на оба мотора
    void setPower(int powerL, int powerR); // разная скорость для моторов
    void setMovementParams(direction dirL, direction dirR, int powerL, int powerR); // настройка всех параметров двигателей
    void startMotors(); // запуск двигателей
  private:
    void stopLeftMotor(bool fullStop = true);
    void stopRightMotor(bool fullStop = true);
    void setLeftMotorDirection(direction dir); // направление левого двигателя
    void setRightMotorDirection(direction dir); // направление правого двигателя
    void setRightMotorPower(int power); // скорость правого мотора
    void setLeftMotorPower(int power); // скорость левого мотора
    void startLeftMotor(); // запуск левого двигателя с заданным направлением и мощностью
    void startRightMotor(); // запуск правого двигателя с заданным направлением и мощностью
    int _leftMotorPower; // мощность левого двигателя
    int _rightMotorPower; // мощность правого двигателя
    direction _leftDir; // направление левого двигателя
    direction _rightDir; // направление правого двигателя
    bool _isLeftMotorOn; // включен ли левый мотор?
    bool _isRightMotorOn; // включен ли правый мотор?
    int _sensorPorts[6] = {A0, A1, A2, A3, A4, A5}; // список портов с датчиками
};

#endif
