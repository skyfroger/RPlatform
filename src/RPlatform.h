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

enum direction
{
  FW,
  BW
}; // FW - вперёд; BW - назад.

class RPlatform
{
public:
  RPlatform(); // конструктор
  String getLog();
  uint16_t readSensor(uint8_t portNumber, bool raw = false); // чтение показаний с датчика на позиции sensor
  bool isStartPressed();                                     // нажата ли кнопка старт
  void stop(bool fullStop = true);                           // остановка двигателей
  void setDirection(direction dir);
  void setDirection(direction dirL, direction dirR);
  void setPower(uint8_t power);                                                        // одинаковая скорость на оба мотора
  void setPower(uint8_t powerL, uint8_t powerR);                                       // разная скорость для моторов
  void setRunSettings(direction dirL, direction dirR, uint8_t powerL, uint8_t powerR); // настройка всех параметров двигателей
  void run();                                                                          // запуск двигателей
  void runTime(float seconds);                                                         // запуск двигателей на N секунд
  void runSteps(uint8_t steps);                                                        // запуск двигателей на N шагов
  void runAngle(uint8_t angle);                                                        // запуск двигателей на N градусов
  void resetStepCounters();                                                            // обнуление счётчиков шагов
  void getLeftSteps();                                                                 // количество шагов левого двигателя
  void getRightSteps();                                                                // количество шагов правого двигателя
  void turnLeft(uint8_t angle);                                                        // повернуть налево на заданный угол
  void turnRight(uint8_t angle);                                                       // повернуть направо на заданный угол

private:
  void stopLeftMotor(bool fullStop = true);
  void stopRightMotor(bool fullStop = true);
  void setLeftMotorDirection(direction dir);          // направление левого двигателя
  void setRightMotorDirection(direction dir);         // направление правого двигателя
  void setRightMotorPower(uint8_t power);             // скорость правого мотора
  void setLeftMotorPower(uint8_t power);              // скорость левого мотора
  void startLeftMotor();                              // запуск левого двигателя с заданным направлением и мощностью
  void startRightMotor();                             // запуск правого двигателя с заданным направлением и мощностью
  uint8_t _leftMotorPower;                            // мощность левого двигателя
  uint8_t _rightMotorPower;                           // мощность правого двигателя
  direction _leftDir;                                 // направление левого двигателя
  direction _rightDir;                                // направление правого двигателя
  bool _isLeftMotorOn;                                // включен ли левый мотор?
  bool _isRightMotorOn;                               // включен ли правый мотор?
  uint8_t _sensorPorts[6] = {A0, A1, A2, A3, A4, A5}; // список портов с датчиками
};

#endif
