/*
 * Библиотека для управления РОББО Платформой
 */

#ifndef RPlatform_h
#define RPlatform_h

#include "Arduino.h"

#define R_F 5              // правый вперёд
#define R_B 6              // правый назад
#define L_F 9              // левый вперёд
#define L_B 10             // левый назад
#define START_BUTTON 14    // кнопка Start
#define L_ENC 3            // пин левого энкодера
#define R_ENC 2            // пин правого энкодера
#define FULL_TURN_STEPS 24 // количество шагов на полный оборот колеса
#define TURN_COEFF 2.5     // для поворота колёс на указанный угол
#define PLATFORM_TURN 2.6  // для поворота платформы на указанный угол

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
  int readSensor(int portNumber, bool raw = false); // чтение показаний с датчика на позиции sensor
  bool isStartPressed();                            // нажата ли кнопка старт
  void stop(bool fullStop = true);                  // остановка двигателей
  void setDirection(direction dir);
  void setDirection(direction dirL, direction dirR);
  void setPower(int power);                                                    // одинаковая скорость на оба мотора
  void setPower(int powerL, int powerR);                                       // разная скорость для моторов
  void setRunSettings(direction dirL, direction dirR, int powerL, int powerR); // настройка всех параметров двигателей
  void run();                                                                  // запуск двигателей
  void move(int leftPower, int rightPower);
  void move(int leftPower, int rightPower, int time);
  void move(int leftPower, int rightPower, int lAngle, int rAngle);
  void runTime(float seconds);           // запуск двигателей на N секунд
  void runSteps(int steps);              // запуск двигателей на N шагов
  void runSteps(int stepsL, int stepsR); // запуск левого и правого мотора на разное количество шагов
  void runAngle(int angle);              // запуск двигателей на N градусов
  void runAngle(int angleL, int angleR); // поворот левого и правого колеса на заданный угол
  void turnLeft(int angle);              // повернуть налево на заданный угол
  void turnRight(int angle);             // повернуть направо на заданный угол
  int getLeftSteps();                    // количество шагов левого двигателя
  int getRightSteps();                   // количество шагов правого двигателя
  void resetSteps();
  void ledOn(int portNumber);
  void ledOff(int portNumber);

private:
  void stopLeftMotor(bool fullStop = true);
  void stopRightMotor(bool fullStop = true);
  void setLeftMotorDirection(direction dir);  // направление левого двигателя
  void setRightMotorDirection(direction dir); // направление правого двигателя
  void setRightMotorPower(int power);         // скорость правого мотора
  void setLeftMotorPower(int power);          // скорость левого мотора
  void startLeftMotor();                      // запуск левого двигателя с заданным направлением и мощностью
  void startRightMotor();                     // запуск правого двигателя с заданным направлением и мощностью
  static void incLeftSteps();
  static void incRightSteps();
  int _leftMotorPower;  // мощность левого двигателя
  int _rightMotorPower; // мощность правого двигателя
  direction _leftDir;   // направление левого двигателя
  direction _rightDir;  // направление правого двигателя
  bool _isLeftMotorOn;  // включен ли левый мотор?
  bool _isRightMotorOn; // включен ли правый мотор?
  volatile static int _leftStepsCount;
  volatile static int _rightStepsCount;
  int _sensorPorts[6] = {A0, A1, A2, A3, A4, A5};    // аналоговые пины на портах 1-5
  int _digitalOutputPorst[6] = {4, 4, 7, 8, 11, 12}; // цифровые пины на портах 1-5
};

#endif
