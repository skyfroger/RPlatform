#include "Arduino.h"
#include "RPlatform.h"

volatile int RPlatform::_leftStepsCount = 0;
volatile int RPlatform::_rightStepsCount = 0;

RPlatform::RPlatform()
{
  /**
   * Конструктор класса
   */

  //Пины, которые отвечают за двигатели, настраиваем на вход
  pinMode(R_F, INPUT);
  pinMode(R_B, INPUT);
  pinMode(L_F, INPUT);
  pinMode(L_B, INPUT);

  pinMode(L_ENC, INPUT);
  pinMode(R_ENC, INPUT);

  // кнопка Start
  pinMode(START_BUTTON, INPUT);

  // начальное направление Вперёд
  this->_leftDir = FW;
  this->_rightDir = FW;

  // начальная мощность для моторов = 0
  this->_leftMotorPower = 0;
  this->_rightMotorPower = 0;

  // изначально платформа неподвижна
  this->_isLeftMotorOn = false;
  this->_isRightMotorOn = false;

  attachInterrupt(digitalPinToInterrupt(L_ENC), RPlatform::incLeftSteps, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC), RPlatform::incRightSteps, CHANGE);
}

String RPlatform::getLog()
{
  /**
   * Вывод отладочной информации
   * Метод вернёт строку с текущими настройками платформы и данными с портов.
   */
  char buffer[128];
  sprintf(buffer, "LM: p=%d dir=%d; RM: p=%d dir=%d; S1=%d; S2=%d; S3=%d; S4=%d; S5=%d steps: %d %d",
          this->_leftMotorPower, this->_leftDir,
          this->_rightMotorPower, this->_rightDir,
          this->readSensor(1), this->readSensor(2),
          this->readSensor(3), this->readSensor(4),
          this->readSensor(5), RPlatform::_leftStepsCount, RPlatform::_rightStepsCount);
  return buffer;
}

void RPlatform::stopLeftMotor(bool fullStop = true)
{
  /**
   * Остановка левого двигателя
   */
  this->_isLeftMotorOn = false;

  if (fullStop)
  {
    digitalWrite(L_F, HIGH);
    digitalWrite(L_B, HIGH);
  }
  else
  {
    digitalWrite(L_F, LOW);
    digitalWrite(L_B, LOW);
  }
}

void RPlatform::stopRightMotor(bool fullStop = true)
{
  /**
   * Остановка правого двигателя
   */
  this->_isRightMotorOn = false;

  if (fullStop)
  {
    digitalWrite(R_F, HIGH);
    digitalWrite(R_B, HIGH);
  }
  else
  {
    digitalWrite(R_F, LOW);
    digitalWrite(R_B, LOW);
  }
}

void RPlatform::stop(bool fullStop = true)
{
  /*
   * Остановка платформы
   */
  this->stopLeftMotor(fullStop);
  this->stopRightMotor(fullStop);
}

int RPlatform::readSensor(int portNumber, bool raw = false)
{
  /**
   * Возвращаем значение, измеренное датчиком в указанном слоте
   */
  int rawValue = analogRead(this->_sensorPorts[portNumber]);
  if (raw)
    return rawValue;
  return map(rawValue, 0, 1023, 0, 100);
}

bool RPlatform::isStartPressed()
{
  /**
   * Нажата ли кнопка Start
   */
  return !digitalRead(START_BUTTON);
}

void RPlatform::setLeftMotorDirection(direction dir)
{
  /**
   * Направление вращения левого мотора
   */
  this->_leftDir = dir;

  if (this->_isLeftMotorOn)
  {
    this->startLeftMotor();
  }
}

void RPlatform::setRightMotorDirection(direction dir)
{
  /**
   *  Направление вращения правого мотора
   */
  this->_rightDir = dir;

  if (this->_isRightMotorOn)
    this->startRightMotor();
}

void RPlatform::setDirection(direction dir)
{
  /**
   *  Выбор направления движения платформы
   */
  this->setLeftMotorDirection(dir);
  this->setRightMotorDirection(dir);
}

void RPlatform::setDirection(direction dirL, direction dirR)
{
  /**
   *  Выбор направления движения платформы
   */
  this->setLeftMotorDirection(dirL);
  this->setRightMotorDirection(dirR);
}

void RPlatform::setRightMotorPower(int power)
{
  /**
   * Изменяем мощность правого мотора
   */
  int p = constrain(power, 0, 100);
  this->_rightMotorPower = p;

  if (this->_isRightMotorOn)
  {
    this->startRightMotor();
  }
}

void RPlatform::setLeftMotorPower(int power)
{
  /**
   * Изменяем мощность левого мотора
   */
  int p = constrain(power, 0, 100);
  this->_leftMotorPower = p;

  if (this->_isLeftMotorOn)
  {
    this->startLeftMotor();
  }
}

void RPlatform::setPower(int power)
{
  /**
   * Выбор одинаковой мощности для обоих моторов
   */
  this->setLeftMotorPower(power);
  this->setRightMotorPower(power);
}

void RPlatform::setPower(int powerL, int powerR)
{
  /**
   * Выбор мощности для каждого мотора отдельно
   */
  this->setLeftMotorPower(powerL);
  this->setRightMotorPower(powerR);
}

void RPlatform::setRunSettings(direction dirL, direction dirR, int powerL, int powerR)
{
  /**
   * Настройка направления и мощности моторов
   */
  this->setDirection(dirL, dirR);
  this->setPower(powerL, powerR);
}

void RPlatform::startLeftMotor()
{
  /**
   * Включении левого мотора
   */
  this->_isLeftMotorOn = true;

  int pwmValue = map(this->_leftMotorPower, 0, 100, 0, 1023);

  if (this->_leftDir == FW)
  {
    analogWrite(L_F, pwmValue);
    digitalWrite(L_B, LOW);
  }
  else
  {
    analogWrite(L_B, pwmValue);
    digitalWrite(L_F, LOW);
  }
}

void RPlatform::startRightMotor()
{
  /**
   * Включение правого мотора
   */
  this->_isRightMotorOn = true;

  int pwmValue = map(this->_rightMotorPower, 0, 100, 0, 1023);

  if (this->_rightDir == FW)
  {
    analogWrite(R_F, pwmValue);
    digitalWrite(R_B, LOW);
  }
  else
  {
    analogWrite(R_B, pwmValue);
    digitalWrite(R_F, LOW);
  }
}

void RPlatform::run()
{
  /**
   * Включение двух моторов одновременно
   */
  this->startLeftMotor();
  this->startRightMotor();
}

void RPlatform::runTime(float seconds)
{
  /**
   * Включение моторов на заданный промежуток времени
   */
  this->run();
  delay(seconds * 1000);
  this->stop();
}

void RPlatform::runSteps(int steps)
{
  /**
   * Запуск моторов на определённое количество шагов
   */
  int startLCount = RPlatform::getLeftSteps();
  int startRCount = RPlatform::getRightSteps();

  this->run();

  while (steps > RPlatform::getLeftSteps() - startLCount && steps > RPlatform::getRightSteps() - startRCount)
  {
    Serial.print(steps);
    Serial.println(RPlatform::getLeftSteps() - startLCount);
  }

  this->stop();
}

void RPlatform::runAngle(int angle)
{
  /**
   * Поворот колёс на заданный угол
   */
  int steps = (angle * FULL_TURN_STEPS) / 360;
  this->runSteps(steps);
}

void RPlatform::turnLeft(int angle)
{
  /**
   * Поворот платформы налево
   */
  direction currentLeftDir = this->_leftDir;
  direction currentRightDir = this->_rightDir;

  this->setDirection(BW, FW);
  this->runAngle(angle * PLATFORM_TURN);
  this->setDirection(currentLeftDir, currentRightDir);
}

void RPlatform::turnRight(int angle)
{
  /**
   * Поворот платформы направо
   */
  direction currentLeftDir = this->_leftDir;
  direction currentRightDir = this->_rightDir;

  this->setDirection(FW, BW);
  this->runAngle(angle * PLATFORM_TURN);
  this->setDirection(currentLeftDir, currentRightDir);
}

int RPlatform::getLeftSteps()
{
  /**
   * Возвращает количество шагов левого мотора
   */
  return RPlatform::_leftStepsCount;
}

int RPlatform::getRightSteps()
{
  /**
   * Возвращает количество шагов правого мотора
   */
  return RPlatform::_rightStepsCount;
}

void RPlatform::incLeftSteps()
{
  RPlatform::_leftStepsCount++;
}

void RPlatform::incRightSteps()
{
  RPlatform::_rightStepsCount++;
}
