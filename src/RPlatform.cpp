#include "Arduino.h"
#include "RPlatform.h"

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
}

String RPlatform::getLog()
{
  /**
   * Вывод отладочной информации
   * Метод вернёт строку с текущими настройками платформы и данными с портов.
   */
  char buffer[128];
  sprintf(buffer, "LM: p=%d dir=%d; RM: p=%d dir=%d; S1=%d; S2=%d; S3=%d; S4=%d; S5=%d",
          this->_leftMotorPower, this->_leftDir,
          this->_rightMotorPower, this->_rightDir,
          this->readSensor(1), this->readSensor(2),
          this->readSensor(3), this->readSensor(4),
          this->readSensor(5));
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

uint16_t RPlatform::readSensor(uint8_t portNumber, bool raw = false)
{
  /**
   * Возвращаем значение, измеренное датчиком в указанном слоте
   */
  uint8_t rawValue = analogRead(this->_sensorPorts[portNumber]);
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

void RPlatform::setRightMotorPower(uint8_t power)
{
  /**
   * Изменяем мощность правого мотора
   */
  uint8_t p = constrain(power, 0, 100);
  this->_rightMotorPower = p;

  if (this->_isRightMotorOn)
  {
    this->startRightMotor();
  }
}

void RPlatform::setLeftMotorPower(uint8_t power)
{
  /**
   * Изменяем мощность левого мотора
   */
  uint8_t p = constrain(power, 0, 100);
  this->_leftMotorPower = p;

  if (this->_isLeftMotorOn)
  {
    this->startLeftMotor();
  }
}

void RPlatform::setPower(uint8_t power)
{
  /**
   * Выбор одинаковой мощности для обоих моторов
   */
  this->setLeftMotorPower(power);
  this->setRightMotorPower(power);
}

void RPlatform::setPower(uint8_t powerL, uint8_t powerR)
{
  /**
   * Выбор мощности для каждого мотора отдельно
   */
  this->setLeftMotorPower(powerL);
  this->setRightMotorPower(powerR);
}

void RPlatform::setRunSettings(direction dirL, direction dirR, uint8_t powerL, uint8_t powerR)
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

  uint8_t pwmValue = map(this->_leftMotorPower, 0, 100, 0, 1023);

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

  uint8_t pwmValue = map(this->_rightMotorPower, 0, 100, 0, 1023);

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
