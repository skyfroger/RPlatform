/*
Простейший алгоритм следования по линии.
Датчик линии подключён к первому слоту.
*/

#include <RPlatform.h>

// среднее значение между минимальным и максимальным показаниями датчика линии
#define BORDER 37

RPlatform robot; // создаём объект класса RPlatform

void setup()
{
  /*
     В условии цикла проверяем, нажата ли кнопка Start.
     Основная часть программы запуститься с паузой в 3 секунды
     только после нажатия на кнопку.
  */
  while (!robot.isStartPressed())
  {
  }
  delay(3000);

  robot.run(); // запускаем моторы
}

int lineSensor;

void loop()
{

  if (robot.readSensor(1) > BORDER)
  {
    robot.setPower(3, 20);
  }
  else
  {
    robot.setPower(20, 3);
  }
}