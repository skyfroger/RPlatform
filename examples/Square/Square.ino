/*
Движение платформы по квадратной траектории.
*/

#include <RPlatform.h> // импорт библиотеки

RPlatform robot; // создание объекта класса RPlatform

void setup()
{
  /*
   * В условии цикла проверяем, нажата ли кнопка Start.
   * Основная часть программы запуститься с паузой в 3 секунды
   * только после нажатия на кнопку.
   */
  while (!robot.isStartPressed())
  {
  }
  delay(3000);
}
void loop()
{
  robot.setPower(20); // мощность моторов - 20
  robot.runTime(1);   // движение 1 секунду вперёд
  robot.turnLeft(90); // поворот налево на 90 градусов
}
