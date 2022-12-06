/*
Объезд препятсвий с помощью датчика расстояния.
Датчик подключается к первому слоту.
*/

// импорт библиотеки
#include <RPlatform.h>

// объявляем объект RPlatform
RPlatform robot;

void setup()
{
  // Запуск программы после нажатия на Start с паузой в три секунды
  while (!robot.isStartPressed())
  {
  }
  delay(3000);

  robot.setPower(10); // задаём мощность моторов
  robot.run();        // включаем моторы
}

void loop()
{

  // считываем показания датчика в первом слоте
  if (robot.readSensor(1) > 20)
  {
    robot.stop();               // останавливаем платформу
    robot.setDirection(BW);     // направление вращения моторов - "назад"
    robot.runSteps(44);         // запускаем вращение моторов на 44 шага
    robot.setDirection(BW, FW); // направление вращения для левого и правого моторов - "назад" и "вперёд"
    robot.runSteps(24);         // запускаем вращение моторов на 44 шага
    robot.setDirection(FW);     // направление вращения моторов - "вперёд"
    robot.run();                // включаем моторы
  }
}
