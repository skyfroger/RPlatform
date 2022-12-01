# RPlatform

Arduino-библиотека для управления РОББО Платформой.

## Установка

## Примеры скетчей

## Подключение

Для импорта библиотеки в скетч, нужно добавить в начале следующую строку:

```c++
#include <RPlatform.h>
```

Добавить эту строку можно вручную или с помощью меню **Скетч - Подключить библиотеку - RPlatform**.

## Документация

Перед тем, как обращаться к следующим методам, нужно создать объект класса `RPlatform`:

```c++
#include <RPlatform.h> // подключение библиотеки

// объект класса RPlatform создаётся вне функции setup() и loop()
RPlatform robot;

void setup(){
    // начальная настройка
}

void loop(){
    // главыный цикл Arduino-скетча
}
```

### Чтение показаний датчика

```c++
int readSensor(int portNumber, bool raw = false);
```

Функция возвращает показания датчика на позиции с номером `portNumber`. Значение показаний находятся в диапазоне *0..100*. Если вторым аргументом передать значение `true`, то функция вернёт *сырое* значение (*0..1023*).

Пример:

```c++
int sensor = robot.portNumber(1); // данные с датчика на первой позиции
int rawData = robot.portNumber(2, true); // сырые данные с датчика на второй позиции
```

### Кнопка START

```c++
bool isStartPressed();
```

Функция возвращает `true`, если кнопка START нажата, иначе возвращается `false`.

Пример:

```c++
// цикл выполняется, пока кнопка не нажата
while(!robot.isStartPressed()) {}
```

С помощью такого цикла можно остановить программу до нажатия на кнопку START.

### Мощность моторов

```c++
void setPower(int power);
void setPower(int powerL, int powerR);
```

С помощью функции можно изменить мощность, как обоих моторов одновременно, так и по отдельности. Значение мощности указывается в диапазоне *0..100*. Если указать значение, которое выходит за диапазон, то оно будет приведено к ближайшему допустимому - *0* или *100*.

Пример:

```c++
robot.setPower(40); // мощность обоих моторов - 40
robot.setPower(5, 30); // левый мотор - 5, правый мотор - 30
```

### Направление вращения моторов

```c++
void setDirection(direction dir);
void setDirection(direction dirL, direction dirR);
```

Функция задаёт направление вращения моторов. В качестве аргументов передаются два возможных значения-константы:

```c++
FW // движение вперёд (forward)
BW // движение назад (backward)
```

Пример:

```c++
robot.setDirection(FW); // направление обоих двигателей - "вперёд"
robot.setDirection(BW); // направление обоих двигателей - "назад"
robot.setDirection(BW, FW); // левый двигатель - "назад", правый двигатель - "вперёд"
```

### Одновременная настройка мощности и направления вращения

```c++
void setMovementParams(direction dirL, direction dirR, int powerL, int powerR);
```

Функция позволяет изменить направление вращения левого двигателя `dirL` и правого двигателя `dirR`, а также мощность левого двигателя `powerL` и правого двигателя `powerR`.

Пример:

```c++
/*
Направление обоих двигателей - "вперёд"
Мощность левого мотора - 10
Мощность правого мотора - 20
*/
robot.setMovementParams(FW, FW, 10, 20);

/*
Направление левого двигателя - "назад"
Направление правого двигателя - "вперёд"
Мощность обоих двигателей - 35
*/
robot.setMovementParams(BW, FW, 35, 35);
```

### Запуск и остановка моторов

```c++
void startMotors(); // запуск моторов
```

```c++
void stopMotors(); // остановка моторов
```

Моторы работают до тех пор, пока не будут остановлены функцией `stopMotors()`.

Пока моторы запущены, можно менять их мощность и направление вращения. Изменения сразу будут отражаться на поведении платформы.

### Запуск моторов на определённое время

```c++
void startForSec(float seconds);
```

Функция запустит двигатели на `seconds` секунд.

Пример:

```c++
robot.setDirection(FW); // направление движения - "вперёд"
robot.setPower(20); // мощность - 20

// Платформа будет ехать прямо 4 секунды
robot.startForSec(4);

robot.setDirection(BW); // направление движения - "назад"
// Платформа будет ехать назад пол секунды
robot.startForSec(0.5);
```

