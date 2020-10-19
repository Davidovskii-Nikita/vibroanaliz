# Инструкция по применению
### Содержание:
* [Готовая реализация](https://github.com/Davidovskii-Nikita/vibroanaliz#%D0%B3%D0%BE%D1%82%D0%BE%D0%B2%D0%B0%D1%8F-%D1%80%D0%B5%D0%B0%D0%BB%D0%B8%D0%B7%D0%B0%D1%86%D0%B8%D1%8F)
* [Как работает код?](https://github.com/Davidovskii-Nikita/vibroanaliz/blob/master/README.md#%D0%BA%D0%B0%D0%BA-%D1%80%D0%B0%D0%B1%D0%BE%D1%82%D0%B0%D0%B5%D1%82-%D0%BA%D0%BE%D0%B4)
* [Как настроить?](https://github.com/Davidovskii-Nikita/vibroanaliz/blob/master/README.md#%D0%BA%D0%B0%D0%BA-%D0%BD%D0%B0%D1%81%D1%82%D1%80%D0%BE%D0%B8%D1%82%D1%8C)
* [Как прошить?](https://github.com/Davidovskii-Nikita/vibroanaliz/blob/master/README.md#%D0%BA%D0%B0%D0%BA-%D0%BF%D1%80%D0%BE%D1%88%D0%B8%D1%82%D1%8C)
* [Как отладить?](https://github.com/Davidovskii-Nikita/vibroanaliz/blob/master/README.md#%D0%BA%D0%B0%D0%BA-%D0%BE%D1%82%D0%BB%D0%B0%D0%B4%D0%B8%D1%82%D1%8C)
* [Подводные камни.](https://github.com/Davidovskii-Nikita/vibroanaliz/blob/master/README.md#%D0%BF%D0%BE%D0%B4%D0%B2%D0%BE%D0%B4%D0%BD%D1%8B%D0%B5-%D0%BA%D0%B0%D0%BC%D0%BD%D0%B8)
---

### Готовая реализация.

Электрическая схема:

![ПЭС устройства](https://github.com/Davidovskii-Nikita/vibroanaliz/blob/master/docs/%D0%AD%D0%BB%D0%B5%D0%BA%D1%82.%20%D1%81%D1%85%D0%B5%D0%BC%D0%B0.jpg)

Для прошивки нужно перейти на сайт [http://esp-01_black.local/firmware](http://esp-01_black.local/firmware) или набрать в браузере IP-адрес. После подачи питания *ESP-01* подключится к WiFi-сети, и на корпусе загорится синий светодиод, сигнализирующий о корректной работе устройства.

---

### Как работает код?
Файл [main.ino](https://github.com/Davidovskii-Nikita/vibroanaliz/blob/master/main_sketch/main.ino), содержит в себе прошивку для ESP-01 (1 Mb *"черная"*). Код выполняет несколько функций: 
* **Собирает данные с датчика акселерометра и температуры (плата MPU6050).**

По протоколу I2C.
Используется библиотека:
``` c
#include <Wire.h>  
```
Функции:
``` c
double Read_RawValue_AXEL(uint8_t deviceAddress, uint8_t regAddress)    
void Read_RawValue_TEMP(uint8_t deviceAddress, uint8_t regAddress) 
void MPU6050_Init()  
void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data)
```
* **Обрабатывает полученные значения с помощью цифровых фильтров.**
В наличии постоянное получение СКЗ ускорения по трем осям в функции   
``` c
double Read_RawValue_AXEL(uint8_t deviceAddress, uint8_t regAddress)
```
А также существуют цифровые фильтры:
``` c 
float filter_kalman(float value)  //Калмана
float running_midle(float Z)      //ФНЧ
void init_moda(float diap)        //мода  
```
В перспективе ждут медианный и альфа-бета фильтры. 

* **Получает точное значение времени.**

Используется библиотека для подключения к NTP-серверам:
``` c
#include <NTPClient.h>
```

Значение получается с помощью функции 
``` c
String getTime() // Значение передается в формате UNIX.
```
* **Собирает данные в локальный буфер.**

Сбор происходит в цикле loop в переменные:
``` c
String opros_axel [RANGE_A];
String opros_axel_time [RANGE_A];
String opros_temp [RANGE_T];
String opros_temp_time [RANGE_T];
```
* **Отправляет JSON документ с собранной информацией на сервер.**

Используется библиотека:
``` c
#include <ESP8266HTTPClient.h
#include <ArduinoJson.h> 
#include <ESP8266WiFi.h> 
```
Отвечает функция 
```c
void post_json(void)
```
B которой создается и уничтожается JSON документ, а также создается связь с сервером и отправляются данные. **Данные отправляются с помощью POST запросов.**
* **Имеет возможность перепрошиться через браузер.**

Используются библиотеки:
~~~ c
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
~~~
Создается локальное имя в зоне **.lосаl** (mDNS). Для прошивки нужно знать логин и пароль.

---
### Как настроить?
Для настройки прошивки требуется изменить некоторые строки в блоке кода с. 12 - с. 42.

**Для того чтобы получать правильное время с NTP cерверов, требуется выбрать ближайший к вам NTP сервер(*by.pool.ntp.org*) и выбрать смещение часового пояса(*10800*) в строке 14:**
~~~ c
NTPClient timeClient(ntpUDP, "by.pool.ntp.org", 10800);// выбор сервера NTP("by.pool.ntp.org") //смещение пояса(10800)
~~~
Сервера выбирать [тут](https://www.ntp-servers.net/servers.html).

**Следующие 4 строки отвечают за количество данных в буфере:**
``` c
#define RANGE_A 100        //количество данных опроса в 1 цикл по ускорению
#define RANGE_T 10         //количество данных опроса в 1 цикл по температуре
```
И частоту опроса датчиков:
``` c
#define PERIOD_A 1000;    // частота опроса СКЗ ускорения 3 оси
#define PERIOD_T 10000;   // частота опроса температуры
```
Рекомендуется оставить существующие значения, так как память ESP-01 **ограничена**. Таким образом, контроллер будет отправлять значения каждые 100 секунд.

**Далее идут настройки цифровых фильтров**. Но до этого следует подключить фильтр. Для этого следует обратиться к функции *double filter()* (с. 181), закомментировать строку:
~~~ c
val_filter=Z;
~~~
И раскомментировать любую из следующих.

Калман:
~~~ c
val_filter=filter_kalman(Z);
~~~ 
ФНЧ:
~~~ c
val_filter=running_midle(Z);
~~~
Мода:
~~~ c
val_filter=filter_moda(Z);
~~~
После этого можно менять коэффициенты каждого из фильтров в соответствующих строках. Существующие коэффициенты были получены **экспериментально**.

**Со строки 35 идут настройки WiFi и сервера.** Следовательно, Вам требуется написать название Вашей WiFi сети и пароль от нее в строках:
~~~ C
const char* ssid = "SSID";          //имя Wi-Fi сети
const char* password = "PASS";      //пароль cети
~~~
Требуется написать адрес сервера и адрес URL, на который будут приходить POST запросы, в строках:
~~~ C
const char* host ="host.com";       // адрес хоста
String URL="http://host.com/data";  //адрес, куда отправляются POST запросы
~~~

**Для удобства прошивки через браузер рекомендуется выбрать уникальное имя для каждой ESP-01**
~~~ C
const char* host_OTA = "esp-01_black";  // название устройства в локальной сети для прошивки через браузер 
~~~
Тогда адрес будет иметь вид: http://esp-01_black.local/firmware

**Настроить чувствительность акселерометра.** Для этого требуется вписать соответствующую цифру.
~~~ C
const uint16_t Full_Scale_Range=4;// выбор диапазона измерений акселерометра +-2,+-4,+-8,+-16
~~~
В данном случае выбран диапазон +-4g. [Спецификация](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf) и [мануал](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf) акселерометра.

---
### Как прошить?
Для того чтобы прошить ESp-01 через браузер, требуется, чтобы на плате **уже** была залита прошивка, позволяющая сделать *OTA-обновление*, а также Вам требуется знать *локальное имя контроллера* или его *локальный IP-адрес*. Для этого нужно перейти на сайт [http://esp-01_black.local/firmware](http://esp-01_black.local/firmware) (адрес сайта настраивается в строке 38) или набрать в браузере IP-адрес ESP-01(192.168.0.1/firmware), и залить BIN-файл, который создается с помощью сочетания клавиш *Ctrl+ALt+S* или по пути *Скетч-Экспорт бинарного файла* в программе *Arduino IDE*. Bin-файл прошивки после создания будет лежать в папке со скетчем. Обязательное требование: **Компьютер и ESP-01 должны быть подключены к одной WiFi сети!** 

Для загрузки **первого** скетча Вам потребуется:
* Конвертер USB-TO-TTL;
* Макетная плата;
* Провода типа мама-папа и папа-папа;
* ESP-01.

Схема подключения будет выглядеть следующим образом:

![Cхема прошивки ESP-01 через USB-TO-TTL](https://github.com/Davidovskii-Nikita/vibroanaliz/blob/master/docs/C%D1%85%D0%B5%D0%BC%D0%B0%20%D0%BF%D1%80%D0%BE%D1%88%D0%B8%D0%B2%D0%BA%D0%B8.png)
*На схеме ESP-01 отображена в перевернутом виде.*
При подключении конвертера к USB порту на плате должен загореться диод **POWER**. Перед прошивкой рекомендуется проверить правильность подключения, присоединив к GND порт RST на секунду. При отключения этого порта от земли должен *моргнуть* диод **DATA** на плате конвертера и синий диод на плате ESP-01. В этом случае можно прошивать плату. Во время прошивки на корпусе *ESP-01* должен мигаить синий свретодиод.
Для прошивки (и формировании bin-файла) удобно использовать *Arduino IDE*. Для этого должны быть установлены все требуемые библиотеки и настроены дополнительные ссылки *Менеджера плат*. 

### Как отладить?
Для отладки работы устройства удобнее подключить его к компьютеру через конвертер по схеме:
![Схема отладки ESP-01 через USB-TO-TTL](https://github.com/Davidovskii-Nikita/vibroanaliz/blob/master/docs/%D0%A1%D1%85%D0%B5%D0%BC%D0%B0%20%D0%BE%D1%82%D0%BB%D0%B0%D0%B4%D0%BA%D0%B8.png)
*На схеме ESP-01 отображена в перевернутом виде.*
Для отладки требуется **написать** следующий код:
~~~ C
Serial.print("Name");
Serial.print(Value);
Serial.println('\t');
~~~~
Где Name-название переменной (упрощает отладку). Value - интересующая переменная. 

При использовании сочетания клавиш *Ctrl-Shift-M* откроется монитор последовательного порта. Если нужно оценить **динамику** изменений, для **визуализации** удобно использовать встроенный в Arduino IDE плоттер последовательного порта, включить который можно *Ctrl-Shift-L*.

**Во время отладки**, при просмотре значений через последовательный порт **возможно уменьшение** производительности *ESP-01*.

### Подводные камни.

**Одним из самых важных моментов** является организация достаточного питания на все элементы схемы. При работе с устройством в режиме отладки требуется собрать схему с **надежной** посадкой проводов и избегать **дребезг** контактов.

**В процессе прошивки** устройства через браузер цикл прошивки может повториться, что может привести к неправильной прошивке *ESP-01*. В этом случае нужно **дождаться** конца прошивки и перезагрузки модуля. Так же, сайт , в котором происходила отправка прошивки, может уходить в бесконечную загрузку.

**В коде скетча** реализован автоматический расчет объема динамического *JSON документа*, выходить за рамки диапазонов **не рекомендуется**.

**Во время работы** кода возможны потери данных в объеме нескольких секунд за цикл.

**Во время работы** температура *ESP-01* может достигать 35 °C.

**Возможна потеря данных** времени с *NTP-серверов*, получаемые данные  будут содержать дату *07.02.2036*. В этом случае помогает перезагрузка. 
 
**Изменение памяти**

![Cтатистика памяти](https://github.com/Davidovskii-Nikita/vibroanaliz/blob/master/docs/%D0%A1%D1%82%D0%B0%D1%82%D0%B8%D1%81%D1%82%D0%B8%D0%BA%D0%B0%20%D0%BF%D0%B0%D0%BC%D1%8F%D1%82%D0%B8.PNG)

При добавлении **одинакового** колличества требуемых **данных** ускорения и температуры (на 50 и 5) увеличение объема *JSON-файла* увеличивалось на **одинаковую** величину, как видно в стобце 4. Однако значение оставшейся **памяти увеличивалось ( и иногда уменьшалось) непостоянно** (стобец 6). Экспериментально было найдена примерная величина **максимально** возможного *JSON-файла*, она равняется **45%** от всего объема *ESP-01*(44000). В нашем случает максимум был достигнут на шаге значений: *450 ускорение и 45 температуры.*
