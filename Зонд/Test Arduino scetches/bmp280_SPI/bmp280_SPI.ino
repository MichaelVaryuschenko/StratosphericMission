#include <SPI.h>                                    // Стандартная библиотека шины SPI
#include <Adafruit_BMP280.h>                        // Библиотека датчика
#define BMP_CS 10                                   // CS подключено на 10 пин
Adafruit_BMP280 bmp280SPI(BMP_CS);                  // Создаем объект bmp280SPI
 
void setup() {
  Serial.begin(9600);                               // Инициализируем монитор порта на скорости 9600
  Serial.println("Перезагрузка");                   // Печатаем: Перезагрузка
  if(bmp280SPI.begin())                             // Инициализируем датчик 
    Serial.println("BMP280 подключен по SPI");      // Если успешно печатаем: BMP280 подключен по I2C
    else{                                           // Иначе
      Serial.println("BMP280 не подключен");        // Печатаем: BMP280 не подключен
      while(1);                                     // Заканчиваем выполнение
        }
  delay(2000);                                      // Пауза в 2 секунды  
}  
void loop() {
  float temperature = bmp280SPI.readTemperature();  // Считываем показания температуры в градусах Цельсия
  float pressure = bmp280SPI.readPressure();        // Считываем показания давления в Паскалях
  float altitude = bmp280SPI.readAltitude(1013.25); // Расчитываем высоту относительно уровня моря  
  Serial.print("Temperature = ");                   // Печатаем Temperature = 
  Serial.print(temperature);                        // Печатаем значение температуры в грудусах Цельсия
  Serial.println(" *C");                            // Печатаем *С
 
  Serial.print("Pressure = ");                      // Печатаем Pressure = 
  Serial.print(pressure);                           // Печатаем значение давления в Паскалях
  Serial.println(" Па");                            // Печатаем Па

  Serial.print("Pressure = ");                      // Печатаем Pressure = 
  Serial.print(pressure*0.00750063755419211);       // Печатаем значение давления переведенное в мм рт.ст.
  Serial.println(" мм рт.ст.");                     // Печатаем мм рт.ст.
 
  Serial.print("Altitude = ");                      // Печатаем Altitude =
  Serial.print(altitude);                           // Печатаем значение высоты относительно уровня моря в метрах
  Serial.println(" м");                             // Печатаем м 
  Serial.println();                                 // Печатаем пустую строку для разделения
  delay(2000);                                      // Пауза в 2 секунды
}
