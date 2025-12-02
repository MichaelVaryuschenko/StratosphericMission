#include <SX126x_driver.h>
#include <GyverPWM.h>
#include <SPI.h>                                    // Стандартная библиотека шины SPI
#include <Adafruit_BMP280.h>                        // Библиотека датчика
#define BMP_CS 5                                   // CS подключено на 10 пин
Adafruit_BMP280 bmp280SPI(BMP_CS); 

//Number of seconds for interrupt
volatile int sec_interrupt = 0;
volatile int PID_count =0 ;
volatile int transmit_count=0;

//Pid regulator
int32_t voltage = 0;
int32_t target_voltage = 270;
int32_t err = 175;
int32_t err_i = 0;
int32_t err_old = 0;
int pulse_width = 25;
const float k_p = 0.1;
const float k_i = 0.01;
const float k_d = 0;


//Dosimeter variables
volatile unsigned long counter = 0;
volatile uint32_t Value = 0;
volatile bool flagg = false;
//Voltage stabilizer PID
/*uint16_t voltage = 0;
const uint16_t target_voltage = 175;
float pulse_width = 25;
const float k_i = 1;*/

// Pin setting
int8_t nssPin = 10, resetPin = 8, busyPin = 4, irqPin = 3;

// Clock reference setting. RF module using either TCXO or XTAL as clock reference
// uncomment code below to use XTAL
//#define SX126X_XTAL
//uint8_t xtalCap[2] = {0x12, 0x12};
// uncomment code below to use TCXO
#define SX126X_TCXO
uint8_t dio3Voltage = SX126X_DIO3_OUTPUT_1_8;
uint32_t tcxoDelay = SX126X_TCXO_DELAY_10;

// Configure DIO2 as RF switch control or using TXEN and RXEN pin
#define SX126X_USING_TXEN_RXEN

// RF frequency setting
uint32_t rfFrequency = 915000000UL;

// PA and TX power setting
uint8_t paDutyCycle = 0x02;
uint8_t hpMax = 0x03;
uint8_t deviceSel = 0x00;
uint8_t power = 0x16;

// Define modulation parameters setting
uint8_t sf = 7;                               // spreading factor 7
uint8_t bw = SX126X_BW_125000;                // 125 kHz
uint8_t cr = SX126X_CR_4_5;                   // 4/5 code rate
uint8_t ldro = SX126X_LDRO_OFF;               // low data rate optimize off

// Define packet parameters setting
uint16_t preambleLength = 12;                 // 12 bytes preamble
uint8_t headerType = SX126X_HEADER_EXPLICIT;  // explicit packet header
uint8_t payloadLength = 64;                   // 64 bytes payload
uint8_t crcType = SX126X_CRC_ON;              // cyclic redundancy check (CRC) on
uint8_t invertIq = SX126X_IQ_STANDARD;        // standard IQ setup

// SyncWord setting
uint8_t sw[2] = {0x34, 0x44};

volatile bool transmitted = false;

void checkTransmitDone() {
  transmitted = true;
  digitalWrite(2, HIGH);
}

void settingFunction() {

  Serial.println("-- SETTING FUNCTION --");

  // Pin setting
  Serial.println("Setting pins");
  sx126x_setPins(nssPin, busyPin);
  pinMode(irqPin, INPUT);

  // Reset RF module by setting resetPin to LOW and begin SPI communication
  Serial.println("Resetting RF module");
  sx126x_reset(resetPin);
  sx126x_begin();

  // Set to standby mode
  sx126x_setStandby(SX126X_STANDBY_RC);
  if (!sx126x_busyCheck()) {
    Serial.println("Going to standby mode");
  } else {
    Serial.println("Something wrong, can't set to standby mode");
  }

  // Optionally configure TCXO or XTAL used in RF module
#ifdef SX126X_TCXO
  Serial.println("Set RF module to use TCXO as clock reference");
  sx126x_setDio3AsTcxoCtrl(dio3Voltage, tcxoDelay);
#endif
#ifdef SX126X_XTAL
  Serial.println("Set RF module to use XTAL as clock reference");
  sx126x_writeRegister(SX126X_REG_XTA_TRIM, xtalCap, 2);
#endif



  // Set packet type to LoRa
  Serial.println("Set packet type to LoRa");
  sx126x_setPacketType(SX126X_LORA_MODEM);

  // Set frequency to selected frequency (rfFrequency = rfFreq * 32000000 / 2 ^ 25)
  Serial.print("Set frequency to ");
  Serial.print(rfFrequency / 1000000);
  Serial.println(" Mhz");
  uint32_t rfFreq = ((uint64_t) rfFrequency * 33554432UL) / 32000000UL;
  sx126x_setRfFrequency(rfFreq);

  // Set tx power to selected TX power
  Serial.print("Set TX power to ");
  Serial.print(power, DEC);
  Serial.println(" dBm");
  sx126x_setPaConfig(paDutyCycle, hpMax, deviceSel, 0x01);
  sx126x_setTxParams(power, SX126X_PA_RAMP_200U);

  // Configure modulation parameter with predefined spreading factor, bandwidth, coding rate, and low data rate optimize setting
  Serial.println("Set modulation with predefined parameters");
  sx126x_setModulationParamsLoRa(sf, bw, cr, ldro);

  // Configure packet parameter with predefined preamble length, header mode type, payload length, crc type, and invert iq option
  Serial.println("Set packet with predefined parameters");
  sx126x_setPacketParamsLoRa(preambleLength, headerType, payloadLength, crcType, invertIq);

  // Set predefined syncronize word
  Serial.print("Set syncWord to 0x");
  Serial.println((sw[0] << 8) + sw[1], HEX);
  sx126x_writeRegister(SX126X_REG_LORA_SYNC_WORD_MSB, sw, 2);
}

uint16_t transmitFunction(char* message, uint8_t length, uint32_t timeout) {
  

  Serial.println("\n-- TRANSMIT FUNCTION --");

  // Set buffer base address
  //Serial.println("Mark a pointer in buffer for transmit message");
  sx126x_setBufferBaseAddress(0x00, 0x80);

  // Write the message to buffer
  uint8_t* msgUint8 = (uint8_t*) message;
  Serial.print("Write message \'");
  Serial.print(message);
  Serial.println("\' in buffer");
  //Serial.print("Message in bytes : [ ");
  sx126x_writeBuffer(0x00, msgUint8, length);
  //for (uint8_t i = 0; i < length; i++) {
  //  Serial.print((uint8_t) message[i]);
  //  Serial.print("  ");
  //}
  //Serial.println("]");

  // Set payload length same as message length
  //Serial.print("Set payload length same as message length (");
  //Serial.print(length);
  //Serial.println(")");
  sx126x_setPacketParamsLoRa(preambleLength, headerType, length, crcType, invertIq);

  // Activate interrupt when transmit done on DIO1
  //Serial.println("Set TX done and timeout IRQ on DIO1");
  uint16_t mask = SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT;
  sx126x_setDioIrqParams(mask, mask, SX126X_IRQ_NONE, SX126X_IRQ_NONE);
  // Attach irqPin to DIO1
  //Serial.println("Attach interrupt on IRQ pin");
  attachInterrupt(digitalPinToInterrupt(irqPin), checkTransmitDone, RISING);


  // Calculate timeout (timeout duration = timeout * 15.625 us)
  uint32_t tOut = timeout * 64;
  // Set RF module to TX mode to transmit message
  //Serial.println("Transmitting LoRa packet");
  sx126x_setTx(tOut);
  uint32_t tStart = millis(), tTrans = 0;

  // Wait for TX done interrupt and calcualte transmit time
  //Serial.println("Wait for TX done interrupt");
  //while (!transmitted) delayMicroseconds(4);
  tTrans = millis() - tStart;
  // Clear transmit interrupt flag
  transmitted = false;
  Serial.println("Packet transmitted!");

  // Display transmit time
  //Serial.print("Transmit time = ");
  //Serial.print(tTrans);
  //Serial.println(" ms");

  // Clear the interrupt status
  uint16_t irqStat;
  sx126x_getIrqStatus(&irqStat);
  //Serial.println("Clear IRQ status");
  sx126x_clearIrqStatus(irqStat);

  // return interrupt status
  return irqStat;
}

void setup() {
  pinMode(BMP_CS, OUTPUT);
  pinMode(nssPin, OUTPUT);
  digitalWrite(BMP_CS, HIGH); 
  digitalWrite(nssPin, HIGH); 

  digitalWrite(BMP_CS, LOW);
  digitalWrite(nssPin, HIGH);
  // Begin serial communication
  Serial.begin(9600);                               // Инициализируем монитор порта на скорости 9600
  Serial.println("Перезагрузка");                   // Печатаем: Перезагрузка
  if(bmp280SPI.begin())                             // Инициализируем датчик 
    Serial.println("BMP280 подключен по SPI");      // Если успешно печатаем: BMP280 подключен по SPI
    else{                                           // Иначе
      Serial.println("BMP280 не подключен");        // Печатаем: BMP280 не подключен
      while(1);                                     // Заканчиваем выполнение
        }
  //delay(2000);                                      // Пауза в 2 секунды  

  digitalWrite(BMP_CS, HIGH);
  digitalWrite(nssPin, LOW);
  pinMode(9, OUTPUT);
  pinMode(2, OUTPUT);
  PWM_frequency(9, 20500, FAST_PWM);
  PWM_set(9, pulse_width);

  ACSR= 
        (0<<ACD) |   // Analog Comparator: Enabled
        (0<<ACBG) |   // Analog Comparator Bandgap Select: AIN0 is applied to the positive input
        //(1<<ACO) |   // Analog Comparator Output: On
        //(1<<ACI) |   // Analog Comparator Interrupt Flag: Clear Pending Interrupt
        (1<<ACIE) |   // Analog Comparator Interrupt: Enabled
        (0<<ACIC) |   // Analog Comparator Input Capture: Disabled
        (1<<ACIS1) | (0<ACIS0); // По НИСПАДАЮЩЕМУ фронту
  
  noInterrupts();
  TCCR2A=0;
  TCCR2B = 0;
  TCNT2 = 0;
  
  //OCR2A=15625;
  OCR2A=255;
  TCCR2A |= (1<< WGM21);
  TCCR2B |= (1<< CS22) | (1<< CS21) | (1<<CS20);
  TIMSK2 |= (1<< OCIE2A);
  interrupts();
  // Settings for LoRa communication
  settingFunction();
}

ISR(ANALOG_COMP_vect) {
    counter+=1;
}
  ISR(TIMER2_COMPA_vect) { // ПРЕРЫВАНИЕ
    flagg=true;
    sec_interrupt +=1;
    PID_count+=1;
    transmit_count+=1;
      if (PID_count>=60){
      PID_count=0;

      voltage = (float)analogRead(A0) / 1023 * 420;
      err = target_voltage - voltage;
      pulse_width += k_p * (target_voltage - voltage)  + k_i * err_i + k_d * (err - err_old);
      if(pulse_width > 128)
        pulse_width = 128;
      PWM_set(9, pulse_width);
      err_old = err;
      err_i += min(err, 20);
      Serial.println("---------------------------");
      Serial.print("Dosimeter voltage: ");
      Serial.println(voltage);
      Serial.print("PWM fill coefficient: ");
      Serial.println((float)pulse_width / 255);
      Serial.print("Error: ");
      Serial.println(err);
      Serial.print("Integral error: ");
      Serial.println(err_i);
      Serial.println("---------------------------");
      
    }
    if(sec_interrupt>=50) {
      digitalWrite(2, LOW);
    }
    
}
void loop() {digitalWrite(BMP_CS, LOW); // выбор датчика в качестве собеседника
  

  
  
  char message[4];
  uint8_t nBytes = sizeof(message);

    // Transmit message
  uint32_t timeout = 1000; // 1000 ms timeout
  
  if (sec_interrupt>=600){
    sec_interrupt=0;
    if (flagg){ // то есть, это выполняется только по прерыванию для дозиметра
      flagg = false;
      Value=counter;
      counter=0;

      Serial.println("+++++++++++++++++++++++++++");
      Serial.print("Dosimeter readings: ");
      Serial.print((float)Value / 62);
      Serial.print(" uSv/h; Dosimeter voltage: ");
      Serial.println(voltage);
      /*Serial.print("A0 readings: ");
      Serial.println(analogRead(A0));
      Serial.print("PWM fill coefficient: ");
      Serial.println((float)pulse_width / 255);*/
      Serial.println("+++++++++++++++++++++++++++");
      for (int i = 0; i < 4; i++) {
        message[3 - i] = Value % 10 + 48;
        Value /= 10;
      }
      uint16_t status = transmitFunction(message, 4, timeout);
      digitalWrite(nssPin, HIGH);

    float temperature = bmp280SPI.readTemperature();  // Считываем показания температуры в градусах Цельсия
    float pressure = bmp280SPI.readPressure();        // Считываем показания давления в Паскалях
    float altitude = bmp280SPI.readAltitude(1013.25); // Расчитываем высоту относительно уровня моря  
//    Serial.print("Temperature = ");                   // Печатаем Temperature = 
//    Serial.print(temperature);                        // Печатаем значение температуры в грудусах Цельсия
//    Serial.println(" *C");                            // Печатаем *С
  
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
    //delay(2000);                                      // Пауза в 2 секунды

    digitalWrite(BMP_CS, HIGH);
    digitalWrite(nssPin, LOW);

    }
  }

  
    /* Вне  if нужно тоже добавить, так как контролировать выходное напряжение преобразователя 
    нужно всегда, не только в момент вывода на экран */

    // Message to transmit
    
    
    // Display status if error
//    if (status & SX126X_IRQ_TIMEOUT){
//      Serial.println("Transmit timeout");
//    }

    // Don't load RF module with continous transmit
    /*delay(10000); */ //Ждем мертвые 10 секунд, избавиться - реализовать по таймеру функцию прерывания, 
    //в котором изменяются флаги 
  
}