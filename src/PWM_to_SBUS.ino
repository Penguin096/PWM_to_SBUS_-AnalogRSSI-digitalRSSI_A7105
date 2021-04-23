#define RC_CHANNEL_MIN 990   // Servo minimum position
#define RC_CHANNEL_MAX 2010  // Servo maximum position

#define SBUS_UPDATE_RATE 15  //ms
#define ADCTime_UPDATE 100   //ms

#define SERVO_INT_VECTOR PCINT2_vect
ISR(PCINT0_vect, ISR_ALIASOF(PCINT2_vect));  //общий обработчик двух прерываний

#define SERVO_CHANNELS 8  // <=8
#define RSSI_CHANNEL 9
#define RSSI_PIN A0

//#define AnalogRSSI    // закомментировать для цифроаналогового входа RSSI для A7105
#define ADCRSSIMAX 47   // Значение ацп при котором RSSI =100% (47=0,05вольта)
#define ADCRSSIMIN 651  // Значение ацп при котором RSSI =0% (651=0,7вольта по осцилографу срыв связи !!!в datashit=0.8)
#define ADCSTB 1020     // Минимальное значение АЦП строба
#define ADCNRM 837      // максимальное значение АЦП сигнала rssi (837=0,9Вольта)
#define TWAIT 200       // время ожидания получения rssi //ms >ADCTime_UPDATE
#define numReadings 16  // количество отсчетов для усреднеия

#include <avr/wdt.h>

uint32_t sbusTime = 0;
uint32_t ADCTime = 0;

uint8_t channelorder[SERVO_CHANNELS + 1] = { 2, 3, 4, 5, 6, 7, 0, 1, 8 };  //, 9, 10, 11, 12, 13, 14, 15
uint8_t sbusPacket[25];
int16_t rcChannels[16];

int16_t AdcVal[5];  // масив значений АЦП для детектора положительного фронта импульса
int16_t AdcFil[3];  // масив значений АЦП для детектра среднего из трех семплов

int16_t readings[numReadings];  // the readings from the analog input
uint8_t readIndex = 0;          // the index of the current reading
int16_t total = 0;              // the running total
int16_t average = 0;            // the average (последнее усредненное значение АЦП здесь)

void pciSetup() {
  PCMSK0 = 0b00000011;  // Разрешаем PCINT для указанных пинов D13...D8
  //PCMSK1 = 0b00000000;  // Разрешаем PCINT для указанных пинов A5...A0
  PCMSK2 = 0b11111100;  // Разрешаем PCINT для указанных пинов D7...D0
  //PCICR  = 0b00000001;  // Разрешаем PCINT для соответствующей группы пинов D13...D8
  //PCICR  = 0b00000010;  // Разрешаем PCINT для соответствующей группы пинов A5...A0
  PCICR = 0b00000101;  // Разрешаем PCINT для соответствующей группы пинов D7...D0 и D9...D8
  //PCIFR = 0b00000000;  // Очищаем признак запроса прерывания
}

void fastADC() {
  // set prescale to 16
  (_SFR_BYTE(ADCSRA) |= _BV(ADPS2));
  (_SFR_BYTE(ADCSRA) &= ~_BV(ADPS1));
  (_SFR_BYTE(ADCSRA) &= ~_BV(ADPS0));
}

ISR(SERVO_INT_VECTOR) {  // Обработчик запросов прерывания от пинов
  // Servo pulse start timing
  static uint16_t servo_start[SERVO_CHANNELS] = { 0, 0, 0, 0, 0, 0, 0, 0 };

  // Servo input pin storage
  static uint8_t servo_pins_old = 0;

  // Used to store current servo input pins
  uint8_t servo_pins;

  // ------------------------------------------------------------------------------
  // SERVO PWM MODE
  // ------------------------------------------------------------------------------

  // Store current servo input pins
  servo_pins = ((PIND & ~0b11) | (PINB & 0b11));  //D9...D8 и D7...D0

  do {
    // Set initial servo pin
    uint8_t servo_pin = 1;

    // Calculate servo inputT= pin change mask
    uint8_t servo_change = servo_pins ^ servo_pins_old;

    for (uint8_t servo_channel = 0; servo_channel < SERVO_CHANNELS; servo_channel++) {

      // Check for pin change on current servo channel
      if (servo_change & servo_pin) {  // Изменился
        //HIGH (Фронт импульса)
        if (servo_pins & servo_pin) {
          servo_start[servo_channel] = micros();
        } else {

          //Получаем ширину сервоимпульса
          uint16_t servo_width = micros() - servo_start[servo_channel];

          // Check that servo pulse signal is valid before sending to ppm encoder
          if (servo_width > RC_CHANNEL_MAX)
            servo_width = rcChannels[servo_channel];  // all channels hold their previous position!
          if (servo_width < RC_CHANNEL_MIN)
            servo_width = rcChannels[servo_channel];  // all channels hold their previous position!

          // Update rcChannels[..]
          rcChannels[servo_channel] = servo_width;
        }
      }
      // Select next servo pin
      servo_pin <<= 1;
    }

    // Reset Watchdog Timer
    wdt_reset();

    // Store current servo input pins for next check
    servo_pins_old = servo_pins;
    servo_pins = ((PIND & ~0b11) | (PINB & 0b11));  //D7...D0 и D9...D8
    //Has servo input changed while processing pins, if so we need to re-check pins
  } while (servo_pins != servo_pins_old);

  // Clear interrupt event from already processed pin changes
  PCIFR |= ((1 << PCIF2) | (1 << PCIF0));
}

void sbusPreparePacket(bool digitalCH1, bool digitalCH2, bool isSignalLoss, bool isFailsafe) {

  byte SBUS_Current_Packet_Bit = 0;
  byte SBUS_Packet_Position = 0;

  for (SBUS_Packet_Position = 0; SBUS_Packet_Position < 25; SBUS_Packet_Position++) {
    sbusPacket[SBUS_Packet_Position] = 0x00;  //Zero out packet data
  }

  sbusPacket[0] = 0x0F;  //Header
  SBUS_Packet_Position = 1;

  for (byte SBUS_Current_Channel = 0; SBUS_Current_Channel < SERVO_CHANNELS + 1; SBUS_Current_Channel++) {
    /*
      Map 990-2010 with middle at 1500 chanel values to
      173-1811 with middle at 992 S.BUS protocol requires
    */
    uint16_t sbusval;
    sbusval = map(rcChannels[channelorder[SBUS_Current_Channel]], RC_CHANNEL_MIN, RC_CHANNEL_MAX, 173, 1811);

    for (byte SBUS_Current_Channel_Bit = 0; SBUS_Current_Channel_Bit < 11; SBUS_Current_Channel_Bit++) {
      if (SBUS_Current_Packet_Bit > 7) {
        SBUS_Current_Packet_Bit = 0;  //If we just set bit 7 in a previous step, reset the packet bit to 0 and
        SBUS_Packet_Position++;       //Move to the next packet uint8_t
      }
      sbusPacket[SBUS_Packet_Position] |= (((sbusval >> SBUS_Current_Channel_Bit) & 0x1) << SBUS_Current_Packet_Bit);  //Downshift the channel data bit, then upshift it to set the packet data uint8_t
      SBUS_Current_Packet_Bit++;
    }
  }

  if (digitalCH1) sbusPacket[23] |= (1 << 0);
  if (digitalCH2) sbusPacket[23] |= (1 << 1);
  if (isSignalLoss) sbusPacket[23] |= (1 << 2);
  if (isFailsafe) sbusPacket[23] |= (1 << 3);

  sbusPacket[24] = 0x00;  //Footer 0x00 for SBUS or 0x04 for SBUS2
}

void setup() {
  wdt_enable(WDTO_250MS);  //Enable watchdog

  Serial.begin(100000, SERIAL_8E2);

  // SERVO and RSSI INPUT PINS
  // ------------------------------------------------------------------------------
  DDRD &= 0b00000011;  // Set PortD to inputs
  DDRB &= 0b11111100;  // Set only PB0,PB1 PortB to inputs
  DDRC &= 0b11111110;  // Set only A0 PortC to inputs

  PORTD |= 0b11111100;  // Activate pullups PortD input pins
  PORTB |= 0b00000011;  // Activate pullups PortB input pins
  // ------------------------------------------------------------------------------

  pciSetup();

  fastADC();

#ifndef AnalogRSSI
  analogReference(INTERNAL);  // Внутреннее опорное 1.1В (между пином REF и GND поставить конденсатор
#endif

  for (uint8_t i = 0; i < 16; i++) {
    rcChannels[i] = 1500;
  }
}

void loop() {
  if ((millis() - sbusTime) > SBUS_UPDATE_RATE) {
    //Serial.write(millis() - sbusTime);  //Отладка
    sbusTime = millis();
    sbusPreparePacket(false, false, false, false);
    Serial.write(sbusPacket, 25);
  }

#ifdef AnalogRSSI
  if ((millis() - ADCTime) > ADCTime_UPDATE) {
    ADCTime = millis();
    rcChannels[RSSI_CHANNEL - 1] += map(analogRead(RSSI_PIN), 0, 1023, RC_CHANNEL_MIN, RC_CHANNEL_MAX);
    rcChannels[RSSI_CHANNEL - 1] >>= 1;
  }
#endif

#ifndef AnalogRSSI
  if ((millis() - ADCTime) > ADCTime_UPDATE) {

    AdcVal[0] = AdcVal[1];
    AdcVal[1] = AdcVal[2];
    AdcVal[2] = AdcVal[3];
    AdcVal[3] = AdcVal[4];
    AdcVal[4] = analogRead(RSSI_PIN);  //хранит последние 5 значений АЦП

    // если найден положительный фронт импульса на пине 1 A7105
    if (AdcVal[4] >= ADCSTB && AdcVal[3] >= ADCSTB && AdcVal[2] < ADCSTB && AdcVal[1] < ADCNRM && AdcVal[0] < ADCNRM) {
      ADCTime = millis();

      // Вычисляю новое значение RSSI

      AdcFil[0] = AdcFil[1];
      AdcFil[1] = AdcFil[2];
      AdcFil[2] = AdcVal[0];
      // Фильтрация первый этап - отброс импульсных помех
      // поиск среднего (отбрасываю мин и мах) значения из трех последних семплов (сумирую и отнимаю мин и макс)
      int16_t adcrssi = (AdcFil[0] + AdcFil[1] + AdcFil[2]) - max(AdcFil[0], max(AdcFil[1], AdcFil[2])) - min(AdcFil[0], min(AdcFil[1], AdcFil[2]));

      adcrssi = constrain(adcrssi, ADCRSSIMAX, ADCRSSIMIN);  // Ограничиваем диапазон значений

      // Фильтрация второй этап - усреднение
      // subtract the last reading:
      total = total - readings[readIndex];
      // read from the sensor:
      readings[readIndex] = adcrssi;
      // add the reading to the total:
      total = total + readings[readIndex];
      // advance to the next position in the array:
      readIndex = readIndex + 1;
      // if we're at the end of the array...
      if (readIndex >= numReadings) readIndex = 0;
      // calculate the average:
      average = total / numReadings;

      rcChannels[RSSI_CHANNEL - 1] = map(average, ADCRSSIMIN, ADCRSSIMAX, RC_CHANNEL_MIN, RC_CHANNEL_MAX);
    }

    // На случай отключения пина 1 чипа A7105 сигна RSSI установим в минимум
    if ((millis() - ADCTime) > TWAIT) {
      rcChannels[RSSI_CHANNEL - 1] = RC_CHANNEL_MIN;
    }
  }
#endif
}