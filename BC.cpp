#pragma GCC optimize ("Og")

#include <SoftwareSerial.h>
#include <SPI.h>
#include <TroykaIMU.h>
//#include <GOST4401_81.h>

#include <SD.h>
#define PIN_CHIP_SELECT 8

#include <Wire.h>
#define I2C_SEP   0x1
#define I2C_BUEMU 0x2
#define I2C_BUSOS 0x3
#define IC2_CMD_GET_AKB      30
#define IC2_CMD_GET_IMU      31
#define IC2_CMD_GET_PTH      43
#define IC2_CMD_GET_PTH_COEF 70

#include <TroykaGPS.h>
#define RX_GPS_PIN 2
#define TX_GPS_PIN 3

#include <RF24.h>
#define NRF_TX_ADDRESS     0xAAE10CF1F0UL
#define NRF_RX_ADDRESS     0xAAE10CF1F1UL
#define NRF_RX_PSIZE       8
#define NRF_TX_PSIZE       8
#define NRF_RETRIES_DELAY  0
#define NRF_RETRIES_COUNT  3
#define NRF_AUTO_ACK       1
#define NRF_RX_PACKET_SIZE 20
#define NRF_TX_PACKET_SIZE 32

/* Интервалы, специально сделаны переменными, чтобы
/* менять во время выполнения программы, но пока не реализовано */
#define MIN_INTERVAL_VALUE 10
uint16_t gpsUpdateInterval = 1000;
uint16_t imuUpdateInterval = 1000;
uint16_t phtUpdateInterval = 1000;
uint16_t akbUpdateInterval = 1000;
uint16_t gpsSendInterval =   1000;
uint16_t imuSendInterval =   1000;
uint16_t phtSendInterval =   1000;
uint16_t sdWriteInterval =   1000;

// Флаги
#define FLG_BUSOS_UPDATE_IMU   0x01
#define FLG_CALIB_RNG_READED   0x02
#define SD_CARD_INITIALIZATRED 0x04
uint8_t FLAGS = FLG_BUSOS_UPDATE_IMU;

SoftwareSerial gpsSerial(RX_GPS_PIN, TX_GPS_PIN);
Gyroscope      gyroscope;
Accelerometer  accelerometer;
Compass        compass;
Barometer      barometer;
GPS            gps(gpsSerial);
RF24           nrf24(9, 10);

struct Vector {
	float x = 0;
	float y = 0;
	float z = 0;
};
struct Euler {
	float x = 0;
	float y = 0;
	float z = 0;
};
struct Range {
    float min;
    float max;
};
struct Datetime {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
};

// Глобальные переменные
uint8_t stlCount = 0;
Datetime gpsDatetime;
float gpsLatitude = 0,  gpsLongitude = 0, gpsAltitude = 0, azimut = 0, altitude = 0;
int16_t gpsSpeed = 0;
float mainVoltage = 0, batteryVoltage = 0, solarVoltage = 0;
float press = 0, temp = 0;
Vector gyro, acl, mgn;
Euler rotateAngle;
uint16_t phtValues[8] = {};
Range phtCalibRange[8];

// Обновление данных
void updateAkbData() {
    Wire.beginTransmission(I2C_SEP);
    Wire.write(IC2_CMD_GET_AKB);
    Wire.endTransmission(false);

    Wire.requestFrom(I2C_SEP, 12);
    uint8_t data[12];
    for (uint8_t i = 0; i < 12; ++i) { data[i] = Wire.read(); }

    memcpy(&mainVoltage, data+0, sizeof(float));
    memcpy(&batteryVoltage, data+4, sizeof(float));
    memcpy(&solarVoltage, data+8, sizeof(float));
}
void updatePhtData() {
    Wire.beginTransmission(I2C_BUSOS);
    Wire.write(IC2_CMD_GET_PTH);
    Wire.endTransmission(false);

    Wire.requestFrom(I2C_BUSOS, 16);
    uint8_t data[16];
    for (uint8_t i = 0; i < 16; ++i) { data[i] = Wire.read(); }

    memcpy(phtValues, data, 16);
}
void updateIMUData() {
    if (FLAGS&FLG_BUSOS_UPDATE_IMU) {
        // Если обновление с БУСОС
        Wire.beginTransmission(I2C_BUSOS);
        Wire.write(IC2_CMD_GET_IMU);
        Wire.endTransmission(false);

        Wire.requestFrom(I2C_BUSOS, 23);
        press = readFloatI2C(26000, 126000, 16777215, 3);

        temp = readFloatI2C(-30, 110, 4095, 2);

        gyro.x = readFloatI2C(-250000, 250000, 65535, 2);
        gyro.y = readFloatI2C(-250000, 250000, 65535, 2);
        gyro.z = readFloatI2C(-250000, 250000, 65535, 2);

        acl.x = readFloatI2C(-8000, 8000, 65535, 2);
        acl.y = readFloatI2C(-8000, 8000, 65535, 2);
        acl.z = readFloatI2C(-8000, 8000, 65535, 2);

        mgn.x = readFloatI2C(-16000, 16000, 65535, 2);
        mgn.y = readFloatI2C(-16000, 16000, 65535, 2);
        mgn.z = readFloatI2C(-16000, 16000, 65535, 2);
    }
    else {
        accelerometer.readAccelerationAXYZ(acl.x, acl.y, acl.z);

        compass.readMagneticGaussXYZ(mgn.x, mgn.y, mgn.z);

        gyroscope.readRotationDegXYZ(gyro.x, gyro.y, gyro.z);
        azimut = compass.readAzimut();

        temp = barometer.readTemperatureC();
        press = barometer.readPressurePascals();
        altitude = 0;//GOST4401_getAltitude(press);
    }
}
void updateGPSData() {
  if (gps.available()) {
    gps.readParsing();
    // проверка состояния GPS-модуля
    switch (gps.getState()) {
      case GPS_OK:
        // Координаты
        gpsLatitude = gps.getLatitudeBase10();
        gpsLongitude = gps.getLongitudeBase10();
		gpsAltitude = gps.getAltitude();
        // Количество видимых спутников
        stlCount = gps.getSat();
        // Скорость
        gpsSpeed = gps.getSpeedKm();
        // Дата и время
        gpsDatetime.hour = gps.getHour();
        gpsDatetime.minute = gps.getMinute();
        gpsDatetime.second = gps.getSecond();
        gpsDatetime.day = gps.getDay();
        gpsDatetime.month = gps.getMonth();
        gpsDatetime.year = gps.getYear();
        break;
      case GPS_ERROR_DATA:
        //Serial.println("GPS error data");
        break;
      case GPS_ERROR_SAT:
        //Serial.println("GPS no connect to satellites!!!");
        break;
    }
  }
}

// Запись данных на карту
void sdWriteData() {}

// Настройка модулей
void setupNrf() {
  if (!nrf24.begin()) {
     Serial.println(F("radio hardware is not responding!"));
     while (1) {} // hold in infinite loop
   }

  nrf24.setAutoAck(NRF_AUTO_ACK);         //режим подтверждения приёма, 1 вкл 0 выкл
  nrf24.enableAckPayload();    //разрешить отсылку данных в ответ на входящий сигнал
  nrf24.setRetries(NRF_RETRIES_DELAY, NRF_RETRIES_COUNT);    //(время между попыткой достучаться, число попыток)

  nrf24.openWritingPipe(NRF_TX_ADDRESS);   //мы - труба 0, открываем канал для передачи данных
  nrf24.openReadingPipe(1, NRF_RX_ADDRESS);
  nrf24.setChannel(0x60);  //выбираем канал (в котором нет шумов!)

  nrf24.setPALevel (RF24_PA_MIN); //уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  nrf24.setDataRate (RF24_250KBPS); //скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  //должна быть одинакова на приёмнике и передатчике!
  //при самой низкой скорости имеем самую высокую чувствительность и дальность!!

  nrf24.setPayloadSize(NRF_RX_PACKET_SIZE);     //размер пакета, в байтах
  nrf24.powerUp(); //начать работу
  nrf24.startListening();
}
void setupIMU() {
    barometer.begin();
    compass.begin();
    gyroscope.begin();
    accelerometer.begin();

    compass.setRange(CompassRange::RANGE_8GAUSS);
    gyroscope.setRange(GyroscopeRange::RANGE_250DPS);
    accelerometer.setRange(AccelerometerRange::RANGE_8G);
}
void readPhtCalibRange() {
    Wire.beginTransmission(I2C_BUSOS);
    Wire.write(IC2_CMD_GET_PTH_COEF);
    Wire.endTransmission(false);

    Wire.requestFrom(I2C_BUSOS, 64);
    uint8_t data[4];
    for (uint8_t i = 0; i < 8; ++i) {
        for (uint8_t q = 0; q < 4; ++q) { data[q] = Wire.read(); }
        memcpy(&phtCalibRange[i].min, data, 4);
        for (uint8_t q = 0; q < 4; ++q) { data[q] = Wire.read(); }
        memcpy(&phtCalibRange[i].max, data, 4);
    }
}

// Функции преобразования
float readFloatI2C(int32_t minValue, int32_t maxValue, int32_t resolution, uint8_t count) {
    uint32_t value = 0;
    for (uint8_t i = 0; i < count; ++i) {
        value = Wire.read()<<8*i | value;
    }
    return static_cast<float>(value) / resolution * (maxValue - minValue) + minValue;
}
uint32_t floatToUint32(float value, int32_t minValue, int32_t maxValue, int32_t resolution) {
    if (minValue <= value && value <= maxValue) {
        return (value - minValue) * resolution/ (maxValue - minValue);
    }
    else { return 0; }
}
uint32_t dateToUint32(Datetime date) {
    uint32_t value;
    uint8_t *data = reinterpret_cast<uint8_t*>(&value);
    data[0] = date.year&0x3F;
    data[0] |= (date.month<<6)&0xC0;
    data[1] = (date.month>>2)&0x03;
    data[1] |= (date.day<<2)&0x7C;
    data[1] |= (date.hour<<7)&0x80;
    data[2] = (date.hour>>1)&0x0F;
    data[2] |= (date.minute<<4)&0xF0;
    data[3] = (date.minute>>4)&0x03;
    data[3] |= (date.second<<2)&0xFC;
    return value;
}
uint16_t calcCRC16(uint16_t *data, uint8_t count) {
    uint16_t CRC = 0;
    for (uint8_t i = 0; i < count; ++i) {
        CRC = CRC + *data*44111;  //все данные 16-битные
        CRC = CRC ^ (CRC >> 8);
    }
    return CRC;
}

// Отправка данных на землю
bool sendImuData() {
    uint8_t data[32];
    uint32_t value;

    data[0] = 0x1F;
    data[1] = 0xFF;
    data[2] = 0xFF;

    value = floatToUint32(press, 26000, 126000, 16777215);
    memcpy(data+3+0, &value, 3);

    value = floatToUint32(temp, -30, 110, 4095);
    memcpy(data+3+3, &value, 2);

    value = floatToUint32(gyro.x, -250000, 250000, 65535);
    memcpy(data+3+5, &value, 2);
    value = floatToUint32(gyro.y, -250000, 250000, 65535);
    memcpy(data+3+7, &value, 2);
    value = floatToUint32(gyro.z, -250000, 250000, 65535);
    memcpy(data+3+9, &value, 2);

    value = floatToUint32(acl.x, -8000, 8000, 65535);
    memcpy(data+3+11, &value, 2);
    value = floatToUint32(acl.y, -8000, 8000, 65535);
    memcpy(data+3+13, &value, 2);
    value = floatToUint32(acl.z, -8000, 8000, 65535);
    memcpy(data+3+15, &value, 2);

    value = floatToUint32(mgn.x, -16000, 16000, 65535);
    memcpy(data+3+17, &value, 2);
    value = floatToUint32(mgn.y, -16000, 16000, 65535);
    memcpy(data+3+19, &value, 2);
    value = floatToUint32(mgn.z, -16000, 16000, 65535);
    memcpy(data+3+21, &value, 2);

    value = millis();
    memcpy(data+3+23, &value, 4);

    uint16_t CRC = calcCRC16(reinterpret_cast<uint16_t*>(data), 15);
    memcpy(data+30, &CRC, 2);

    return nrf24SendData(data);
}
bool sendGpsData() {
    uint8_t data[32];
    uint32_t value;

    data[0] = 0x24;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[25] = 0xFF;

    memcpy(data+3+0, &gpsLatitude, 4);
    memcpy(data+3+4, &gpsLongitude, 4);
    memcpy(data+3+8, &gpsAltitude, 4);
    memcpy(data+3+12, &azimut, 4);
    memcpy(data+3+16, &gpsSpeed, 2);

    value = dateToUint32(gpsDatetime);
    memcpy(data+3+18, &value, 4);

    value = millis();
    memcpy(data+3+23, &value, 4);

    uint16_t CRC = calcCRC16(reinterpret_cast<uint16_t*>(data), 15);
    memcpy(data+30, &CRC, 2);

    return nrf24SendData(data);
}
bool sendCalibCoef() {
    uint8_t data[32];

    data[0] = 0x46;
    data[1] = 0xFF;
    data[2] = 0x01;
    data[27] = 0xFF;
    data[28] = 0xFF;
    data[29] = 0xFF;
    memcpy(data+3, &phtCalibRange[0].min, 4);
    memcpy(data+7, &phtCalibRange[0].max, 4);
    memcpy(data+11, &phtCalibRange[1].min, 4);
    memcpy(data+15, &phtCalibRange[1].max, 4);
    memcpy(data+19, &phtCalibRange[2].min, 4);
    memcpy(data+23, &phtCalibRange[2].max, 4);

    uint16_t CRC = calcCRC16(reinterpret_cast<uint16_t*>(data), 15);
    memcpy(data+30, &CRC, 2);
    nrf24SendData(data);


    data[2] = 0x02;
    memcpy(data+3, &phtCalibRange[3].min, 4);
    memcpy(data+7, &phtCalibRange[3].max, 4);
    memcpy(data+11, &phtCalibRange[4].min, 4);
    memcpy(data+15, &phtCalibRange[4].max, 4);
    memcpy(data+19, &phtCalibRange[5].min, 4);
    memcpy(data+23, &phtCalibRange[5].max, 4);

    CRC = calcCRC16(reinterpret_cast<uint16_t*>(data), 15);
    memcpy(data+30, &CRC, 2);
    nrf24SendData(data);


    data[2] = 0x03;
    memcpy(data+3, &phtCalibRange[6].min, 4);
    memcpy(data+7, &phtCalibRange[6].max, 4);
    memcpy(data+11, &phtCalibRange[7].min, 4);
    memcpy(data+15, &phtCalibRange[7].max, 4);
    data[19] = 0xFF;
    data[20] = 0xFF;
    data[21] = 0xFF;
    data[22] = 0xFF;
    data[23] = 0xFF;
    data[24] = 0xFF;
    data[25] = 0xFF;
    data[26] = 0xFF;

    CRC = calcCRC16(reinterpret_cast<uint16_t*>(data), 15);
    memcpy(data+30, &CRC, 2);

    return nrf24SendData(data);
}
bool sendPhtData() {
    uint8_t data[32];

    data[0] = 0x2B;
    data[1] = 0xFF;
    data[2] = 0xFF;
    for (uint8_t i = 19; i < 26; ++i) { data[i] = 0xFF; }

    memcpy(data+3, phtValues, 16);

    uint32_t value = millis();
    memcpy(data+26, &value, 4);

    uint16_t CRC = calcCRC16(reinterpret_cast<uint16_t*>(data), 15);
    memcpy(data+30, &CRC, 2);

    return nrf24SendData(data);
}
bool nrf24SendData(uint8_t *data) {
    nrf24.setPayloadSize(NRF_TX_PACKET_SIZE);
	nrf24.stopListening();
	uint8_t ans = nrf24.write(data, NRF_TX_PACKET_SIZE);

	nrf24.setPayloadSize(NRF_RX_PACKET_SIZE);
	nrf24.startListening();

	return ans;
}

// Запрос по Serial
void serialRequest() {
	while(Serial.available()) {
		if (Serial.read() == 'K') {
			int32_t request = Serial.parseInt(SKIP_NONE);
			if (!request) { serialRequestInvalid(); continue; }

			Serial.print('K');
			Serial.print(request);
			Serial.print('\n');

			switch (request) {
				case 1:  serialRequest_1();  break;
				case 30: serialRequest_30(); break;
				case 31: serialRequest_31(); break;
				case 35: serialRequest_35(); break;
				case 36: serialRequest_36(); break;
				case 42: serialRequest_42(); break;
				case 43: serialRequest_43(); break;
				case 70: serialRequest_70(); break;
				default: serialRequestIndefined(request);
			}

			while(Serial.available() && Serial.read() != '\n') {}
		}
		else { serialRequestInvalid(); }
	}
}
void serialRequest_1() {
	Serial.print(F("OK\n"));
	while(Serial.available() && Serial.read() != '\n') {}

	uint8_t data[32];
	data[0] = 0x01;

	uint16_t CRC = calcCRC16(reinterpret_cast<uint16_t*>(data), 15);
    memcpy(data+30, &CRC, 2);

    nrf24SendData(data);
}
void serialRequest_30() {
	Serial.print(F("Напряжение: "));
	Serial.print(mainVoltage);
	Serial.print(F(" %\nНапряжение батареи: "));
	Serial.print(batteryVoltage);
	Serial.print(F(" %\nНапряжение солнечных понелей: "));
	Serial.print(solarVoltage);
	Serial.print(F(" %\nOK\n"));
}
void serialRequest_31() {
	Serial.print(F("Давление: "));
    Serial.print(press);
    Serial.print(F(" Па\nТемпература: "));
    Serial.print(temp);
    Serial.print(F(" °C\nВектор ускорения (X) (Y) (Z): "));
    Serial.print(acl.x); Serial.print(' '); Serial.print(acl.y); Serial.print(' '); Serial.print(acl.z);
    Serial.print(F(" м\\с²\nВектор магнитного поля (X) (Y) (Z): "));
    Serial.print(mgn.x); Serial.print(' '); Serial.print(mgn.y); Serial.print(' '); Serial.print(mgn.z);
    Serial.print(F(" мГаус\nВектор угловой скорости (X) (Y) (Z): "));
    Serial.print(gyro.x); Serial.print(' '); Serial.print(gyro.y); Serial.print(' '); Serial.print(gyro.z);
    Serial.print(F(" °\\с\n"));
    printMillisTime(millis());
    Serial.print(F("OK\n"));
}
void serialRequest_35() {
	Serial.print(F("Угол Эйлера (X) (Y) (Z): "));
	Serial.print(rotateAngle.x); Serial.print(' '); Serial.print(rotateAngle.y); Serial.print(' '); Serial.print(rotateAngle.z);
	Serial.print(F(" °\nOK\n"));
}
void serialRequest_36() {
	Serial.print(F("Широта: "));
    Serial.print(gpsLatitude);
    Serial.print(F(" °\nДолгота: "));
    Serial.print(gpsLongitude);
    Serial.print(F(" °\nВысота: "));
    Serial.print(gpsAltitude);
    Serial.print(F(" м\nСкорость: "));
    Serial.print(gpsSpeed);
    Serial.print(F(" м\\с\nВремя (чч:мм с): "));

    if (gpsDatetime.hour < 10) { Serial.print('0'); }
    Serial.print(gpsDatetime.hour);
    Serial.print(':');

    if (gpsDatetime.minute < 10) { Serial.print('0'); }
    Serial.print(gpsDatetime.minute);
    Serial.print(' ');
    Serial.print(gpsDatetime.second);

    Serial.print(F("\nДата: "));

    if (gpsDatetime.day < 10) { Serial.print('0'); }
    Serial.print(gpsDatetime.day);
    Serial.print('.');

    if (gpsDatetime.month < 10) { Serial.print('0'); }
    Serial.print(gpsDatetime.month);
    Serial.print('.');

    Serial.println(gpsDatetime.year + 2000);
    printMillisTime(millis());
    Serial.print(F("OK\n"));
}
void serialRequest_42() {
    for (uint8_t i = 0; i < 8; ++i) {
        Serial.print(F("Значение фоторезистора ("));
        Serial.print(i + 1);
        Serial.print(F("): "));
        Serial.println(map(phtValues[i], 0, 1023, phtCalibRange[i].min, phtCalibRange[i].max));
    }
	Serial.print(F("OK\n"));
}
void serialRequest_43() {
    for (uint8_t i = 0; i < 8; ++i) {
        Serial.print(F("Значение АЦП фоторезистора ("));
        Serial.print(i + 1);
        Serial.print(F("): "));
        Serial.println(phtValues[i]);
    }
	Serial.print(F("OK\n"));
}
void serialRequest_70() {
    for (uint8_t i = 0; i < 8; ++i) {
        Serial.print(F("Диапазон измерений фоторезистора ("));
        Serial.print(i + 1);
        Serial.print(F("): "));
        Serial.println(phtCalibRange[i].min);
        Serial.print(' ');
        Serial.println(phtCalibRange[i].max);
    }
	Serial.print(F("OK\n"));
}
void printMillisTime(uint32_t time) {
	Serial.print(F("Время с момента старта МК: "));

	uint8_t day = time/1000/60/60/24;
	if (day) {
		Serial.print(day);
		if (day % 10 == 1) { Serial.print(F(" день ")); }
		else if (2 <= day % 10 && day % 10 <= 4) { Serial.print(F(" дня ")); }
		else { Serial.print(F(" дней ")); }
	}

	uint8_t hour = (time/1000/60/60) % 24;
	if (hour) {
		Serial.print(hour);
		if (hour % 10 == 1) { Serial.print(F(" час ")); }
		else if (2 <= hour % 10 && hour % 10 <= 4) { Serial.print(F(" часа ")); }
		else { Serial.print(F(" часов ")); }
	}

	uint8_t minute = (time/1000/60) % 60;
	if (minute) {
		Serial.print(minute);
		if (minute % 10 == 1) { Serial.print(F(" минута ")); }
		else if (2 <= minute % 10 && minute % 10 <= 4) { Serial.print(F(" минуты ")); }
		else { Serial.print(F(" минут ")); }
	}

	uint8_t second = (time/1000) % 60;
	Serial.print(second);
	if (second % 10 == 1) { Serial.print(F(" секунда\n")); }
	else if (2 <= second % 10 && second % 10 <= 4) { Serial.print(F(" секунды\n")); }
	else { Serial.print(F(" секунд\n")); }
}
void serialRequestInvalid() {
	Serial.print(F("Команда не разпознана...\n"));
}
void serialRequestIndefined(int32_t request) {
	Serial.print(F("Команда \"")); Serial.print(request); Serial.print(F("\" недействительна...\n"));
}

// Запрос с земли
void nrf24Request() {
    uint8_t data[NRF_RX_PACKET_SIZE];
    nrf24.read(data, NRF_RX_PACKET_SIZE);
    switch(data[0]) {
    case 1: Serial.print("nRF24L01 is receive data!\n"); break;
    case 70: sendCalibCoef(); break;
    };
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    // Скорость менять с 115200 не нужно
    gpsSerial.begin(9600);
    // ВАЖНО: РАДИОМОДУЛЬ НЕ БУДЕТ РАБОТАТЬ, БЕЗ SD КАРТЫ!
    // С недочётом сделана платы, проблема физическая
    setupNrf();

    Serial.println(F("BC board is ready...\n"));

    uint32_t gpsUpdateTimeMark = 0;
    uint32_t imuUpdateTimeMark = 0;
    uint32_t akbUpdateTimeMark = 0;
    uint32_t phtUpdateTimeMark = 0;

    uint32_t imuSendTimeMark = 0;
    uint32_t gpsSendTimeMark = 0;
    uint32_t phtSendTimeMark = 0;

    uint32_t sdWriteTimeMark = 0;

    while(true) {
	    if (gpsUpdateTimeMark < millis() && gpsUpdateInterval >= MIN_INTERVAL_VALUE) {
		    updateGPSData();
		    gpsUpdateTimeMark = millis() + gpsUpdateInterval;
	    }
	    if (imuUpdateTimeMark < millis() && imuUpdateInterval >= MIN_INTERVAL_VALUE) {
		    updateIMUData();
		    imuUpdateTimeMark = millis() + imuUpdateInterval;
	    }
	    if (akbUpdateTimeMark < millis() && akbUpdateInterval >= MIN_INTERVAL_VALUE) {
		    updateAkbData();
		    akbUpdateTimeMark = millis() + akbUpdateInterval;
	    }
	    if (phtUpdateTimeMark < millis() && phtUpdateInterval >= MIN_INTERVAL_VALUE) {
		    updatePhtData();
		    phtUpdateTimeMark = millis() + phtUpdateInterval;
	    }
	    if (gpsSendTimeMark < millis() && gpsSendInterval >= MIN_INTERVAL_VALUE) {
		    sendGpsData();
		    gpsSendTimeMark = millis() + gpsSendInterval;
	    }
	    if (imuSendTimeMark < millis() && imuSendInterval >= MIN_INTERVAL_VALUE) {
		    sendImuData();
		    imuSendTimeMark = millis() + imuSendInterval;
	    }
	    if (phtSendTimeMark < millis() && phtSendInterval >= MIN_INTERVAL_VALUE) {
		    sendPhtData();
		    phtSendTimeMark = millis() + phtSendInterval;
	    }
	    if (sdWriteTimeMark < millis() && sdWriteInterval >= MIN_INTERVAL_VALUE) {
            // Чтобы сразу инициализировать карту, при её подключении
            if (FLAGS&SD_CARD_INITIALIZATRED) {
                sdWriteData();
            }
		    else if (SD.begin(PIN_CHIP_SELECT)) {
		        FLAGS |= SD_CARD_INITIALIZATRED;
                sdWriteData();
                sdWriteTimeMark = millis() + sdWriteInterval;
		    }
            else { sdWriteTimeMark = millis() + sdWriteInterval/2; }
	    }
	    if (!(FLAGS&FLG_CALIB_RNG_READED)) {
            readPhtCalibRange();
            FLAGS |= FLG_CALIB_RNG_READED;
	    }
	    if (Serial.available()) {
		    serialRequest();
	    }
	    if (nrf24.available()) {
            nrf24Request();
	    }
  }
}

void loop() {}
