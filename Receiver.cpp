#include <GOST4401_81.h>
#include <SoftwareSerial.h>

#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>

#define NRF_RX_PSIZE 8
#define NRF_TX_PSIZE 8
#define NRF_RETRIES_DELAY 0
#define NRF_RETRIES_COUNT 3
#define NRF_AUTO_ACK 1

#define RX_PACKET_SIZE 32
#define TX_PACKET_SIZE 20

/* Интервалы, специально сделаны переменными, чтобы
/* менять во время выполнения программы, но пока не реализовано */
#define MIN_INTERVAL_VALUE 10
uint16_t calibCoefRequestInterval = 1000;

#define SERIAL_SEP ';'

// Флаги
#define FLG_HUMAN_UI          0x01
#define FLG_PRINT_DATA_ALWAYS 0x02
#define FLG_PHT_READED_P_1    0x04
#define FLG_PHT_READED_P_2    0x08
#define FLG_PHT_READED_P_3    0x10
uint8_t FLAGS = 0;


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


RF24 nrf24(10, 9);

uint64_t addressTX = 0xAAE10CF1F1;
uint64_t addressRX = 0xAAE10CF1F0;
uint8_t stlCount = 0, btr_1 = 0, btr_2 = 0;
Datetime gpsDatetime;
float gpsLatitude = 0,  gpsLongitude = 0, gpsAltitude = 0, azimut = 0, altitude = 0;
float press = 0, temp = 0;
Vector gyro, acl, mgn;
Euler rotateAngle;
uint16_t gpsSpeed = 0, phtValues[8] = {};
Range phtCalibRange[8];
uint32_t lastImuMillis = 0, lastGpsMillis = 0, lastPhtMillis = 0;

void setupNrf() {
  if (!nrf24.begin()) {
     Serial.println(F("radio hardware is not responding!"));
     while (1) {} // hold in infinite loop
   }

  nrf24.setAutoAck(NRF_AUTO_ACK);         //режим подтверждения приёма, 1 вкл 0 выкл
  nrf24.enableAckPayload();    //разрешить отсылку данных в ответ на входящий сигнал
  nrf24.setRetries(NRF_RETRIES_DELAY, NRF_RETRIES_COUNT);    //(время между попыткой достучаться, число попыток)

  nrf24.openWritingPipe(addressTX);   //мы - труба 0, открываем канал для передачи данных
  nrf24.openReadingPipe(1, addressRX);
  nrf24.setChannel(0x60);  //выбираем канал (в котором нет шумов!)

  nrf24.setPALevel (RF24_PA_MIN); //уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  nrf24.setDataRate (RF24_250KBPS); //скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  //должна быть одинакова на приёмнике и передатчике!
  //при самой низкой скорости имеем самую высокую чувствительность и дальность!!

  nrf24.setPayloadSize(RX_PACKET_SIZE);     //размер пакета, в байтах
  nrf24.powerUp(); //начать работу
  nrf24.startListening();
}
bool nrf24SendData(const uint8_t *data) {
    nrf24.setPayloadSize(TX_PACKET_SIZE);
	nrf24.stopListening();  //не слушаем радиоэфир, мы передатчик
	uint8_t ans = nrf24.write(data, TX_PACKET_SIZE);

	nrf24.setPayloadSize(RX_PACKET_SIZE);
	nrf24.startListening();

	return ans;
}

Datetime dateFromUint32(uint32_t value) {
    Datetime datetime;
    uint8_t *data = reinterpret_cast<uint8_t*>(&value);

    datetime.year = data[0]&0x3F;
    datetime.month = ((data[1]<<2)&0x0C) | (data[0]>>6);
    datetime.day = (data[1]>>2)&0x1F;
    datetime.hour = ((data[2]<<1)&0x1E) | (data[1]>>7);
    datetime.minute = ((data[3]<<4)&0x30) | (data[2]>>4);
    datetime.second = (data[3]>>2);

    return datetime;
}
uint32_t floatToUint32(float value, int32_t minValue, int32_t maxValue, int32_t resolution) {
    if (minValue <= value && value <= maxValue) {
        return (value - minValue) * resolution/ (maxValue - minValue);
    }
    else { return 0; }
}
float parseFloat(const uint8_t *data, int32_t minValue, int32_t maxValue, int32_t resolution, uint8_t count) {
    uint32_t value = 0;
    memcpy(&value, data, count);
    return static_cast<float>(value) / resolution * (maxValue - minValue) + minValue;
}
uint16_t calcCRC16(const uint16_t *data, uint8_t count) {
    uint16_t CRC = 0;
    for (uint8_t i = 0; i < count; ++i) {
        CRC = CRC + *data*44111;  //все данные 16-битные
        CRC = CRC ^ (CRC >> 8);
    }
    return CRC;
}

bool readImuData(const uint8_t *data) {
    uint16_t cCRC = calcCRC16(reinterpret_cast<const uint16_t*>(data), 15);
    uint16_t rCRC = (data[31]<<8) | data[30];
    if (cCRC != rCRC) { return false; }

    if (data[0] != 0x1F || data[1] != 0xFF || data[2] != 0xFF) { return false; }

    press = parseFloat(data+3, 26000, 126000, 16777215, 3);
    temp = parseFloat(data+6, -30, 110, 4095, 2);

    gyro.x = parseFloat(data+8, -250000, 250000, 65535, 2);
    gyro.y = parseFloat(data+10, -250000, 250000, 65535, 2);
    gyro.z = parseFloat(data+12, -250000, 250000, 65535, 2);

    acl.x = parseFloat(data+14, -8000, 8000, 65535, 2);
    acl.y = parseFloat(data+16, -8000, 8000, 65535, 2);
    acl.z = parseFloat(data+18, -8000, 8000, 65535, 2);

    mgn.x = parseFloat(data+20, -16000, 16000, 65535, 2);
    mgn.y = parseFloat(data+22, -16000, 16000, 65535, 2);
    mgn.z = parseFloat(data+24, -16000, 16000, 65535, 2);

    memcpy(&lastImuMillis, data+26, 4);

    if (FLAGS&FLG_PRINT_DATA_ALWAYS) { printImuData(); }
    return true;
}
bool readGpsData(const uint8_t *data) {
    uint16_t cCRC = calcCRC16(reinterpret_cast<const uint16_t*>(data), 15);
    uint16_t rCRC = (data[31]<<8) | data[30];
    if (cCRC != rCRC) { return false; }

    if (data[0] != 0x24 || data[1] != 0xFF || data[2] != 0xFF || data[25] != 0xFF) {
        return false;
    }

    memcpy(&gpsLatitude,  data+3, 4);
    memcpy(&gpsLongitude, data+7, 4);
    memcpy(&gpsAltitude,  data+11, 4);
    memcpy(&azimut,       data+15, 4);
    memcpy(&gpsSpeed,     data+19, 2);

    uint32_t value;
    memcpy(&value, data+21, 4);
    gpsDatetime = dateFromUint32(value);

    memcpy(&lastGpsMillis, data+26, 4);

    if (FLAGS&FLG_PRINT_DATA_ALWAYS) { printGpsData(); }
    return true;
}
bool readCalibCoef(const uint8_t *data) {
    uint16_t cCRC = calcCRC16(reinterpret_cast<const uint16_t*>(data), 15);
    uint16_t rCRC = (data[31]<<8) | data[30];
    if (cCRC != rCRC) { return false; }

    if (data[0] != 0x46 || data[1] != 0xFF || data[27] != 0xFF ||
        data[28] != 0xFF || data[29] != 0xFF) {
        return false;
    }

    switch(data[2]) {
    case 0x01: {
        memcpy(&phtCalibRange[0].min, data+3, 4);
        memcpy(&phtCalibRange[0].max, data+7, 4);
        memcpy(&phtCalibRange[1].min, data+11, 4);
        memcpy(&phtCalibRange[1].max, data+15, 4);
        memcpy(&phtCalibRange[2].min, data+19, 4);
        memcpy(&phtCalibRange[2].max, data+23, 4);
        FLAGS |= FLG_PHT_READED_P_1;
        break;
    }
    case 0x02: {
        memcpy(&phtCalibRange[3].min, data+3, 4);
        memcpy(&phtCalibRange[3].max, data+7, 4);
        memcpy(&phtCalibRange[4].min, data+11, 4);
        memcpy(&phtCalibRange[4].max, data+15, 4);
        memcpy(&phtCalibRange[5].min, data+19, 4);
        memcpy(&phtCalibRange[5].max, data+23, 4);
        FLAGS |= FLG_PHT_READED_P_2;
        break;
    }
    case 0x03: {
        for (uint8_t i = 19; i < 26; ++i) {
            if (data[i] != 0xFF) { return false; }
        }
        memcpy(&phtCalibRange[6].min, data+3, 4);
        memcpy(&phtCalibRange[6].max, data+7, 4);
        memcpy(&phtCalibRange[7].min, data+11, 4);
        memcpy(&phtCalibRange[7].max, data+15, 4);
        FLAGS |= FLG_PHT_READED_P_3;
        break;
    }};
    return true;
}
bool readPthData(const uint8_t *data) {
    uint16_t cCRC = calcCRC16(reinterpret_cast<const uint16_t*>(data), 15);
    uint16_t rCRC = (data[31]<<8) | data[30];
    if (cCRC != rCRC) { return false; }

    if (data[0] != 0x2B || data[1] != 0xFF || data[2] != 0xFF) {
        return false;
    }
    for (uint8_t i = 19; i < 26; ++i) {
        if (data[i] != 0xFF) { return false; }
    }

    memcpy(phtValues, data+3, 16);
    memcpy(&lastPhtMillis, data+26, 4);

    if (FLAGS&FLG_PRINT_DATA_ALWAYS) { printGpsData(); }
    return true;
}

void printImuData() {
    if (FLAGS&FLG_HUMAN_UI) {
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
        printMillisTime(lastImuMillis);
    }
    else {
        Serial.print(press);
        Serial.print(SERIAL_SEP);
        Serial.print(temp);
        Serial.print(SERIAL_SEP);
        Serial.print(acl.x);
        Serial.print(SERIAL_SEP);
        Serial.print(acl.y);
        Serial.print(SERIAL_SEP);
        Serial.print(acl.z);
        Serial.print(SERIAL_SEP);
        Serial.print(mgn.x);
        Serial.print(SERIAL_SEP);
        Serial.print(mgn.y);
        Serial.print(SERIAL_SEP);
        Serial.print(mgn.z);
        Serial.print(SERIAL_SEP);
        Serial.print(gyro.x);
        Serial.print(SERIAL_SEP);
        Serial.print(gyro.y);
        Serial.print(SERIAL_SEP);
        Serial.print(gyro.z);
        Serial.print(SERIAL_SEP);
        Serial.print(lastImuMillis);
        Serial.print('\n');
    }
}
void printGpsData() {
    if (FLAGS&FLG_HUMAN_UI) {
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
        printMillisTime(lastGpsMillis);
    }
    else {
        Serial.print(gpsLatitude);
        Serial.print(SERIAL_SEP);
        Serial.print(gpsLongitude);
        Serial.print(SERIAL_SEP);
        Serial.print(gpsAltitude);
        Serial.print(SERIAL_SEP);
        Serial.print(gpsSpeed);
        Serial.print(SERIAL_SEP);
        Serial.print(gpsDatetime.hour);
        Serial.print(SERIAL_SEP);
        Serial.print(gpsDatetime.minute);
        Serial.print(SERIAL_SEP);
        Serial.print(gpsDatetime.second);
        Serial.print(SERIAL_SEP);
        Serial.print(gpsDatetime.day);
        Serial.print(SERIAL_SEP);
        Serial.print(gpsDatetime.month);
        Serial.print(SERIAL_SEP);
        Serial.print(gpsDatetime.year);
        Serial.print(SERIAL_SEP);
        Serial.print(lastGpsMillis);
        Serial.print('\n');
    }
}
void printPhtData() {
    if (FLAGS&FLG_HUMAN_UI) {
        for (uint8_t i = 0; i < 8; ++i) {
            Serial.print(F("Значение фоторезистора ("));
            Serial.print(i + 1);
            Serial.print(F("): "));
            Serial.println(map(phtValues[i], 0, 1023, phtCalibRange[i].min, phtCalibRange[i].max));
        }
        printMillisTime(lastPhtMillis);
    }
    else {
        for (uint8_t i = 0; i < 8; ++i) {
            Serial.print(phtValues[i]);
            Serial.print(SERIAL_SEP);
        }
        Serial.print(lastImuMillis);
        Serial.print('\n');
    }
}

void requestUpdatePthCR() {
    uint8_t data[TX_PACKET_SIZE];
    for (uint8_t i = 0; i < TX_PACKET_SIZE; ++i) { data[i] = 0xFF; }
    data[0] = 70;

    nrf24SendData(data);
}

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
	Serial.print(F("Заряд 1 аккамулятора: "));
	Serial.print(btr_1*0.5);
	Serial.print(F(" %\nЗаряд 2 аккамулятора: "));
	Serial.print(btr_2*0.5);
	Serial.print(F(" %\nOK\n"));
	while(Serial.available() && Serial.read() != '\n') {}
}
void serialRequest_31() {
	printImuData();
    Serial.print(F("OK\n"));
	while(Serial.available() && Serial.read() != '\n') {}
}
void serialRequest_35() {
	Serial.print(F("Угол Эйлера (X) (Y) (Z): "));
	Serial.print(rotateAngle.x); Serial.print(' '); Serial.print(rotateAngle.y); Serial.print(' '); Serial.print(rotateAngle.z);
	Serial.print(F(" °\nOK\n"));
	while(Serial.available() && Serial.read() != '\n') {}
}
void serialRequest_36() {
	printGpsData();
    Serial.print(F("OK\n"));
	while(Serial.available() && Serial.read() != '\n') {}
}
void serialRequest_42() {
    printPhtData();
	Serial.print(F("OK\n"));
	while(Serial.available() && Serial.read() != '\n') {}
}
void serialRequest_43() {
    for (uint8_t i = 0; i < 8; ++i) {
        Serial.print(F("Значение АЦП фоторезистора ("));
        Serial.print(i + 1);
        Serial.print(F("): "));
        Serial.println(phtValues[i]);
    }
	Serial.print(F("OK\n"));
	while(Serial.available() && Serial.read() != '\n') {}
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
	while(Serial.available() && Serial.read() != '\n') {}
}
void printMillisTime(uint32_t time) {
	Serial.print(F("Время: "));

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
	while(Serial.available() && Serial.read() != '\n') {}
}
void serialRequestIndefined(int32_t request) {
	Serial.print(F("Команда \"")); Serial.print(request); Serial.print(F("\" недействительна...\n"));
	while(Serial.available() && Serial.read() != '\n') {}
}

void Nrf24Ok(uint8_t *data) {
    Serial.print("nRF24L01 is OK\n");
}

void nrf24Request() {
    uint8_t data[RX_PACKET_SIZE];
    nrf24.read(data, RX_PACKET_SIZE);
    switch(data[0]) {
    case 1: Nrf24Ok(data); break;
    case 31: readImuData(data); break;
    case 36: readGpsData(data); break;
    case 43: readPthData(data); break;
    case 70: readCalibCoef(data); break;
    };
}

void setup() {
    Serial.begin(115200);
    Serial.print(F("Receiver is OK\n"));
    setupNrf();

    uint32_t phtCRReqTimeMark = 0;

    while(true) {
        if (phtCRReqTimeMark < millis() && calibCoefRequestInterval > MIN_INTERVAL_VALUE
             && (!(FLAGS&FLG_PHT_READED_P_1) || !(FLAGS&FLG_PHT_READED_P_2) || !(FLAGS&FLG_PHT_READED_P_3))) {
            requestUpdatePthCR();
            phtCRReqTimeMark = millis() + calibCoefRequestInterval;
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

