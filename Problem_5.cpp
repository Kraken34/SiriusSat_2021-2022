/* Задача №5
*  Это фрагмент кода из BC.cpp, отвечающий за выполнение задачи по программирования №5
*  Файл BC.cpp находится в этом же каталоге на GitHub
*/
#include <SoftwareSerial.h>
#include <SPI.h>

/* ВАЖНО: Небходимо перед использованием <TroykaGPS.h> заменить файл TroykaGPS.cpp,
*  который находится в папке с библиотекой TroykaGPS (обычно: \Documents\Arduino\libraries\TroykaGPS-master\src), 
*  файлом TroykaGPS.cpp, который находится на томже GitHub репозитории, где разположен сейчас BC.cpp и Problem_5.cpp */
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

#include <SD.h>
#define PIN_CHIP_SELECT 8

// Мнимально допустимый интревал
#define MIN_INTERVAL_VALUE 10
#define gpsUpdateInterval 1000
#define gpsSendInterval   1000
#define imuSendInterval   1000
#define phtSendInterval   1000
#define sdWriteInterval   1000

// Флаги
#define SD_CARD_INITIALIZATRED 0x04
#define GPS_READY              0x08
uint8_t FLAGS = 0;

SoftwareSerial gpsSerial(RX_GPS_PIN, TX_GPS_PIN);
File           logfile;
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
    float min = 0;
    float max = 1000;
};
struct Datetime {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
};

// Количество спутников
uint8_t stlCount = 0;
// Дата и время с GPS
Datetime gpsDatetime;
// Координаты
float gpsLatitude = 35,  gpsLongitude = 56, gpsAltitude = 200, azimut = 90, altitude = 233;
// Скорость
int16_t gpsSpeed = 35;
// Напряжения
float mainVoltage = 0, batteryVoltage = 0, solarVoltage = 0;
// Остальное
float press = 96000, temp = 22.5;
uint16_t phtValues[8] = {300, 300, 500, 500, 700, 700, 1024, 1024};
Range phtCalibRange[8];
Vector gyro = {567, -456, 56.78}, acl = {400, 20, 4540}, mgn = {-2344, 455, 859};
Euler rotateAngle = {30, -60, 90};

void updateGPSData() {
  if (gps.available()) {
    gps.readParsing();
    // Проверка состояния GPS
    switch (gps.getState()) {
      case GPS_OK:
        // Координаты
        gpsLatitude = gps.getLatitudeBase10();
        gpsLongitude = gps.getLongitudeBase10();
		gpsAltitude = gps.getAltitude();
        // Количество видимых спутников
        stlCount = gps.getSat();
		if (stlCount > 2) { FLAGS |= GPS_READY; }
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
      case GPS_ERROR_DATA: break;
      case GPS_ERROR_SAT: break;
    }
  }
}


// Проверка позиции
bool isTargetPosition() {
    if (FLAGS&GPS_READY) {
        /*  Писать в этом блоке
         *  gpsLatitude  -  Широта
         *  gpsLongitude -  Долгота
         *  gpsAltitude  -  Высота   */
        return true;
    }
    return true;
}

// Настройка модуля радиосвязи
void setupNrf() {
  if (!nrf24.begin()) {
	 // ВАЖНО: РАДИОМОДУЛЬ НЕ БУДЕТ РАБОТАТЬ, БЕЗ SD КАРТЫ!
     // С недочётом сконструированы платы, проблема физическая
     Serial.println(F("Радиомодуль не подключён или отсутствует SD карта!"));
     while (1) {}
   }

  nrf24.setAutoAck(NRF_AUTO_ACK); // режим подтверждения приёма
  nrf24.enableAckPayload(); //разрешить отсылку данных в ответ на входящий сигнал
  nrf24.setRetries(NRF_RETRIES_DELAY, NRF_RETRIES_COUNT);

  nrf24.openWritingPipe(NRF_TX_ADDRESS);
  nrf24.openReadingPipe(1, NRF_RX_ADDRESS);
  nrf24.setChannel(0x60);  // выбираем канал

  nrf24.setPALevel (RF24_PA_MIN); //уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  nrf24.setDataRate (RF24_250KBPS); //скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS

  nrf24.setPayloadSize(NRF_RX_PACKET_SIZE);
  nrf24.powerUp();
  nrf24.startListening();
}

// Сжатие данных, для более быстрой передачи
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
// Вычисление контрольной суммы
uint16_t calcCRC16(uint16_t *data, uint8_t count) {
    uint16_t CRC = 0;
    for (uint8_t i = 0; i < count; ++i) {
        CRC = CRC + *data*44111;  //все данные 16-битные
        CRC = CRC ^ (CRC >> 8);
    }
    return CRC;
}

// Отправка данных по радиосвязи
bool sendImuData() {
    uint8_t data[32];
    uint32_t value;

	// Заголовок
    data[0] = 0x1F;
    data[1] = 0xFF;
    data[2] = 0xFF;

	// Сжатие данных
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

	// Контрольная сумма
    uint16_t CRC = calcCRC16(reinterpret_cast<uint16_t*>(data), 15);
    memcpy(data+30, &CRC, 2);

    return nrf24SendData(data);
}
bool sendGpsData() {
    uint8_t data[32];
    uint32_t value;

	// Заголовок
    data[0] = 0x24;
    data[1] = 0xFF;
    data[2] = 0xFF;
	// Неиспользуемые байты
    data[25] = 0xFF;

	// Сжатие данных
    memcpy(data+3+0, &gpsLatitude, 4);
    memcpy(data+3+4, &gpsLongitude, 4);
    memcpy(data+3+8, &gpsAltitude, 4);
    memcpy(data+3+12, &azimut, 4);
    memcpy(data+3+16, &gpsSpeed, 2);

    value = dateToUint32(gpsDatetime);
    memcpy(data+3+18, &value, 4);

    value = millis();
    memcpy(data+3+23, &value, 4);

	// Контрольная сумма
    uint16_t CRC = calcCRC16(reinterpret_cast<uint16_t*>(data), 15);
    memcpy(data+30, &CRC, 2);

    return nrf24SendData(data);
}
bool sendCalibCoef() {
    uint8_t data[32];

	// Заголовок
    data[0] = 0x46;
    data[1] = 0xFF;
    data[2] = 0x01;
	// Неиспользуемые байты
    data[27] = 0xFF;
    data[28] = 0xFF;
    data[29] = 0xFF;
	// Данные
    memcpy(data+3, &phtCalibRange[0].min, 4);
    memcpy(data+7, &phtCalibRange[0].max, 4);
    memcpy(data+11, &phtCalibRange[1].min, 4);
    memcpy(data+15, &phtCalibRange[1].max, 4);
    memcpy(data+19, &phtCalibRange[2].min, 4);
    memcpy(data+23, &phtCalibRange[2].max, 4);

	// Контрольная сумма
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

	// Контрольная сумма
    CRC = calcCRC16(reinterpret_cast<uint16_t*>(data), 15);
    memcpy(data+30, &CRC, 2);
    nrf24SendData(data);


    data[2] = 0x03;
    memcpy(data+3, &phtCalibRange[6].min, 4);
    memcpy(data+7, &phtCalibRange[6].max, 4);
    memcpy(data+11, &phtCalibRange[7].min, 4);
    memcpy(data+15, &phtCalibRange[7].max, 4);
	// Неиспользуемые байты
    data[19] = 0xFF;
    data[20] = 0xFF;
    data[21] = 0xFF;
    data[22] = 0xFF;
    data[23] = 0xFF;
    data[24] = 0xFF;
    data[25] = 0xFF;
    data[26] = 0xFF;
	
	// Контрольная сумма
    CRC = calcCRC16(reinterpret_cast<uint16_t*>(data), 15);
    memcpy(data+30, &CRC, 2);

    return nrf24SendData(data);
}
bool sendPhtData() {
    uint8_t data[32];

	// Заголовок
    data[0] = 0x2B;
    data[1] = 0xFF;
    data[2] = 0xFF;
	// Неиспользуемые байты
    for (uint8_t i = 19; i < 26; ++i) { data[i] = 0xFF; }

	// Данные
    memcpy(data+3, phtValues, 16);

	// Время
    uint32_t value = millis();
    memcpy(data+26, &value, 4);

	// Контрольная сумма
    uint16_t CRC = calcCRC16(reinterpret_cast<uint16_t*>(data), 15);
    memcpy(data+30, &CRC, 2);

    return nrf24SendData(data);
}
// Отправка готовых данных
bool nrf24SendData(uint8_t *data) {
    nrf24.setPayloadSize(NRF_TX_PACKET_SIZE);
	nrf24.stopListening();
	uint8_t ans = nrf24.write(data, NRF_TX_PACKET_SIZE);

	nrf24.setPayloadSize(NRF_RX_PACKET_SIZE);
	nrf24.startListening();

	return ans;
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

// Запись данных на карту
void sdWriteData() {
	logfile = SD.open("log.csv", FILE_WRITE);
	
	//dev|stlCount|mVolt|bVolt|sVolt|press|term|azimut|gyX|gyY|gyZ|aX|aY|aZ|mX|mY|mZ|eX|eY|eZ|pAlt|gAlt|gLat|gLon|gSpeed|year|month|day|hour|minute|second
	logfile.print(stlCount); logfile.print('|');
	logfile.print(mainVoltage); logfile.print('|');
	logfile.print(batteryVoltage); logfile.print('|');
	logfile.print(solarVoltage); logfile.print('|');
	logfile.print(press); logfile.print('|');
	logfile.print(temp); logfile.print('|');
	logfile.print(azimut); logfile.print('|');
	logfile.print(gyro.x); logfile.print('|');
	logfile.print(gyro.y); logfile.print('|');
	logfile.print(gyro.z); logfile.print('|');
	logfile.print(acl.x); logfile.print('|');
	logfile.print(acl.y); logfile.print('|');
	logfile.print(acl.z); logfile.print('|');
	logfile.print(mgn.x); logfile.print('|');
	logfile.print(mgn.y); logfile.print('|');
	logfile.print(mgn.z); logfile.print('|');
	logfile.print(rotateAngle.x); logfile.print('|');
	logfile.print(rotateAngle.y); logfile.print('|');
	logfile.print(rotateAngle.z); logfile.print('|');
	logfile.print(altitude); logfile.print('|');
	logfile.print(gpsAltitude); logfile.print('|');
	logfile.print(gpsLatitude); logfile.print('|');
	logfile.print(gpsLongitude); logfile.print('|');
	logfile.print(gpsSpeed); logfile.print('|');
	logfile.print(gpsDatetime.year); logfile.print('|');
	logfile.print(gpsDatetime.month); logfile.print('|');
	logfile.print(gpsDatetime.day); logfile.print('|');
	logfile.print(gpsDatetime.hour); logfile.print('|');
	logfile.print(gpsDatetime.minute); logfile.print('|');
	logfile.print(gpsDatetime.second); logfile.print('|');
	logfile.print(millis()); logfile.print('\n');
	
	logfile.close();
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
    // Скорость менять с 115200 не нужно для модуля второй версии
    gpsSerial.begin(9600);
    // ВАЖНО: РАДИОМОДУЛЬ НЕ БУДЕТ РАБОТАТЬ, БЕЗ SD КАРТЫ!
    // С недочётом сделана платы, проблема физическая
    setupNrf();

    Serial.print(F("BC board is ready...\n"));

    uint32_t gpsUpdateTimeMark = 0;

    uint32_t imuSendTimeMark = 0;
    uint32_t gpsSendTimeMark = 0;
    uint32_t phtSendTimeMark = 0;
	
	uint32_t sdWriteTimeMark = 0;

    while(true) {
	    if (gpsUpdateTimeMark < millis() && gpsUpdateInterval >= MIN_INTERVAL_VALUE) {
		    updateGPSData();
		    gpsUpdateTimeMark = millis() + gpsUpdateInterval;
	    }
	    if (gpsSendTimeMark < millis() && gpsSendInterval >= MIN_INTERVAL_VALUE) {
		    if (isTargetPosition()) { // Если мы находимся в нужной позиции
                sendGpsData();
                gpsSendTimeMark = millis() + gpsSendInterval;
		    }
		    else { gpsSendTimeMark = millis() + gpsSendInterval/10; }
	    }
	    if (imuSendTimeMark < millis() && imuSendInterval >= MIN_INTERVAL_VALUE) {
		    if (isTargetPosition()) { // Если мы находимся в нужной позиции
                sendImuData();
                imuSendTimeMark = millis() + imuSendInterval;
		    }
		    else { imuSendTimeMark = millis() + imuSendInterval/10; }
	    }
	    if (phtSendTimeMark < millis() && phtSendInterval >= MIN_INTERVAL_VALUE) {
		    if (isTargetPosition()) { // Если мы находимся в нужной позиции
                sendPhtData();
                phtSendTimeMark = millis() + phtSendInterval;
		    }
		    else { phtSendTimeMark = millis() + phtSendInterval/10; }
	    }
		if (sdWriteTimeMark < millis() && sdWriteInterval >= MIN_INTERVAL_VALUE) {
            // Чтобы сразу инициализировать карту, при её подключении
            if (FLAGS&SD_CARD_INITIALIZATRED) {
                sdWriteData();
				sdWriteTimeMark = millis() + sdWriteInterval;
            }
		    else if (SD.begin(PIN_CHIP_SELECT)) {
		        FLAGS |= SD_CARD_INITIALIZATRED;
                sdWriteData();
                sdWriteTimeMark = millis() + sdWriteInterval;
		    }
            else { sdWriteTimeMark = millis() + sdWriteInterval/2; }
	    }
	    if (nrf24.available()) {
            nrf24Request();
	    }
  }
}

void loop() {}
