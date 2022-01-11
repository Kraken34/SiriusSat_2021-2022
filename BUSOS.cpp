#include <GOST4401_81.h>
#include <TroykaIMU.h>
#include <MCP3008.h>
#include <string.h>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>

#include <EEPROM.h>
#define EEPROM_PHT_OFFSET 0

#define MCP3008_CLK  5
#define MCP3008_DOUT 6
#define MCP3008_DIN  7
#define MCP3008_CS   8

#define I2C_SEP   0x1
#define I2C_BUEMU 0x2
#define I2C_BUSOS 0x3

#define MOTOR_1_PIN     12
#define MOTOR_2_PIN     10
#define MOTOR_RESET_PIN 4

MCP3008       mcp3008(MCP3008_CLK, MCP3008_DIN, MCP3008_DOUT, MCP3008_CS);
Gyroscope     gyroscope;
Accelerometer accelerometer;
Compass       compass;
Barometer     barometer;
Servo         motor_1;
Servo         motor_2;

/* Интервалы, специально сделаны переменными, чтобы
/* менять во время выполнения программы, но пока не реализовано */
uint16_t IMU_Delay = 1000;
uint16_t pht_Delay = 50;
uint16_t pos_Delay = 50;

uint8_t lastRequestI2C = 0;

struct Vector {
	float x = 0;
	float y = 0;
	float z = 0;
};
struct Euler {
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;
};
struct Range {
    float min;
    float max;
};

float press = 0, temp = 0, altitude = 0, azimut = 0;
Vector gyro, acl, mgn;
Euler rotateAngle;
uint16_t phtValues[8] = {};
Range phtCalibRange[8];

void motorReset() {
    delay(1000);
    digitalWrite(MOTOR_RESET_PIN, HIGH);
    delay(2);
    digitalWrite(MOTOR_RESET_PIN, LOW);
    delay(100);
    digitalWrite(MOTOR_RESET_PIN, HIGH);
    Serial.print("Restart complite\n");
}
void motorInit() {
    motorReset();

    motor_1.writeMicroseconds(2000);
    motor_2.writeMicroseconds(2000);
    Serial.print("Speed: 2000\n");
    delay(3000);
    motor_1.writeMicroseconds(1000);
    motor_2.writeMicroseconds (1000);
    Serial.print("Speed: 1000\n");
    delay(10000);
    Serial.print("Motor is ready...\n");
}
float getLedValue(int index) {
    return map(phtValues[index], 0, 1023, phtCalibRange[index].min, phtCalibRange[index].max);
}
float getBoardValue(int index) {
    return getLedValue(index*2) + getLedValue(index*2 + 1);
}
int findMax(int p1, int p2, int p3, int p4) {
    int index = -1;
    int m = -1;
    if (p1 >= m) {
        index = 0;
        m = p1;
    }
    if (p2 >= m) {
        index = 1;
        m = p2;
    }
    if (p3 >= m) {
        index = 2;
        m = p3;
    }
    if (p4 >= m) {
        index = 3;
        m = p4;
    }
    return index;
}
#define MIN_INTERVAL 5
#define MAX_INTERVAL 30
bool motorLeftEnable = false;
bool motorRightEnable = false;
void setPosition() {

    int p1 = getBoardValue(0);
    int p2 = getBoardValue(1);
    int p3 = getBoardValue(2);
    int p4 = getBoardValue(3);
    int maxIndex = findMax(p1, p2, p3, p4);

    int lIndex = maxIndex - 1;
    if (lIndex < 0) { lIndex = 3; }
    int rIndex = maxIndex + 1;
    if (rIndex > 3) { rIndex = 0; }

    // motor_2 - right
    bool rightPosition = getBoardValue(rIndex) >= getBoardValue(lIndex);
    if (rightPosition && motorLeftEnable) { motor_1.writeMicroseconds(1000); }
    if (!rightPosition && motorRightEnable) { motor_2.writeMicroseconds(1000); }

    Serial.print("\nStart\n");
    Serial.print("lIndex: "); Serial.println(lIndex);
    Serial.print("maxIndex: "); Serial.println(maxIndex);
    Serial.print("rIndex: "); Serial.println(rIndex);

    Serial.print("\nlIndexValue: "); Serial.println(getBoardValue(lIndex));
    Serial.print("maxIndexValue: "); Serial.println(getBoardValue(maxIndex));
    Serial.print("rIndexValue: "); Serial.println(getBoardValue(rIndex));

    Serial.print("\nmotorRightEnable: "); Serial.println(motorRightEnable);
    Serial.print("motorLeftEnable: "); Serial.println(motorLeftEnable);

    if (rightPosition) {
        if (motorRightEnable) {
            if (getBoardValue(maxIndex) - getBoardValue(rIndex) < MIN_INTERVAL) {
                motor_2.writeMicroseconds(1000); // stop
                motorRightEnable = false;
            }
        }
        else {
            if (getBoardValue(maxIndex) - getBoardValue(rIndex) > MAX_INTERVAL) {
                motor_2.writeMicroseconds(1500); // start
                motorRightEnable = true;
            }
        }
    }
    else {
        if (motorLeftEnable) {
            if (getBoardValue(maxIndex) - getBoardValue(lIndex) < MIN_INTERVAL) {
                motor_1.writeMicroseconds(1000); // stop
                motorLeftEnable = false;
            }
        }
        else {
            if (getBoardValue(maxIndex) - getBoardValue(lIndex) > MAX_INTERVAL) {
                motor_1.writeMicroseconds(1500); // start
                motorLeftEnable = true;
            }
        }
    }
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

// Обновление данных
void updateIMUData() {
    accelerometer.readAccelerationAXYZ(acl.x, acl.y, acl.z);

    compass.readMagneticGaussXYZ(mgn.x, mgn.y, mgn.z);

    gyroscope.readRotationDegXYZ(gyro.x, gyro.y, gyro.z);
    azimut = compass.readAzimut();

    temp = barometer.readTemperatureC();
    press = barometer.readPressurePascals();
    altitude = GOST4401_getAltitude(press);
}
void updatePhtValues() {
    for (int i = 0; i <= 7; ++i) {
        phtValues[i] = mcp3008.readADC(i);
    }
}

// Функции чтения и преобразования
uint32_t floatToUint32(float value, int32_t minValue, int32_t maxValue, int32_t resolution) {
    if (minValue <= value && value <= maxValue) {
        return (value - minValue) * resolution/ (maxValue - minValue);
    }
    else { return 0; }
}
bool parseFloat(float *f) {
    do { *f = Serial.parseFloat(SKIP_NONE); }
    while(*f == 0);
    Serial.print(*f);
    return true;
}
bool parseBool() {
    char c = 0;
    while(c != 'y' && c != 'n' && c != 'Y' && c != 'N') {
        if (Serial.available()) { c = Serial.read(); }
    }
    Serial.print(c);
    return (c == 'y' || c == 'Y');
}

// Калибровка
void calibrationPht() {
    float minAbsoluteValue, maxAbsoluteValue;
    do { Serial.print(F("Начало процедуры калибровки...\nВведите эталонное значение минимума: ")); }
    while(!parseFloat(&minAbsoluteValue));

    do { Serial.print(F("\nВведите эталонное значение максимума: ")); }
    while(!parseFloat(&maxAbsoluteValue));
    Serial.print('\n');
    delay(500);
    Serial.print('\n');

    uint8_t step = 0, phase = 0, measureCount = 10;
    uint16_t updateDelay = 150;
    Range furstPhtRange, secondPhtRange;
    uint32_t timeCounter = 0;
    while(step != 4) {
        if (timeCounter < millis()) {
            if (phase == 0) { Serial.print(F("Текущее значение минимума фоторезисторов (")); }
            else { Serial.print(F("Текущее значение максимума фоторезисторов (")); }

            Serial.print(step*2 + 1); Serial.print(F(") и (")); Serial.print(step*2 + 2); Serial.print("): ");
            Serial.print(mcp3008.readADC(step*2)); Serial.print(' '); Serial.println(mcp3008.readADC(step*2 + 1));

            timeCounter = millis() + updateDelay;
        }
        while(Serial.available()) {
            if (Serial.read() == 'C') {
                int32_t request = Serial.parseInt(SKIP_NONE);
                if (!request) { serialRequestInvalid(); continue; }

                switch (request) {
                    case 1: {
                        if(phase == 0) {
                            furstPhtRange.min = 0;
                            secondPhtRange.min = 0;
                            for (uint8_t i = 0; i < measureCount; ++i) {
                                furstPhtRange.min += mcp3008.readADC(step*2);
                                secondPhtRange.min += mcp3008.readADC(step*2 + 1);
                            }
                            furstPhtRange.min /= measureCount;
                            secondPhtRange.min /= measureCount;

                            Serial.print(F("\nСредний минимум из "));
                            Serial.print(measureCount);
                            Serial.print(F(" значений для фоторезисторов ("));
                            Serial.print(step*2 + 1);
                            Serial.print(F(") и ("));
                            Serial.print(step*2 + 2);
                            Serial.print("): ");
                            Serial.print(furstPhtRange.min);
                            Serial.print(' ');
                            Serial.print(secondPhtRange.min);

                            Serial.print(F("\nПродолжить? (y\\n): "));
                            if (!parseBool()) { Serial.println('\n'); continue; }
                            Serial.println('\n');
                            ++phase;
                        }
                        else {
                            furstPhtRange.max = 0;
                            secondPhtRange.max = 0;
                            for (uint8_t i = 0; i < measureCount; ++i) {
                                furstPhtRange.max += mcp3008.readADC(step*2);
                                secondPhtRange.max += mcp3008.readADC(step*2 + 1);
                            }
                            furstPhtRange.max /= measureCount;
                            secondPhtRange.max /= measureCount;

                            Serial.print(F("\nСредний максимум из "));
                            Serial.print(measureCount);
                            Serial.print(F(" значений для фоторезисторов ("));
                            Serial.print(step*2 + 1);
                            Serial.print(F(") и ("));
                            Serial.print(step*2 + 2);
                            Serial.print("): ");
                            Serial.print(furstPhtRange.max);
                            Serial.print(' ');
                            Serial.print(secondPhtRange.max);

                            calcRange(furstPhtRange.min, furstPhtRange.max, minAbsoluteValue, maxAbsoluteValue,
                                      &phtCalibRange[step*2].min, &phtCalibRange[step*2].max);
                            calcRange(secondPhtRange.min, secondPhtRange.max, minAbsoluteValue, maxAbsoluteValue,
                                      &phtCalibRange[step*2 + 1].min, &phtCalibRange[step*2 + 1].max);

                            Serial.print(F("\nДиапазон измерений фоторезистора ("));
                            Serial.print(step*2 + 1);
                            Serial.print("): ");
                            Serial.print(phtCalibRange[step*2].min);
                            Serial.print(' ');
                            Serial.print(phtCalibRange[step*2].max);

                            Serial.print(F("\nДиапазон измерений фоторезистора ("));
                            Serial.print(step*2 + 2);
                            Serial.print("): ");
                            Serial.print(phtCalibRange[step*2 + 1].min);
                            Serial.print(' ');
                            Serial.print(phtCalibRange[step*2 + 1].max);


                            Serial.print(F("\nПродолжить? (y\\n): "));
                            if (!parseBool()) { Serial.println('\n'); continue; }
                            Serial.println('\n');

                            phase = 0;
                            ++step;
                        }
                        break; }
                    case 2: {
                        Serial.read();
                        int32_t value = Serial.parseInt(SKIP_NONE);
                        if (value <= 0) { Serial.print(F("Некорректные параметры команды\n")); break; }
                        if (value > 65535) { Serial.print(F("Максимальное допустимое значение: 65535\n")); break; }
                        updateDelay = value;
                        while(Serial.available() && Serial.read() != '\n') {}
                        break; }
                    case 3: {
                        Serial.read();
                        int32_t value = Serial.parseInt(SKIP_NONE);
                        if (value <= 0) { Serial.print(F("Некорректные параметры команды\n")); break; }
                        if (value > 255) { Serial.print(F("Максимальное допустимое значение: 255\n")); break; }
                        measureCount = value;
                        while(Serial.available() && Serial.read() != '\n') {}
                        break; }
                    case 4: {
                        Serial.print(F("\nКалибровка прервана\n"));
                        while(Serial.available() && Serial.read() != '\n') {}
                        return; }
                    default:
                        serialRequestIndefined(request);
                };
            }
            else { serialRequestInvalid(); }
        }
    }
    Serial.print(F("Процедура калибровка закончена\n"));
    for (uint8_t i = 0; i < 8; ++i) {
        Serial.print(F("Диапазон измерений фоторезистора ("));
        Serial.print(i + 1);
        Serial.print("): ");
        Serial.print(phtCalibRange[i].min);
        Serial.print(' ');
        Serial.println(phtCalibRange[i].max);
    }
    Serial.print(F("Следует вызвать команду \"K72\", чтобы сохранить данные в EEPROM\n"));
}
void savePhtCalibRange() {
    uint8_t data[sizeof(Range)];
    memcpy(data, phtCalibRange, sizeof(Range));

    for(uint8_t i = EEPROM_PHT_OFFSET; i < EEPROM_PHT_OFFSET+sizeof(Range); ++i) {
        EEPROM.update(i, data[i]);
    }
}
void loadPhtCalibRange() {
    uint8_t data[sizeof(Range)];
    for(uint8_t i = EEPROM_PHT_OFFSET; i < EEPROM_PHT_OFFSET+sizeof(Range); ++i) {
        data[i] = EEPROM.read(i);
    }
    memcpy(phtCalibRange, data, sizeof(Range));
}
bool calcRange(float minADC, float maxADC, float minValue, float maxValue, float *minRange, float *maxRange) {
    float minValuePr = minADC/10.23f;
    float maxValuePr = maxADC/10.23f;

    if (maxValuePr == minValuePr) { return false; }
    float UnitPerPr = (maxValue - minValue) / (maxValuePr - minValuePr);

    *minRange = minValue - minValuePr*UnitPerPr;
    *maxRange = maxValue + (100 - maxValuePr)*UnitPerPr;

    return true;
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
				case 10: serialRequest_10(); break;
				case 31: serialRequest_31(); break;
				case 35: serialRequest_35(); break;
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
}
void serialRequest_10() {
	calibrationPht();
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

// Запросы по I2C
void onReceiveI2C(int count) {
    while(count) {
        lastRequestI2C = Wire.read();
        --count;
    }
}
void onRequestI2C() {
    switch (lastRequestI2C) {
    case 31: {
        uint8_t data[23];
        uint32_t value;

        value = floatToUint32(press, 26000, 126000, 16777215);
        memcpy(data+0, &value, 3);

        value = floatToUint32(temp, -30, 110, 4095);
        memcpy(data+3, &value, 2);

        value = floatToUint32(gyro.x, -250000, 250000, 65535);
        memcpy(data+5, &value, 2);
        value = floatToUint32(gyro.y, -250000, 250000, 65535);
        memcpy(data+7, &value, 2);
        value = floatToUint32(gyro.z, -250000, 250000, 65535);
        memcpy(data+9, &value, 2);

        value = floatToUint32(acl.x, -8000, 8000, 65535);
        memcpy(data+11, &value, 2);
        value = floatToUint32(acl.y, -8000, 8000, 65535);
        memcpy(data+13, &value, 2);
        value = floatToUint32(acl.z, -8000, 8000, 65535);
        memcpy(data+15, &value, 2);

        value = floatToUint32(mgn.x, -16000, 16000, 65535);
        memcpy(data+17, &value, 2);
        value = floatToUint32(mgn.y, -16000, 16000, 65535);
        memcpy(data+19, &value, 2);
        value = floatToUint32(mgn.z, -16000, 16000, 65535);
        memcpy(data+21, &value, 2);

        for (uint8_t i = 0; i < 23; ++i) { Wire.write(data[i]); }
        break;
    }
    case 43: {
        uint8_t data[16];
        memcpy(data, phtValues, 16);
        for (uint8_t i = 0; i < 16; ++i) { Wire.write(data[i]); }
        break;
    }
    case 70: {
        uint8_t data[4];
        for (uint8_t i = 0; i < 8; ++i) {
            memcpy(data, &phtCalibRange[i].min, sizeof(float));
            for (uint8_t q = 0; q < sizeof(float); ++q) { Wire.write(data[q]); }
            memcpy(data, &phtCalibRange[i].max, sizeof(float));
            for (uint8_t q = 0; q < sizeof(float); ++q) { Wire.write(data[q]); }
        }
        break;
    }
    };
    lastRequestI2C = 0;
}

void setup() {
    Serial.begin(115200);
    Serial.print(F("Serial init OK\r\n"));

    setupIMU();

    Wire.begin(I2C_BUSOS);
    Wire.onRequest(onRequestI2C);
    Wire.onReceive(onReceiveI2C);

    pinMode(MOTOR_RESET_PIN, OUTPUT);
    motor_1.attach(MOTOR_1_PIN);
    motor_2.attach(MOTOR_2_PIN);
    motorInit();

    uint32_t IMUDelay = IMU_Delay + millis();
    uint32_t phtDelay = pht_Delay + millis();
    uint32_t posDelay = pos_Delay + millis();

    while(true) {
	    if (IMUDelay < millis()) {
		    updateIMUData();
		    IMUDelay = millis() + IMU_Delay;
	    }
	    if (phtDelay < millis()) {
		    updatePhtValues();
		    phtDelay = millis() + pht_Delay;
	    }
	    if (posDelay < millis()) {
            setPosition();
            posDelay = millis() + pos_Delay;
	    }
	    if (Serial.available()) {
		    serialRequest();
	    }
  }
}

void loop() {}
