#include <GOST4401_81.h>
#include <TroykaIMU.h>
#include <MCP3008.h>
#include <string.h>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>

#include <EEPROM.h>
#define EEPROM_PHT_ADDRESS 0

#define MCP3008_CLK  5
#define MCP3008_DOUT 6
#define MCP3008_DIN  7
#define MCP3008_CS   8

#define I2C_SEP   0x1
#define I2C_BUEMU 0x2
#define I2C_BUSOS 0x3

#define MOTOR_1_PIN         12
#define MOTOR_2_PIN         10
#define MOTOR_RESET_PIN     4
// Пока разница больше MIN_LIGHT_DIFFERENS продолжается поворот
#define MIN_LIGHT_DIFFERENS 5
// Пока разница меньше MIN_LIGHT_DIFFERENS поворот не происходит
#define MAX_LIGHT_DIFFERENS 30
#define PWM_MIN             1000
#define PWM_CENTER          1500
#define PWM_MAX             2000

MCP3008       mcp3008(MCP3008_CLK, MCP3008_DIN, MCP3008_DOUT, MCP3008_CS);
Gyroscope     gyroscope;
Accelerometer accelerometer;
Compass       compass;
Barometer     barometer;
Servo         motorLeft;
Servo         motorRight;

// Интервалы
#define imuTimeInterval 1000
#define phtTimeInterval 50
#define posTimeInterval 50

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

// Состояние моторов
bool motorLeftEnable = false;
bool motorRightEnable = false;

// Номер последнего запроса, полученного по I2C
uint8_t lastRequestI2C = 0;

// Значения с АЦП
uint16_t phtValues[8] = {};
// Диапазон измерений каждого фоторезистора
Range phtCalibRange[8];

Vector gyro, acl, mgn;
Euler rotateAngle;

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

    motorLeft.writeMicroseconds(PWM_MAX);
    motorRight.writeMicroseconds(PWM_MAX);
    Serial.print("Max Speed: 2000\n");
    delay(3000);
    motorLeft.writeMicroseconds(PWM_MIN);
    motorRight.writeMicroseconds (PWM_MIN);
    Serial.print("Min Speed: 1000\n");
    delay(10000);
    Serial.print("Motor is ready...\n");
}
// Получить значения освещённости с конкретного фоторезистора
float getPhtValue(int index) {
    return map(phtValues[index], 0, 1023, phtCalibRange[index].min, phtCalibRange[index].max);
}
// Получить среднее значения освещённости на панели
float getBoardValue(int index) {
    return (getPhtValue(index*2) + getPhtValue(index*2 + 1))/2;
}
int findMax(int p1, int p2, int p3, int p4) {
    int index = 0;
    int m = p1;
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
void startPositioning() {
	// Поиск максимума
    int boardValue_1 = getBoardValue(0);
    int boardValue_2 = getBoardValue(1);
    int boardValue_3 = getBoardValue(2);
    int boardValue_4 = getBoardValue(3);
    int maxIndex = findMax(boardValue_1, boardValue_2, boardValue_3, boardValue_4);

	// Крайние случаи
    int lIndex = maxIndex - 1;
    if (lIndex < 0) { lIndex = 3; }
    int rIndex = maxIndex + 1;
    if (rIndex > 3) { rIndex = 0; }

	// Определение стороны, в которую нужно двигаться
    bool rightPosition = getBoardValue(rIndex) >= getBoardValue(lIndex);

	// Если в данный момент, спутник движется не в ту сторону (резко изменилась цель)
    if (rightPosition && motorLeftEnable) { motorLeft.writeMicroseconds(PWM_MIN); }
    if (!rightPosition && motorRightEnable) { motorRight.writeMicroseconds(PWM_MIN); }

    //Serial.print("\nStart\n");
    //Serial.print("lIndex: "); Serial.println(lIndex);
    //Serial.print("maxIndex: "); Serial.println(maxIndex);
    //Serial.print("rIndex: "); Serial.println(rIndex);

    //Serial.print("\nlIndexValue: "); Serial.println(getBoardValue(lIndex));
    //Serial.print("maxIndexValue: "); Serial.println(getBoardValue(maxIndex));
    //Serial.print("rIndexValue: "); Serial.println(getBoardValue(rIndex));

    //Serial.print("\nmotorRightEnable: "); Serial.println(motorRightEnable);
    //Serial.print("motorLeftEnable: "); Serial.println(motorLeftEnable);

	// Если нужно в право
    if (rightPosition) {
		// Если мотор влючён
        if (motorRightEnable) {
			// Если разница между сторонами меньше чем минимально допустимая
            if (getBoardValue(maxIndex) - getBoardValue(rIndex) < MIN_LIGHT_DIFFERENS) {
				// Останавливаемся
                motorRight.writeMicroseconds(PWM_MIN);
                motorRightEnable = false;
            }
        }
        else {
			// Если разница между сторонами больше предельно допустимой
            if (getBoardValue(maxIndex) - getBoardValue(rIndex) > MAX_LIGHT_DIFFERENS) {
				// Начинаем вращения
                motorRight.writeMicroseconds(PWM_CENTER);
                motorRightEnable = true;
            }
        }
    }
	// Тоже самое, но в другую сторону
    else {
        if (motorLeftEnable) {
            if (getBoardValue(maxIndex) - getBoardValue(lIndex) < MIN_LIGHT_DIFFERENS) {
                // Останавливаемся
                motorLeft.writeMicroseconds(PWM_MIN);
                motorLeftEnable = false;
            }
        }
        else {
            if (getBoardValue(maxIndex) - getBoardValue(lIndex) > MAX_LIGHT_DIFFERENS) {
                // Начинаем вращения
                motorLeft.writeMicroseconds(PWM_CENTER);
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
// Сжатие данных, для более быстрой передачи
uint32_t floatToUint32(float value, int32_t minValue, int32_t maxValue, int32_t resolution) {
    if (minValue <= value && value <= maxValue) {
        return (value - minValue) * resolution/ (maxValue - minValue);
    }
    else { return 0; }
}
// Чтение float с консоли
bool parseFloat(float *f) {
    do { *f = Serial.parseFloat(SKIP_NONE); }
    while(*f == 0);
    Serial.print(*f);
    return true;
}
// Чтение bool с консоли
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

	// measureCount - количество измерений для усреднения
    uint8_t phtIndex = 0, phase = 0, measureCount = 50;
	// updateDelay - Задержка обновления данных в консоли о фоторезисторе
    uint16_t updateDelay = 150;
    Range phtRange;
    uint32_t timeCounter = 0;

    while(phtIndex <= 7) {
        // Обновление данных в консоли
        if (timeCounter < millis()) {
            if (phase == 0) { Serial.print(F("Текущее значение минимума фоторезистора (")); }
            else { Serial.print(F("Текущее значение максимума фоторезистора (")); }
            Serial.print(phtIndex + 1); Serial.print("): "); Serial.println(mcp3008.readADC(phtIndex + 1));
            timeCounter = millis() + updateDelay;
        }
        /* Если пришла команда с консоли
        Список команд:
        C1 - Следующий шаг
        C2 - Изменение задержки (updateDelay) перед обновлением данных (1 число int)
        C3 - Изменение количества измерений (measureCount) для усреднения значений (1 число int)
        C4 - Прервать процедуру калибровки */
        while(Serial.available()) {
            if (Serial.read() == 'C') {
                int32_t request = Serial.parseInt(SKIP_NONE);
                if (!request) { serialRequestInvalid(); continue; }

                switch (request) {
                    case 1: { // Следующий шаг

                        if(phase == 0) { // Если сейчас идёт поиск минимального порога
							// Среднее из measureCount значений
                            phtRange.min = 0;
                            for (uint8_t i = 0; i < measureCount; ++i) {
                                phtRange.min += mcp3008.readADC(phtIndex);
                            }
                            phtRange.min /= measureCount;

                            Serial.print(F("\nСредний минимум из "));
                            Serial.print(measureCount);
                            Serial.print(F(" значений для фоторезистора ("));
                            Serial.print(phtIndex + 1);
                            Serial.print("): ");
                            Serial.print(phtRange.min);

							// Отмена или продолжение
                            Serial.print(F("\nПродолжить? (y\\n): "));
                            if (!parseBool()) { Serial.println('\n'); continue; }
                            Serial.println('\n');
                            ++phase;
                        }
                        else { // Если сейчас идёт поиск максимального порога
							// Среднее из measureCount значений
                            phtRange.max = 0;
                            for (uint8_t i = 0; i < measureCount; ++i) {
                                phtRange.max += mcp3008.readADC(phtIndex);
                            }
                            phtRange.max /= measureCount;

                            Serial.print(F("\nСредний максимум из "));
                            Serial.print(measureCount);
                            Serial.print(F(" значений для фоторезистора ("));
                            Serial.print(phtIndex + 1);
                            Serial.print("): ");
                            Serial.print(phtRange.max);

							// Вычисление полного истинного диапазона измерения фоторезистора
                            calcRange(phtRange.min, phtRange.max, minAbsoluteValue, maxAbsoluteValue,
                                      &phtCalibRange[phtIndex].min, &phtCalibRange[phtIndex].max);

                            Serial.print(F("\nДиапазон измерений фоторезистора ("));
                            Serial.print(phtIndex + 1);
                            Serial.print("): ");
                            Serial.print(phtCalibRange[phtIndex].min);
                            Serial.print(' ');
                            Serial.print(phtCalibRange[phtIndex].max);

							// Отмена или продолжение
                            Serial.print(F("\nПродолжить? (y\\n): "));
                            if (!parseBool()) { Serial.println('\n'); continue; }
                            Serial.println('\n');

							// Теперь нужен минимум
                            phase = 0;
                            ++phtIndex;
                        }
                        break; }
                    case 2: { // Изменение задержки
                        Serial.read();
                        int32_t value = Serial.parseInt(SKIP_NONE);
                        if (value <= 0) { Serial.print(F("Некорректные параметры команды\n")); break; }
                        if (value > 65535) { Serial.print(F("Максимальное допустимое значение: 65535\n")); break; }
                        updateDelay = value;
                        while(Serial.available() && Serial.read() != '\n') {}
                        break; }
                    case 3: { // Изменение количества измерений
                        Serial.read();
                        int32_t value = Serial.parseInt(SKIP_NONE);
                        if (value <= 0) { Serial.print(F("Некорректные параметры команды\n")); break; }
                        if (value > 255) { Serial.print(F("Максимальное допустимое значение: 255\n")); break; }
                        measureCount = value;
                        while(Serial.available() && Serial.read() != '\n') {}
                        break; }
                    case 4: { // Прерывание калибровки
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
// Сохранение phtCalibRange в EEPROM
void savePhtCalibRange() {
    uint8_t data[sizeof(Range)*8];
    memcpy(data, phtCalibRange, sizeof(Range)*8);

    for(uint8_t i = EEPROM_PHT_ADDRESS; i < EEPROM_PHT_ADDRESS+sizeof(Range)*8; ++i) {
        EEPROM.update(i, data[i]);
    }
}
// Загрузка phtCalibRange из EEPROM
void loadPhtCalibRange() {
    uint8_t data[sizeof(Range)*8];
    for(uint8_t i = EEPROM_PHT_ADDRESS; i < EEPROM_PHT_ADDRESS+sizeof(Range)*8; ++i) {
        data[i] = EEPROM.read(i);
    }
    memcpy(phtCalibRange, data, sizeof(Range)*8);
}
bool calcRange(float minADC, float maxADC, float minValue, float maxValue, float *minRange, float *maxRange) {
	// Минимальный процент, на который был использован фоторезистор
    float minValuePr = minADC/10.23f;
	// Максимальный процент, на который был использован фоторезистор
    float maxValuePr = maxADC/10.23f;

	// Проверка деления на ноль
    if (maxValuePr == minValuePr) { return false; }

	// Количество единиц в одном проценте
    float UnitPerPr = (maxValue - minValue) / (maxValuePr - minValuePr);

	// Истинное минимальное и максимальное значение
    *minRange = minValue - minValuePr*UnitPerPr;
    *maxRange = maxValue + (100 - maxValuePr)*UnitPerPr;

    return true;
}

// Запрос по Serial
/* Список команд
K1 - Проверка
K10 - Начать калибровку фоторезисторов
K31 - Вывести давление и температуру
K35 - Вывести угол Эйлера
K42 - Вывести значения фоторезисторов
K43 - Вывести сырые значения фоторезисторов
K70 - Вывести калибровочные значение для фоторезисторов
K71 - Ввести калибровочные значение для фоторезисторов (16 чисел float)
K72 - Сохранить калибровочные значение для фоторезисторов в EEPROM
*/
void serialRequest() {
	while(Serial.available()) {
		// Если пришла K - команда
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
				case 71: serialRequest_71(); break;
				case 72: serialRequest_72(); break;
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
        Serial.println(getPhtValue(i));
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
        Serial.print(phtCalibRange[i].min);
        Serial.print(' ');
        Serial.println(phtCalibRange[i].max);
    }
	Serial.print(F("OK\n"));
}
void serialRequest_71() {
    Range tempPhtCalibRange[8] = {};
    for (uint8_t i = 0; i < 8; ++i) {
        tempPhtCalibRange[i].min = Serial.parseFloat();
        tempPhtCalibRange[i].max = Serial.parseFloat();
        Serial.print(tempPhtCalibRange[i].min); Serial.print(' '); Serial.println(tempPhtCalibRange[i].max);
    }
    if (Serial.parseFloat() != 0) { serialRequestToManyParam(); return; }

    for (uint8_t i = 0; i < 8; ++i) {
        phtCalibRange[i].min = tempPhtCalibRange[i].min;
        phtCalibRange[i].max = tempPhtCalibRange[i].max;
    }

	Serial.print(F("OK\n"));
}
void serialRequest_72() {
    savePhtCalibRange();
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
	Serial.print(F("Команда не распознана...\n"));
}
void serialRequestIndefined(int32_t request) {
	Serial.print(F("Команда \"")); Serial.print(request); Serial.print(F("\" недействительна...\n"));
}
void serialRequestToManyParam() {
    Serial.print(F("Ошибка: параметров больше, чем ожидалось\n"));
}
// Запросы по I2C
void onReceiveI2C(int count) {
    while(count) {
        lastRequestI2C = Wire.read();
		// lastRequestI2C - номер команды
        --count;
    }
}
void onRequestI2C() {
    switch (lastRequestI2C) {
    case 31: { // Сжатие и отправка данных IMU
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
    case 43: { // Отправка данных с АЦП
        uint8_t data[16];
        memcpy(data, phtValues, 16);
        for (uint8_t i = 0; i < 16; ++i) { Wire.write(data[i]); }
        break;
    }
    case 70: { // Отправка диапазона измерений фоторезисторов
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
	// Команда выполнена
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
    motorLeft.attach(MOTOR_1_PIN);
    motorRight.attach(MOTOR_2_PIN);
    motorInit();

    uint32_t imuTimeMark = imuTimeInterval + millis();
    uint32_t phtTimeMark = phtTimeInterval + millis();
    uint32_t posTimeMark = posTimeInterval + millis();

    while(true) {
	    if (imuTimeMark < millis()) {
		    updateIMUData();
		    imuTimeMark = millis() + imuTimeInterval;
	    }
	    if (phtTimeMark < millis()) {
		    updatePhtValues();
		    phtTimeMark = millis() + phtTimeInterval;
	    }
	    if (posTimeMark < millis()) {
            startPositioning();
            posTimeMark = millis() + posTimeInterval;
	    }
	    if (Serial.available()) {
		    serialRequest();
	    }
  }
}

void loop() {}
