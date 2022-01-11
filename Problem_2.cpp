/* Задача №2
*  Это фрагмент кода из BUSOS.cpp, отвечающий за выполнение задачи по программирования №2
*  Файл BUSOS.cpp находится в этом же каталоге на GitHub
*/
#include <MCP3008.h>
#include <Servo.h>

#define MCP3008_CLK  5
#define MCP3008_DOUT 6
#define MCP3008_DIN  7
#define MCP3008_CS   8

#define MOTOR_1_PIN         12
#define MOTOR_2_PIN         10
#define MOTOR_RESET_PIN     4
// Пока разница больше MIN_LIGHT_DIFFERENS продолжается поворот
#define MIN_LIGHT_DIFFERENS 5
// Пока разница меньше MIN_LIGHT_DIFFERENS поворот не проиходит
#define MAX_LIGHT_DIFFERENS 30
#define PWM_MIN             1000
#define PWM_CENTER          1500
#define PWM_MAX             2000

// Интервалы
#define phtTimeInterval 50
#define posTimeInterval 50

MCP3008       mcp3008(MCP3008_CLK, MCP3008_DIN, MCP3008_DOUT, MCP3008_CS);
Servo         motorLeft;
Servo         motorRight;

struct Range {
    float min = 0;
    float max = 1000;
};

// Значения с АЦП
uint16_t phtValues[8] = {};
// Диапозон измерений каждого фоторезистора
Range phtCalibRange[8];

// Состояние моторов
bool motorLeftEnable = false;
bool motorRightEnable = false;

// Обновление данных
void updatePhtValues() {
    for (int i = 0; i <= 7; ++i) {
        phtValues[i] = mcp3008.readADC(i);
    }
}


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
// Получить значения освещённости с конткретного фоторезистора
float getPhtValue(int index) {
    return map(phtValues[index], 0, 1023, phtCalibRange[index].min, phtCalibRange[index].max);
}
// Получить среднея значения освещённости на панели
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
			// Если разница между сторонами меньше чем минимально допусмимая
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
                motorLeft.writeMicroseconds(PWM_MIN); // stop
                motorLeftEnable = false;
            }
        }
        else {
            if (getBoardValue(maxIndex) - getBoardValue(lIndex) > MAX_LIGHT_DIFFERENS) {
                motorLeft.writeMicroseconds(PWM_CENTER); // start
                motorLeftEnable = true;
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial.print(F("Serial init OK\r\n"));

    pinMode(MOTOR_RESET_PIN, OUTPUT);
    motorLeft.attach(MOTOR_1_PIN);
    motorRight.attach(MOTOR_2_PIN);
    motorInit();

    uint32_t phtTimeMark = phtTimeInterval + millis();
    uint32_t posTimeMark = posTimeInterval + millis();

    while(true) {
	    if (phtTimeMark < millis()) {
		    updatePhtValues();
		    phtTimeMark = millis() + phtTimeInterval;
	    }
	    if (posTimeMark < millis()) {
            startPositioning();
            posTimeMark = millis() + posTimeInterval;
	    }
  }
}

void loop() {}