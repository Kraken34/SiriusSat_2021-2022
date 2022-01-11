/* Задача №1
*  Это фрагмент кода из BUSOS.cpp, отвечающий за выполнение задачи по программирования №1
*  Файл BUSOS.cpp находится в этом же каталоге на GitHub
*  Фрагмент ОТДЕЛЬНО НЕ СКОМПИЛИРУЕТСЯ!
*/

// Значения с АЦП
uint16_t phtValues[8] = {};
// Диапозон измерений каждого фоторезистора
Range phtCalibRange[8];

// Главная функция калибровки
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
	// updateDelay - Задержка обновления данных в консоле о фоторезисторе
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

							// Вычисленение полного истинного диапозона измерения фоторезистора
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
                    case 4: { // Прерыв колибровки
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
	// Минимальный процент, на который был изпользован фоторезистор
    float minValuePr = minADC/10.23f;
	// Максимальный процент, на который был изпользован фоторезистор
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