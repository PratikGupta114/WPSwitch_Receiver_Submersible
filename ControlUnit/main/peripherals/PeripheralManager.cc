#include "PeripheralManager.h"
#include "../switching_unit/SwitchingUnitManager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "freertos/timers.h"

#define TAG "PeripheralManager.cpp"

static QueueHandle_t gpioEventMessageQueue;
static SemaphoreHandle_t peripheralInitializationMutex;

uint8_t left[] = {
    // CDEGFBA
    0b01110111, // 0
    0b01000010, // 1
    0b00111011, // 2
    0b01101011, // 3
    0b01001110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b01000011, // 7
    0b01111111, // 8
    0b01101111, // 9
};

uint8_t right[] = {
    // AFBCGED
    0b01111011, // 0
    0b00011000, // 1
    0b01010111, // 2
    0b01011101, // 3
    0b00111100, // 4
    0b01101101, // 5
    0b01101111, // 6
    0b01011000, // 7
    0b01111111, // 8
    0b01111101, // 9
};

static void IRAM_ATTR motor_toggle_button_isr_handler(void *arg)
{
    gpio_num_t gpioNumber = MOTOR_TOGGLE_BUTTON_INPUT_PIN;
    xQueueSendFromISR(gpioEventMessageQueue, &gpioNumber, NULL);
}

void config_button_timer_callback(TimerHandle_t xTimer)
{
    PeripheralManager *pm = (PeripheralManager *)pvTimerGetTimerID(xTimer);
    pm->configButtonLongPressTriggered = true;
    if (pm->onConfigButtonLongPressedListener != NULL)
    {
        pm->onConfigButtonLongPressedListener();
    }
}

static void IRAM_ATTR config_button_isr_handler(void *arg)
{
    gpio_num_t gpioNumber = CONFIG_MODE_BUTTON_INPUT;
    xQueueSendFromISR(gpioEventMessageQueue, &gpioNumber, NULL);
}

void delayMicroseconds(uint32_t us)
{
    uint64_t m = (uint64_t)esp_timer_get_time();
    if (us)
    {
        uint64_t e = (m + us);
        if (m > e)
        { // overflow
            while ((uint64_t)esp_timer_get_time() > e)
            {
                NOP();
            }
        }
        while ((uint64_t)esp_timer_get_time() < e)
        {
            NOP();
        }
    }
}

void shiftOut(gpio_num_t dataPin, gpio_num_t clockPin, uint8_t bitOrder, uint8_t val)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        if (bitOrder == LSBFIRST)
        {
            // digitalWrite(dataPin, !!(val & (1 << i)));
            ESP_ERROR_CHECK(gpio_set_level(dataPin, !!(val & (1 << i))));
        }
        else
        {
            ESP_ERROR_CHECK(gpio_set_level(dataPin, !!(val & (1 << (7 - i)))));
        }

        ESP_ERROR_CHECK(gpio_set_level(clockPin, 0));
        ESP_ERROR_CHECK(gpio_set_level(clockPin, 1));
        
        // Yield every 4 iterations to prevent IDLE task starvation during high system load
        // This allows the FreeRTOS scheduler to run lower-priority tasks (including IDLE)
        // which must periodically reset the Task Watchdog Timer (TWDT)
        if (i % 4 == 3) {
            taskYIELD();
        }
    }
}

void gpioEventHandlerTask(void *args)
{
    ESP_LOGI(TAG, "Created gpioEventHandlerTask");
    PeripheralManager *peripheralManager = (PeripheralManager *)args;
    gpio_num_t gpioNumber;
    xSemaphoreGive(peripheralInitializationMutex);

    for (;;)
    {
        if (xQueueReceive(gpioEventMessageQueue, &gpioNumber, portMAX_DELAY))
        {
            switch (gpioNumber)
            {
            case MOTOR_TOGGLE_BUTTON_INPUT_PIN:
            {
                ButtonState state = gpio_get_level((gpio_num_t)MOTOR_TOGGLE_BUTTON_INPUT_PIN) > 0 ? PRESSED : RELEASED;
                if (peripheralManager->motorToggleButtonState != state)
                {
                    peripheralManager->motorToggleButtonState = state;
                    // ESP_LOGI(TAG, "Motor Toggle Button state : %s", state == PRESSED ? "PRESSED" : "RELEASED");
                    if (state == PRESSED && peripheralManager->onMotorToggleButtonPressedListener != NULL)
                        peripheralManager->onMotorToggleButtonPressedListener();
                }
            }
            break;
            case CONFIG_MODE_BUTTON_INPUT:
            {
                ButtonState state = gpio_get_level((gpio_num_t)CONFIG_MODE_BUTTON_INPUT) > 0 ? PRESSED : RELEASED;
                if (peripheralManager->configButtonState != state)
                {
                    // On press start timer; on release stop timer and decide
                    if (state == PRESSED)
                    {
                        peripheralManager->configButtonLongPressTriggered = false;
                        if (peripheralManager->configButtonTimer == NULL)
                        {
                            peripheralManager->configButtonTimer = xTimerCreate("cfgBtnTimer", pdMS_TO_TICKS(CONFIG_BUTTON_LONG_PRESS_TIMEOUT_MS), pdFALSE, (void *)peripheralManager, config_button_timer_callback);
                        }
                        if (peripheralManager->configButtonTimer != NULL)
                        {
                            xTimerStop(peripheralManager->configButtonTimer, 0);
                            xTimerChangePeriod(peripheralManager->configButtonTimer, pdMS_TO_TICKS(CONFIG_BUTTON_LONG_PRESS_TIMEOUT_MS), 0);
                            xTimerStart(peripheralManager->configButtonTimer, 0);
                        }
                    }
                    else // RELEASED
                    {
                        if (peripheralManager->configButtonTimer != NULL)
                        {
                            xTimerStop(peripheralManager->configButtonTimer, 0);
                        }
                        if (peripheralManager->configButtonLongPressTriggered == false)
                        {
                            if (peripheralManager->onConfigButtonClickedListener != NULL)
                                peripheralManager->onConfigButtonClickedListener();
                        }
                    }
                    peripheralManager->configButtonState = state;
                }
            }
            break;
            default:
            {
            }
            break;
            }
        }
    }
}

void PeripheralManager::initPeripherals(SwitchingUnitManager* switchingUnitManager)
{
    this->_switchingUnitManager = switchingUnitManager;

    // Create task and message queue to listen to button interrupts
    peripheralInitializationMutex = xSemaphoreCreateBinary();
    gpioEventMessageQueue = xQueueCreate(5, sizeof(gpio_num_t));
    xTaskCreatePinnedToCore(
        gpioEventHandlerTask,
        "gpioEventHandlerTask",
        4096,  // Increased from 2560 - button callback triggers deep call chain (pump control + cJSON + MQTT publish)
        this,
        10,
        NULL,
        APP_CPU_NUM);

    // Let's yield to the task created recently
    vTaskDelay(pdMS_TO_TICKS(5));

    // Set safe levels for all output pins FIRST
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_TOGGLE_BUTTON_LED_OUTPUT_PIN, 0)); // LED OFF
    ESP_ERROR_CHECK(gpio_set_level(LATCH_PIN, 0));                // Display latch LOW
    ESP_ERROR_CHECK(gpio_set_level(CLOCK_PIN, 0));                // Display clock LOW
    ESP_ERROR_CHECK(gpio_set_level(DATA_PIN, 0));                 // Display data LOW
    ESP_ERROR_CHECK(gpio_set_level(WIRELESS_PRG_MODE_PIN, 1));    // RF module normal mode
    
    // Now configure all output pins
    ESP_ERROR_CHECK(gpio_set_direction(MOTOR_TOGGLE_BUTTON_LED_OUTPUT_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(LATCH_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(CLOCK_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(DATA_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(WIRELESS_PRG_MODE_PIN, GPIO_MODE_OUTPUT));
    
    // Disable pull-ups/pull-downs on output pins
    gpio_set_pull_mode(MOTOR_TOGGLE_BUTTON_LED_OUTPUT_PIN, GPIO_FLOATING);
    gpio_set_pull_mode(LATCH_PIN, GPIO_FLOATING);
    gpio_set_pull_mode(CLOCK_PIN, GPIO_FLOATING);
    gpio_set_pull_mode(DATA_PIN, GPIO_FLOATING);
    gpio_set_pull_mode(WIRELESS_PRG_MODE_PIN, GPIO_FLOATING);

    // Toggle Button
    gpio_config_t toggleButtonGpioConfiguration = {
        .pin_bit_mask = ((1ULL << MOTOR_TOGGLE_BUTTON_INPUT_PIN)),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&toggleButtonGpioConfiguration));

    // Config Mode Button (GPIO34 input only, no pull-ups/downs)
    gpio_config_t configButtonGpioConfiguration = {
        .pin_bit_mask = ((1ULL << CONFIG_MODE_BUTTON_INPUT)),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&configButtonGpioConfiguration));

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(MOTOR_TOGGLE_BUTTON_INPUT_PIN, motor_toggle_button_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_MODE_BUTTON_INPUT, config_button_isr_handler, NULL));

    xSemaphoreTake(peripheralInitializationMutex, portMAX_DELAY);
    this->peripheralsConfigured = true;

    // Read the actual current motor state from hardware
    this->currentMotorState = this->getCurrentMotorState();
    ESP_LOGI(TAG, "Initial motor state read from switching unit: %s", (this->currentMotorState == ON ? "ON" : "OFF"));
    
    this->buzzer.noTone();
    this->currentBuzzerState = OFF;
    this->setMotorToggleButtonLedState(this->currentMotorState);
}

PeripheralState PeripheralManager::getCurrentMotorState()
{
    if (_switchingUnitManager) {
        this->currentMotorState = _switchingUnitManager->isMotorRunning() ? ON : OFF;
    } else {
        this->currentMotorState = OFF;
    }
    return this->currentMotorState;
}
PeripheralState PeripheralManager::getCurrentBuzzerState()
{
    return this->currentBuzzerState;
}
PeripheralState PeripheralManager::getCurrentMotorToggleButtonLedState()
{
    return this->currentMotorToggleButtonLedState;
}

void PeripheralManager::enableRFModuleProgramMode()
{
    if (!peripheralsConfigured)
    {
        ESP_LOGE(TAG, "Peripherals need to be initialized before enabling / disabling RF Program Mode");
        return;
    }
    ESP_ERROR_CHECK(gpio_set_level(WIRELESS_PRG_MODE_PIN, 0));
}
void PeripheralManager::disableRFModuleProgramMode()
{
    if (!peripheralsConfigured)
    {
        ESP_LOGE(TAG, "Peripherals need to be initialized before enabling / disabling RF Program Mode");
        return;
    }
    ESP_ERROR_CHECK(gpio_set_level(WIRELESS_PRG_MODE_PIN, 1));
    // vTaskDelay(pdMS_TO_TICKS(1));
    // ESP_ERROR_CHECK(gpio_set_level(WIRELESS_PRG_MODE_PIN, 0));
    // vTaskDelay(pdMS_TO_TICKS(1));
    // ESP_ERROR_CHECK(gpio_set_level(WIRELESS_PRG_MODE_PIN, 1));
}
void PeripheralManager::setMotorState(PeripheralState newMotorState)
{
    if (this->peripheralsConfigured == false)
    {
        ESP_LOGE(TAG, "Peripherals need to be configured first");
        return;
    }
    if (this->currentMotorState != newMotorState)
    {
        if (_switchingUnitManager) {
            if (newMotorState == ON)
            {
                _switchingUnitManager->sendPumpOn();
            }
            else
            {
                _switchingUnitManager->sendPumpOff();
            }
        }
        this->currentMotorState = newMotorState;
    }
}
void PeripheralManager::setBuzzerState(PeripheralState newBuzzerState)
{
    if (this->peripheralsConfigured == false)
    {
        ESP_LOGE(TAG, "Peripherals need to be configured first");
        return;
    }
    if (this->currentBuzzerState != newBuzzerState)
    {
        if (newBuzzerState == ON)
        {
            this->buzzer.tone(NOTE_GS7);
        }
        else
        {
            this->buzzer.noTone();
        }
        this->currentBuzzerState = newBuzzerState;
    }
}
void PeripheralManager::setMotorToggleButtonLedState(PeripheralState newLedState)
{
    if (this->peripheralsConfigured == false)
    {
        ESP_LOGE(TAG, "Peripherals need to be configured first");
        return;
    }
    if (this->currentMotorToggleButtonLedState != newLedState)
    {
        ESP_ERROR_CHECK(gpio_set_level(MOTOR_TOGGLE_BUTTON_LED_OUTPUT_PIN, (newLedState == ON) ? 1 : 0));
        this->currentMotorToggleButtonLedState = newLedState;
    }
}
void PeripheralManager::setOnMotorToggleButtonPressedListener(function<void()> onMotorToggleButtonPressedListener)
{
    this->onMotorToggleButtonPressedListener = onMotorToggleButtonPressedListener;
}

void PeripheralManager::setOnConfigButtonClickedListener(function<void()> onConfigButtonClickedListener)
{
    this->onConfigButtonClickedListener = onConfigButtonClickedListener;
}

void PeripheralManager::setOnConfigButtonLongPressedListener(function<void()> onConfigButtonLongPressedListener)
{
    this->onConfigButtonLongPressedListener = onConfigButtonLongPressedListener;
}

void PeripheralManager::displayTwoDigitNumber(uint8_t number)
{
    if (number > 99)
    {
        ESP_LOGE("displayTwoDigitNumber()", "received a number beyond acceptable range : %d", number);
        return;
    }

#if NUMERIC_DISPLAY_ENABLED == 1
    uint8_t quo = number / 10;
    uint8_t rem = number % 10;

    uint8_t leftDigitBinaryPattern = left[quo];
    uint8_t rightDigitBinaryPattern = right[rem];

    gpio_set_level(LATCH_PIN, 0);
    delayMicroseconds(10);

    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, rightDigitBinaryPattern);
    delayMicroseconds(10);
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, leftDigitBinaryPattern);
    delayMicroseconds(10);
    gpio_set_level(LATCH_PIN, 1);
    delayMicroseconds(10);
#else
    gpio_set_level(LATCH_PIN, 0);
    delayMicroseconds(10);
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, 0b00000000);
    delayMicroseconds(10);
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, 0b00000000);
    delayMicroseconds(10);
    gpio_set_level(LATCH_PIN, 1);
    delayMicroseconds(10);
#endif
}

bool PeripheralManager::isConfigButtonPressed()
{
    return gpio_get_level(CONFIG_MODE_BUTTON_INPUT) > 0;
}