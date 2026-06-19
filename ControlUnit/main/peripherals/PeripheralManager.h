#ifndef PERIPHERALMANAGER_H_
#define PERIPHERALMANAGER_H_

#include "freertos/FreeRTOS.h"
#include <functional>
#include "driver/gpio.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "BuzzerControl.h"

#include "board_pins.h"

class SwitchingUnitManager;

// Pin mappings matching the Submersible Control Unit board
#define MOTOR_TOGGLE_BUTTON_INPUT_PIN WS_SWITCH_BUTTON_INPUT_PIN
#define MOTOR_TOGGLE_BUTTON_LED_OUTPUT_PIN BUTTON_LED_OUTPUT_PIN
#define NEOPIXEL_DATA_OUTPUT_PIN NEOPIXEL_DATA_PIN
#define CONFIG_MODE_BUTTON_INPUT CONFIG_BUTTON_INPUT_PIN
#define CLOCK_PIN CLOCK_PIN_SHCP
#define LATCH_PIN LATCH_PIN_STCP
#define DATA_PIN DATA_PIN_DS
#define LED_BUILTIN GPIO_NUM_2 // Keep as GPIO2 if needed, though not mapped on the panel schematic

// WIR-1186 Wireless Related Pins
#define WIRELESS_PRG_MODE_PIN WIRELESS_PRG_PIN

// RTC- I2C Pins
#define DS1307_SDA_PIN I2C_SDA_PIN
#define DS1307_SCL_PIN I2C_SCL_PIN

#define NUMERIC_DISPLAY_ENABLED 1
#define PERIPHERAL_MANAGER_TASK_STACK_DEPTH (3 * 1024)
#define CONFIG_BUTTON_LONG_PRESS_TIMEOUT_MS 8000 // 8 seconds
#define LSBFIRST 0
#define MSBFIRST 1
#define NOP() __asm__("nop")

using namespace std;

class PeripheralManager;

typedef enum peripheral_state
{
    ON = 1,
    OFF = 0,
} PeripheralState;

typedef enum button_state
{
    PRESSED = 1,
    RELEASED = 0,
} ButtonState;

typedef struct gpio_event_message
{
    PeripheralManager *peripheralManager;
    gpio_num_t gpioNumber;
} GpioEventMessage;

class PeripheralManager
{
private:
    bool peripheralsConfigured;
    PeripheralState currentMotorState, currentBuzzerState, currentMotorToggleButtonLedState;
    ButtonState motorToggleButtonState;
    ButtonState configButtonState;
    uint64_t configButtonPressStartTime; // microseconds
    TimerHandle_t configButtonTimer;
    bool configButtonLongPressTriggered;

    std::function<void()> onConfigButtonClickedListener;
    std::function<void()> onConfigButtonLongPressedListener;

    function<void()> onMotorToggleButtonPressedListener;
    BuzzerControl buzzer;
    
    SwitchingUnitManager* _switchingUnitManager;
    
    friend void gpioEventHandlerTask(void *args);
    friend void config_button_timer_callback(TimerHandle_t xTimer);

public:
    void initPeripherals(SwitchingUnitManager* switchingUnitManager);
    PeripheralState getCurrentMotorState();
    PeripheralState getCurrentBuzzerState();
    PeripheralState getCurrentMotorToggleButtonLedState();
    void setMotorState(PeripheralState newMotorState);
    void setBuzzerState(PeripheralState newBuzzerState);
    void setMotorToggleButtonLedState(PeripheralState ledState);

    void enableRFModuleProgramMode();
    void disableRFModuleProgramMode();
    void displayTwoDigitNumber(uint8_t number);

    void setOnMotorToggleButtonPressedListener(function<void()> onMotorToggleButtonPressedListener);
    void setOnConfigButtonClickedListener(function<void()> onConfigButtonClickedListener);
    void setOnConfigButtonLongPressedListener(function<void()> onConfigButtonLongPressedListener);
    bool isConfigButtonPressed();

    PeripheralManager()
    {
        this->currentMotorState = OFF;
        this->currentBuzzerState = OFF;
        this->currentMotorToggleButtonLedState = OFF;
        this->peripheralsConfigured = false;
        this->onMotorToggleButtonPressedListener = NULL;
        this->motorToggleButtonState = RELEASED;
        this->configButtonState = RELEASED;
        this->configButtonPressStartTime = 0;
        this->onConfigButtonClickedListener = NULL;
        this->onConfigButtonLongPressedListener = NULL;
        this->configButtonTimer = NULL;
        this->configButtonLongPressTriggered = false;
        this->_switchingUnitManager = NULL;
    }

    // Destructor to clean up timer resources (Requirement 6.4)
    ~PeripheralManager()
    {
        if (this->configButtonTimer != NULL)
        {
            xTimerStop(this->configButtonTimer, portMAX_DELAY);
            xTimerDelete(this->configButtonTimer, portMAX_DELAY);
            this->configButtonTimer = NULL;
        }
    }
};

#endif