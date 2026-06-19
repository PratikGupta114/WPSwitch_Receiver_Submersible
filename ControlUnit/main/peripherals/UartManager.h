#ifndef UARTMANAGER_H_
#define UARTMANAGER_H_

#include <functional>
#include "driver/gpio.h"
#include "hal/uart_types.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "../board_pins.h"
#include "../data/dataTypes.h"
#include "../uart_protocol/UartProtocol.h"
#include "../uart_protocol/UartFraming.h"

// Pin connections for the RF Link Serial module

#define UART_PORT_FOR_RECEIVER WIRELESS_UART_PORT
#define RECEIVER_PORT_UART_BAUD_RATE CONFIG_RECEIVER_PORT_UART_BAUD_RATE
#define RECEIVER_PORT_RX WIRELESS_DATA_RX_PIN
#define RECEIVER_PORT_TX WIRELESS_DATA_TX_PIN
#define WIRELESS_PRG_MODE_PIN WIRELESS_PRG_PIN
#define UART_BUFFER_SIZE CONFIG_UART_BUFFER_SIZE
#define UART_MANAGER_TASK_STACK_DEPTH 3584  // Increased from 3072 - HWM was 696 bytes, rebalanced from wifi task
#define UART_TIMEOUT_TASK_STACK_DEPTH 3584  // Increased from 3072 (+512 bytes) - HWM was 164 bytes, callback does MQTT/cJSON work

#define UART_DATA_RECEPTION_TIMEOUT_SEC CONFIG_UART_DATA_RECEPTION_TIMEOUT_SEC

#define UART_TIMEOUT_BIT BIT0

using namespace std;

class UartManager
{
private:
    // FSM states for packet assembly
    enum class RxState {
        SYNC,       // Waiting for delimiter (0x00)
        COLLECT,    // Collecting encoded bytes
        PROCESS     // Processing complete packet
    };

    bool receiverPortInitialized;
    TaskHandle_t uartTimeoutTaskHandle;
    EventGroupHandle_t uartTaskEventGroup;
    friend void receiverPortDataReceptionTask(void *args);
    // This timer needs to be reset every time a "valid" sensor data is received from the uart
    TimerHandle_t dataReceptionTimerHandle;

    function<bool(char *, size_t)> onReceiverPortDataReceivedListener;
    function<void()> dataReceptionTimeoutListener;
    friend void dataReceptionTimerCallback(TimerHandle_t xTimer);

    // FSM state and buffers for new protocol
    RxState rxState;
    uint8_t rxBuffer[MAX_ENCODED_SIZE];
    size_t rxBufferIndex;
    int64_t lastByteTime;  // CRITICAL FIX: Must be int64_t to store esp_timer_get_time() (microseconds)

    void uartTimeoutTask();
    static void uart_timeout_task_wrapper(void *pvParameters);
    void processReceivedPacket(const uint8_t* data, size_t length);

public:
    bool uartInitialized;
    void initReceiverPort(function<bool(char *, size_t)> onReceiverPortDataReceivedListener);
    void sendData(char *data, size_t length);
    void resetRadioModule();

    void setDataReceptionTimeoutListener(function<void()> dataReceptionTimeoutListener);

    // RF Module Health Check
    bool isRFModuleResponding();

    // RF Module Configuration - Getters
    RFModuleAirDataRate getRFModuleAirDataRate();
    RFModuleBaudrate getRFModuleBaudrate();
    RFModuleCarrierFrequency getRFModuleCarrierFrequency();
    RFModulePowerTransmitLevel getRFModulePowerTransmitLevel();
    RFModuleSignalStrengthLimit getRFModuleSignalStrengthLimit();
    uint16_t getRFModuleHardwareID();
    uint16_t getRFModuleNetworkID();
    uint16_t getRFModuleDestinationID();
    char *getRFModule128bitEncryptionKey();

    // RF Module Configuration - Setters
    void setRFModuleAirDataRate(RFModuleAirDataRate airDataRate);
    void setRFModuleBaudrate(RFModuleBaudrate baudrate);
    void setRFModuleCarrierFrequency(RFModuleCarrierFrequency carrierFrequency);
    void setRFModulePowerTransmitLevel(RFModulePowerTransmitLevel powerTransmitLevel);
    void setRFModuleSignalStrengthLimit(RFModuleSignalStrengthLimit signalStrengthLimit);
    void setRFModuleHardwareID(uint16_t hardwareID);
    void setRFModuleNetworkID(uint16_t networkID);
    void setRFModuleDestinationID(uint16_t destinationID);
    void setRFModule128bitEncryptionKey(char *encryptionKey);

    UartManager();

    ~UartManager();
};

#endif
