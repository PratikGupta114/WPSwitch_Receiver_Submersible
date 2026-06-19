#ifndef I2CMANAGER_H_
#define I2CMANAGER_H_

#include <ctime>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "PeripheralManager.h"

// DS1307 RTC I2C configuration

#define DS1307_I2C_PORT I2C_NUM_0
#define DS1307_I2C_FREQ_HZ 100000 // 100 kHz standard mode
#define DS1307_I2C_ADDRESS 0x68   // 7-bit address

class I2CManager
{
private:
    bool i2cInitialized;
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;

    static uint8_t bcd2bin(uint8_t val);
    static uint8_t bin2bcd(uint8_t val);

public:
    I2CManager();

    /**
     * @brief Initialize I2C peripheral.
     */
    esp_err_t init();

    /**
     * @brief Check whether the DS1307 RTC responds on the bus.
     */
    bool isRTCConnected();

    /**
     * @brief Read current date & time from DS1307.
     *
     * @param[out] timeinfo Filled with current RTC time on success.
     * @return true on success, false otherwise.
     */
    bool readTime(struct tm &timeinfo);

    /**
     * @brief Set DS1307 date & time.
     *
     * @param[in] timeinfo Time structure to write.
     * @return true on success, false otherwise.
     */
    bool setTime(const struct tm &timeinfo);
};

#endif // I2CMANAGER_H_ 