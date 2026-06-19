#include "I2CManager.h"
#include "esp_log.h"

#define TAG "I2CManager"

#define DS1307_REG_SECONDS 0x00
#define DS1307_REG_CONTROL 0x07

I2CManager::I2CManager()
{
    this->i2cInitialized = false;
    this->bus_handle = NULL;
    this->dev_handle = NULL;
}

esp_err_t I2CManager::init()
{
    if (this->i2cInitialized)
        return ESP_OK;

    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = DS1307_I2C_PORT;
    bus_config.sda_io_num = DS1307_SDA_PIN;
    bus_config.scl_io_num = DS1307_SCL_PIN;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    esp_err_t err = i2c_new_master_bus(&bus_config, &this->bus_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = DS1307_I2C_ADDRESS;
    dev_cfg.scl_speed_hz = DS1307_I2C_FREQ_HZ;

    err = i2c_master_bus_add_device(this->bus_handle, &dev_cfg, &this->dev_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add RTC device: %s", esp_err_to_name(err));
        i2c_del_master_bus(this->bus_handle);
        this->bus_handle = NULL;
        return err;
    }

    // Optional: Check and clear CH bit if set (DS1307 specific)
    uint8_t write_buffer[1] = {DS1307_REG_SECONDS};
    uint8_t sec_reg = 0;
    err = i2c_master_transmit_receive(this->dev_handle, write_buffer, 1, &sec_reg, 1, 1000);

    if (err == ESP_OK && (sec_reg & 0x80))
    { // CH bit is set
        ESP_LOGI(TAG, "DS1307 clock halt bit set. Clearing it.");
        uint8_t write_buf[2] = {DS1307_REG_SECONDS, (uint8_t)(sec_reg & 0x7F)};
        err = i2c_master_transmit(this->dev_handle, write_buf, 2, 1000);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to clear DS1307 CH bit: %s", esp_err_to_name(err));
        }
    }
    else if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read DS1307 seconds register: %s", esp_err_to_name(err));
    }

    this->i2cInitialized = true;
    return ESP_OK;
}

bool I2CManager::isRTCConnected()
{
    if (!this->i2cInitialized || !this->dev_handle)
        return false;

    uint8_t dummy;
    esp_err_t ret = i2c_master_receive(this->dev_handle, &dummy, 1, 20);
    return (ret == ESP_OK);
}

uint8_t I2CManager::bcd2bin(uint8_t val)
{
    return (val & 0x0F) + ((val >> 4) * 10);
}

uint8_t I2CManager::bin2bcd(uint8_t val)
{
    return ((val / 10) << 4) | (val % 10);
}

bool I2CManager::readTime(struct tm &timeinfo)
{
    if (!this->i2cInitialized || !this->dev_handle)
        return false;

    uint8_t write_buffer[1] = {0x00}; // DS1307 register pointer
    uint8_t read_buffer[7] = {0};

    esp_err_t ret = i2c_master_transmit_receive(this->dev_handle, write_buffer, 1, read_buffer, 7, 50);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read RTC registers: %s", esp_err_to_name(ret));
        return false;
    }

    timeinfo.tm_sec = bcd2bin(read_buffer[0] & 0x7F);
    timeinfo.tm_min = bcd2bin(read_buffer[1]);
    timeinfo.tm_hour = bcd2bin(read_buffer[2] & 0x3F); // 24-hour mode assumed
    timeinfo.tm_mday = bcd2bin(read_buffer[4]);
    timeinfo.tm_mon = bcd2bin(read_buffer[5]) - 1;     // tm_mon is 0-based
    timeinfo.tm_year = bcd2bin(read_buffer[6]) + 100;  // Years since 1900, DS1307 stores 0-99 (2000-2099)
    timeinfo.tm_wday = bcd2bin(read_buffer[3]);
    timeinfo.tm_isdst = 0;

    return true;
}

bool I2CManager::setTime(const struct tm &timeinfo)
{
    if (!this->i2cInitialized || !this->dev_handle)
        return false;

    uint8_t buffer[8];
    buffer[0] = 0x00; // register address pointer
    buffer[1] = bin2bcd(timeinfo.tm_sec);
    buffer[2] = bin2bcd(timeinfo.tm_min);
    buffer[3] = bin2bcd(timeinfo.tm_hour);
    buffer[4] = bin2bcd(timeinfo.tm_wday);
    buffer[5] = bin2bcd(timeinfo.tm_mday);
    buffer[6] = bin2bcd(timeinfo.tm_mon + 1);
    buffer[7] = bin2bcd((timeinfo.tm_year + 1900) % 100);

    esp_err_t ret = i2c_master_transmit(this->dev_handle, buffer, sizeof(buffer), 50);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write RTC registers: %s", esp_err_to_name(ret));
        return false;
    }
    return true;
}