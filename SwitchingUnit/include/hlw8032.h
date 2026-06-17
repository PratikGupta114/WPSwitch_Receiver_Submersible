#ifndef HLW8032_H
#define HLW8032_H

#include <stdint.h>

/**
 * Parsed HLW8032 data — raw register values.
 *
 * The HLW8032 sends a 24-byte frame every 50 ms at 4800 baud:
 *   [0]     State register
 *   [1]     Check byte (always 0x5A)
 *   [2-4]   Voltage parameter (coefficient)
 *   [5-7]   Voltage data (proportional to Vrms)
 *   [8-10]  Current parameter (coefficient)
 *   [11-13] Current data (proportional to Irms)
 *   [14-16] Power parameter (coefficient)
 *   [17-19] Power data (proportional to active power)
 *   [20]    Data update flag
 *   [21-22] PF count
 *   [23]    Checksum (sum of bytes 2-22, mod 256)
 */
#ifndef FEATURE_HLW8032
#define FEATURE_HLW8032 1  /* Default to enabled if not defined */
#endif

#if FEATURE_HLW8032

typedef struct {
    uint8_t  valid;              /**< 1 if at least one valid packet received */
    uint8_t  state_reg;          /**< HLW8032 state register                 */
    uint8_t  voltage_reg[3];     /**< Voltage parameter (coefficient)        */
    uint8_t  voltage_data[3];    /**< Voltage measurement data               */
    uint8_t  current_reg[3];     /**< Current parameter (coefficient)        */
    uint8_t  current_data[3];    /**< Current measurement data               */
    uint8_t  power_reg[3];       /**< Power parameter (coefficient)          */
    uint8_t  power_data[3];      /**< Power measurement data                 */
    uint8_t  data_update;        /**< Data update flag                       */
    uint8_t  pf_count[2];       /**< PF count register (energy pulses)      */
    uint32_t packet_count;       /**< Total valid packets received           */
} hlw8032_data_t;

/** @brief Initialize HLW8032 driver and software UART. */
void hlw8032_init(void);

/**
 * @brief Poll for new HLW8032 data (call from main loop).
 *
 * If a full 24-byte packet is received and validates, the internal
 * data structure is updated.
 */
void hlw8032_tick(void);

/**
 * @brief Get pointer to the latest parsed HLW8032 data.
 * @return Pointer to static data (valid across calls, updated by tick).
 */
const hlw8032_data_t *hlw8032_get_data(void);

#else

/* Dummy inline functions when HLW8032 is disabled */
static inline void hlw8032_init(void) {}
static inline void hlw8032_tick(void) {}

#endif

#endif /* HLW8032_H */
