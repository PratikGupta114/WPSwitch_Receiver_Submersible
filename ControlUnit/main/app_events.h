#ifndef APP_EVENTS_H_
#define APP_EVENTS_H_

#include "freertos/event_groups.h"

// Event group bits for application-level signals
#define ENTER_CONFIG_MODE_BIT    (1 << 0)
#define PUMP_MONITORING_TICK_BIT (1 << 1)
#define UART_TIMEOUT_EVENT_BIT   (1 << 2)  // UART data reception timeout event

#endif // APP_EVENTS_H_
