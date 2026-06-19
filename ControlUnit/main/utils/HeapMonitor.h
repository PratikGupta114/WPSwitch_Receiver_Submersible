/**
 * @file HeapMonitor.h
 * @brief Heap and stack monitoring utilities for ESP32
 * 
 * Provides runtime monitoring of heap memory usage and task stack watermarks
 * to help identify memory issues and optimize allocations.
 * 
 * Requirements: 1.1, 1.2, 1.3, 1.4, 1.5, 1.6
 */

#ifndef HEAP_MONITOR_H
#define HEAP_MONITOR_H

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Macro to enable/disable watermark logging at compile time
// Set to 1 for development builds, 0 for production to reduce code size
#ifndef WATERMARK_LOGGING_ENABLED
#define WATERMARK_LOGGING_ENABLED 1
#endif

// Heap warning threshold - log critical warning when free heap drops below this
#define HEAP_WARNING_THRESHOLD_BYTES (50 * 1024)  // 50KB

// Stack utilization warning threshold - log warning when task stack usage exceeds this percentage
#define STACK_UTILIZATION_WARNING_PERCENT 80

namespace HeapMonitor {

/**
 * @brief Log the reason for the last system restart
 * 
 * Should be called early in app_main() to capture restart reason.
 * Uses esp_reset_reason() to determine cause.
 * 
 * Validates: Requirement 1.1
 */
void logRestartReason();

/**
 * @brief Log current heap memory status
 * 
 * Logs free heap, minimum free heap since boot, and largest free block.
 * Logs critical warning if free heap is below HEAP_WARNING_THRESHOLD_BYTES.
 * 
 * Validates: Requirements 1.2, 1.6
 */
void logHeapStatus();

/**
 * @brief Log stack high water marks for all application tasks
 * 
 * Only compiled when WATERMARK_LOGGING_ENABLED is set to 1.
 * Logs warning when stack utilization exceeds STACK_UTILIZATION_WARNING_PERCENT.
 * 
 * Validates: Requirements 1.3, 1.4, 1.5
 */
void logTaskStackWatermarks();

#if WATERMARK_LOGGING_ENABLED
/**
 * @brief Log stack watermark for a specific task with utilization percentage
 * 
 * Use this function when you know the total stack size allocated for a task,
 * enabling accurate utilization percentage calculation and threshold warnings.
 * 
 * @param taskHandle Handle to the task to check
 * @param taskName Name of the task for logging
 * @param totalStackBytes Total stack size allocated for the task in bytes
 * 
 * Validates: Requirement 1.5
 */
void logTaskStackWatermarkWithUtilization(TaskHandle_t taskHandle, const char* taskName, uint32_t totalStackBytes);
#endif

/**
 * @brief Get heap information for external use (e.g., MQTT publishing)
 * 
 * @param[out] freeHeap Current free heap in bytes
 * @param[out] minFreeHeap Minimum free heap since boot in bytes
 * @param[out] largestBlock Largest contiguous free block in bytes
 * 
 * Validates: Requirements 9.1, 9.2, 9.3
 */
void getHeapInfo(size_t& freeHeap, size_t& minFreeHeap, size_t& largestBlock);

} // namespace HeapMonitor

#endif // HEAP_MONITOR_H
