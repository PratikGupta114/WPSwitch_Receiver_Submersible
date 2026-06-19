/**
 * @file HeapMonitor.cc
 * @brief Implementation of heap and stack monitoring utilities
 * 
 * Requirements: 1.1, 1.2, 1.3, 1.4, 1.5, 1.6
 */

#include "HeapMonitor.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "HeapMonitor";

namespace HeapMonitor {

void logRestartReason()
{
    esp_reset_reason_t reason = esp_reset_reason();
    const char *reasonStr;
    
    switch (reason) {
        case ESP_RST_UNKNOWN:
            reasonStr = "Unknown";
            break;
        case ESP_RST_POWERON:
            reasonStr = "Power-on";
            break;
        case ESP_RST_EXT:
            reasonStr = "External pin";
            break;
        case ESP_RST_SW:
            reasonStr = "Software reset (esp_restart)";
            break;
        case ESP_RST_PANIC:
            reasonStr = "Exception/panic";
            break;
        case ESP_RST_INT_WDT:
            reasonStr = "Interrupt watchdog";
            break;
        case ESP_RST_TASK_WDT:
            reasonStr = "Task watchdog";
            break;
        case ESP_RST_WDT:
            reasonStr = "Other watchdog";
            break;
        case ESP_RST_DEEPSLEEP:
            reasonStr = "Deep sleep wake";
            break;
        case ESP_RST_BROWNOUT:
            reasonStr = "Brownout";
            break;
        case ESP_RST_SDIO:
            reasonStr = "SDIO";
            break;
        default:
            reasonStr = "Unhandled";
            break;
    }
    
    ESP_LOGW(TAG, "Restart reason: %s (code: %d)", reasonStr, (int)reason);
    
    // Log additional warning for abnormal restarts
    if (reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT || 
        reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT ||
        reason == ESP_RST_BROWNOUT) {
        ESP_LOGE(TAG, "ABNORMAL RESTART DETECTED - investigate cause!");
    }
}

void logHeapStatus()
{
    size_t freeHeap = esp_get_free_heap_size();
    size_t minFreeHeap = esp_get_minimum_free_heap_size();
    size_t largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    
    ESP_LOGI(TAG, "Heap: free=%u, min_free=%u, largest_block=%u bytes",
             (unsigned int)freeHeap, 
             (unsigned int)minFreeHeap, 
             (unsigned int)largestBlock);
    
    // Critical warning when free heap is below threshold (Requirement 1.6)
    if (freeHeap < HEAP_WARNING_THRESHOLD_BYTES) {
        ESP_LOGE(TAG, "CRITICAL: Free heap (%u bytes) below threshold (%u bytes)!",
                 (unsigned int)freeHeap, 
                 (unsigned int)HEAP_WARNING_THRESHOLD_BYTES);
    }
}

void getHeapInfo(size_t& freeHeap, size_t& minFreeHeap, size_t& largestBlock)
{
    freeHeap = esp_get_free_heap_size();
    minFreeHeap = esp_get_minimum_free_heap_size();
    largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
}

#if WATERMARK_LOGGING_ENABLED

/**
 * @brief Check if stack utilization exceeds warning threshold
 * 
 * @param usedBytes Stack bytes used
 * @param totalBytes Total stack size in bytes
 * @return true if utilization exceeds STACK_UTILIZATION_WARNING_PERCENT
 */
static bool isStackUtilizationHigh(uint32_t usedBytes, uint32_t totalBytes)
{
    if (totalBytes == 0) return false;
    uint32_t utilizationPercent = (usedBytes * 100) / totalBytes;
    return utilizationPercent >= STACK_UTILIZATION_WARNING_PERCENT;
}

void logTaskStackWatermarks()
{
    // Get list of all tasks
    UBaseType_t taskCount = uxTaskGetNumberOfTasks();
    TaskStatus_t *taskStatusArray = (TaskStatus_t *)malloc(taskCount * sizeof(TaskStatus_t));
    
    if (taskStatusArray == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for task status array");
        return;
    }
    
    UBaseType_t actualTaskCount = uxTaskGetSystemState(taskStatusArray, taskCount, NULL);
    
    ESP_LOGI(TAG, "=== Task Stack Watermarks (%u tasks) ===", (unsigned int)actualTaskCount);
    
    for (UBaseType_t i = 0; i < actualTaskCount; i++) {
        TaskStatus_t *task = &taskStatusArray[i];
        
        // Get high water mark (minimum free stack since task creation)
        // High water mark is in words (4 bytes on ESP32)
        UBaseType_t highWaterMark = task->usStackHighWaterMark;
        uint32_t freeStackBytes = highWaterMark * sizeof(StackType_t);
        
        // Determine log level based on stack margin thresholds
        // ERROR: < 600 bytes (critical risk)
        // WARNING: < 1000 bytes (concerning)
        // INFO: >= 1000 bytes (healthy)
        if (freeStackBytes < 600) {
            ESP_LOGE(TAG, "  %-20s: HWM=%4u words (%5u bytes free) - CRITICAL LOW STACK!",
                     task->pcTaskName,
                     (unsigned int)highWaterMark,
                     (unsigned int)freeStackBytes);
        } else if (freeStackBytes < 1000) {
            ESP_LOGW(TAG, "  %-20s: HWM=%4u words (%5u bytes free) - Low stack margin",
                     task->pcTaskName,
                     (unsigned int)highWaterMark,
                     (unsigned int)freeStackBytes);
        } else {
            ESP_LOGI(TAG, "  %-20s: HWM=%4u words (%5u bytes free)",
                     task->pcTaskName,
                     (unsigned int)highWaterMark,
                     (unsigned int)freeStackBytes);
        }
    }
    
    ESP_LOGI(TAG, "========================================");
    
    free(taskStatusArray);
}

/**
 * @brief Log stack watermark for a specific task with utilization check
 * 
 * This function can be used to check individual tasks where the total
 * stack size is known, enabling proper utilization percentage calculation.
 * 
 * @param taskHandle Handle to the task to check
 * @param taskName Name of the task for logging
 * @param totalStackBytes Total stack size allocated for the task
 * 
 * Validates: Requirement 1.5
 */
void logTaskStackWatermarkWithUtilization(TaskHandle_t taskHandle, const char* taskName, uint32_t totalStackBytes)
{
    if (taskHandle == NULL || taskName == NULL) {
        return;
    }
    
    UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(taskHandle);
    uint32_t freeStackBytes = highWaterMark * sizeof(StackType_t);
    uint32_t usedStackBytes = totalStackBytes - freeStackBytes;
    uint32_t utilizationPercent = (totalStackBytes > 0) ? (usedStackBytes * 100) / totalStackBytes : 0;
    
    ESP_LOGI(TAG, "  %-20s: used=%5u/%5u bytes (%u%% utilization)",
             taskName,
             (unsigned int)usedStackBytes,
             (unsigned int)totalStackBytes,
             (unsigned int)utilizationPercent);
    
    // Warning when stack utilization exceeds threshold (Requirement 1.5)
    if (isStackUtilizationHigh(usedStackBytes, totalStackBytes)) {
        ESP_LOGW(TAG, "  WARNING: Task '%s' stack utilization (%u%%) exceeds %u%% threshold!",
                 taskName, (unsigned int)utilizationPercent, STACK_UTILIZATION_WARNING_PERCENT);
    }
}

#else // WATERMARK_LOGGING_ENABLED == 0

void logTaskStackWatermarks()
{
    // Compiled out when WATERMARK_LOGGING_ENABLED is 0
    // This function becomes a no-op
}

#endif // WATERMARK_LOGGING_ENABLED

} // namespace HeapMonitor
