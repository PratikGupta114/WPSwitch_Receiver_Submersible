/**
 * @file TaskConfig.h
 * @brief Centralized task stack size configuration for heap memory optimization
 *
 * This header provides optimized task stack sizes based on measured high water marks
 * with 20-30% safety margin. These values are part of the heap memory optimization
 * effort to recover 80-100KB of heap memory for OTA operations.
 *
 * Stack sizes are determined from runtime measurements using uxTaskGetStackHighWaterMark()
 * and include appropriate safety margins for worst-case scenarios.
 *
 * @see .kiro/specs/heap-memory-optimization/requirements.md - Requirement 2
 * @see .kiro/specs/heap-memory-optimization/design.md - Task Stack Size Constants
 */

#ifndef TASK_CONFIG_H
#define TASK_CONFIG_H

// ============================================================================
// Optimized Task Stack Sizes (in bytes)
// Based on measured high water marks with 20-30% safety margin
// ============================================================================

/**
 * @brief LED indication task stack size
 * REBALANCED: Reduced from 1792 to 1536 bytes
 * Measured: 800 bytes used, HWM 992 bytes free (124% margin)
 * New allocation: 1536 bytes (800 used + 736 margin = 92% safety)
 * Savings: 256 bytes
 * Validates: Requirements 2.3
 */
#define LED_TASK_STACK_SIZE_OPTIMIZED           1536

/**
 * @brief GPIO event handler task stack size
 * CRITICAL FIX: Massively over-provisioned, using only 48 bytes of 2304
 * Measured: 48 bytes used, HWM 2256 bytes free (4700% margin!)
 * New allocation: 512 bytes (48 used + 464 margin = 967% safety)
 * Savings: 1792 bytes → redistributed to critical network tasks
 * Validates: Requirements 2.1
 */
#define GPIO_EVENT_HANDLER_STACK_SIZE           512

/**
 * @brief UART receiver task stack size
 * Original: 3072 bytes (UART_MANAGER_TASK_STACK_DEPTH in UartManager.h)
 * Rebalanced: 2048 -> 2560 bytes (+512 for OTA safety)
 * HWM during OTA: 568 bytes -> target ~1080 bytes
 * Validates: Requirements 2.6
 */
#define UART_RECEIVER_TASK_STACK_SIZE           2560

/**
 * @brief UART timeout task stack size
 * Original: 3072 bytes (UART_TIMEOUT_TASK_STACK_DEPTH in UartManager.h)
 * Rebalanced: 1536 -> 2048 bytes (+512 for OTA safety)
 * HWM during OTA: 604 bytes -> target ~1116 bytes
 * Validates: Requirements 2.2
 */
#define UART_TIMEOUT_TASK_STACK_SIZE            2048

/**
 * @brief MQTT command processor task stack size
 * CRITICAL FIX: Network/TLS task with only 22% margin (DANGEROUS)
 * Measured: 6284 bytes used, HWM 1396 bytes free (insufficient for TLS)
 * New allocation: 10240 bytes (6284 used + 3956 margin = 63% safety)
 * Increase: +2560 bytes (meets 50% minimum for network tasks)
 * Validates: Requirements 2.4
 */
#define MQTT_CMD_PROCESSOR_STACK_SIZE_OPTIMIZED 10240

/**
 * @brief MQTT publisher task stack size
 * CRITICAL FIX: Network/TLS task with only 33% margin (RISKY)
 * Measured: 3068 bytes used, HWM 1028 bytes free (insufficient for TLS spikes)
 * New allocation: 5120 bytes (3068 used + 2052 margin = 67% safety)
 * Increase: +1024 bytes
 * Validates: Requirements 2.5
 */
#define MQTT_PUBLISHER_STACK_SIZE_OPTIMIZED     5120

/**
 * @brief SNTP time sync task stack size
 * REBALANCED: Reduced from 2816 to 2304 bytes
 * Measured: 1936 bytes used, HWM 880 bytes free (45% margin)
 * New allocation: 2304 bytes (1936 used + 368 margin = 19% safety)
 * Savings: 512 bytes (acceptable for background task)
 * Validates: Requirements 2.7
 */
#define SNTP_TASK_STACK_SIZE_OPTIMIZED          2304

/**
 * @brief WiFi reconnection task stack size
 * CRITICAL FIX: Network task with only 40% margin (BORDERLINE)
 * Measured: 1096 bytes used, HWM 440 bytes free (risky for network operations)
 * New allocation: 2048 bytes (1096 used + 952 margin = 87% safety)
 * Increase: +512 bytes
 * Validates: Requirements 2.8
 */
#define WIFI_RECONNECT_TASK_STACK_SIZE          2048

/**
 * @brief Restart task stack size
 * Original: 2048 bytes (RESTART_TASK_STACK_DEPTH in main.cc)
 * Optimized: 1536 bytes
 * Validates: Requirements 2.9
 */
#define RESTART_TASK_STACK_SIZE                 1536

// ============================================================================
// Tasks that retain original sizes (safety-critical or TLS requirements)
// ============================================================================

/**
 * @brief OTA task stack size
 * CRITICAL FIX: TLS task with only 18% margin (DANGEROUS for OTA success)
 * Measured: 6944 bytes used, HWM 1248 bytes free (insufficient for TLS handshake)
 * New allocation: 10240 bytes (6944 used + 3296 margin = 47% safety)
 * Increase: +2048 bytes (CRITICAL for OTA reliability)
 */
#define OTA_TASK_STACK_SIZE                     10240

#endif /* TASK_CONFIG_H */
