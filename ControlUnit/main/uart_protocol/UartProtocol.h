#ifndef UART_PROTOCOL_H_
#define UART_PROTOCOL_H_

#include <stdint.h>
#include <stddef.h>

// Protocol version
#define UART_PROTOCOL_VERSION 1

// Packet types (Phase 1: No encryption)
typedef enum {
    PACKET_TYPE_SENSOR_DATA = 0x01,     // Sensor data packet
    // Reserved for Phase 2 (Security):
    // PACKET_TYPE_HANDSHAKE_REQ = 0x02,
    // PACKET_TYPE_HANDSHAKE_RESP = 0x03,
    // PACKET_TYPE_ENCRYPTED_DATA = 0x04
} PacketType;

// Protocol constants (Phase 1: No encryption)
#define CRC32_SIZE 4                    // CRC32 checksum size
#define PACKET_DELIMITER 0x00           // COBS delimiter

// Buffer sizes (Phase 1: No encryption)
#define MAX_SENSOR_DATA_SIZE 32         // Maximum sensor data payload
#define MAX_PACKET_SIZE 64              // Maximum packet size before COBS encoding
#define MAX_ENCODED_SIZE 80             // Maximum size after COBS encoding

// Timeouts (Phase 1: No handshake)
#define PACKET_TIMEOUT_MS 200           // Packet assembly timeout
#define DATA_TIMEOUT_MS 1000            // Data reception timeout (2x transmission interval)

// FSM States
typedef enum {
    FSM_STATE_SYNC,                     // Waiting for delimiter
    FSM_STATE_COLLECT,                  // Collecting packet bytes
    FSM_STATE_PROCESS                   // Processing complete packet
} FSMState;

// Security states (Phase 2 - Reserved for future use)
// typedef enum {
//     SECURITY_STATE_INIT,
//     SECURITY_STATE_WAIT_HANDSHAKE,
//     SECURITY_STATE_HANDSHAKE_IN_PROGRESS,
//     SECURITY_STATE_SECURED
// } SecurityState;

// Packet structures (Phase 1: Simple data packet)
#pragma pack(push, 1)

typedef struct {
    uint8_t type;                       // PACKET_TYPE_SENSOR_DATA
    uint8_t payload[MAX_SENSOR_DATA_SIZE];
} DataPacket;

#pragma pack(pop)

// Phase 2 packet structures (Reserved for future use)
// typedef struct {
//     uint8_t type;
//     uint8_t publicKey[32];
//     uint8_t crc32[4];
// } HandshakeRequestPacket;
//
// typedef struct {
//     uint8_t type;
//     uint8_t iv[16];
//     uint8_t encryptedPayload[64];
//     uint8_t crc32[4];
// } EncryptedDataPacket;

// Protocol statistics (Phase 1: No encryption stats)
typedef struct {
    uint32_t packetsReceived;
    uint32_t packetsSent;
    uint32_t crcFailures;
    uint32_t cobsDecodeFailures;
    uint32_t timeouts;
    // Phase 2 stats (Reserved):
    // uint32_t decryptionFailures;
    // uint32_t handshakeAttempts;
    // uint32_t handshakeSuccesses;
} ProtocolStats;

#endif // UART_PROTOCOL_H_
