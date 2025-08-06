#ifndef ALPHA_5_DRIVER_H
#define ALPHA_5_DRIVER_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>

#define SERIAL_BUFFER_SIZE 256
#define BAUDRATE 115200

struct packet {
  uint8_t length;
    uint8_t address;
    uint16_t code;
    uint16_t crc;
    uint8_t data[64];
    uint8_t transmitData[64];
    uint8_t protocol;
    uint8_t option;
    uint8_t useOption;
    uint16_t receiveRegister;
    uint8_t totalFrames;
};

typedef int8_t (*coms_encodePacket_fn)(struct packet*, uint8_t, uint16_t, uint8_t, uint8_t*, uint8_t);
typedef int8_t (*coms_decodePacket_fn)(struct packet*, uint8_t*, uint8_t);

struct driver_context {
    void* libhandle;
    coms_encodePacket_fn encode_func;
    coms_decodePacket_fn decode_func;
    int serial_fd;
};

// Function declarations
struct driver_context init(const char* serial_device);
int request(struct driver_context* ctx, uint8_t deviceID, uint8_t requestPacketID, int sleepDuration, int writeAttempts, int readAttempts);
int sendPosition(struct driver_context* ctx, uint8_t deviceID, float posData, int sleepDuration);


#endif 