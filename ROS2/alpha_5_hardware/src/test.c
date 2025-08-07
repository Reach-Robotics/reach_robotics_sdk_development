#include "alpha_5_hardware/alpha_5_driver.h"
#include <stdio.h>
#include "alpha_5_hardware/packetID.h"

#define PORT "/dev/ttyUSB0"

int main() {
    struct driver_context ctx = alpha_5_driver_init(PORT);
    if (!ctx.libhandle || !ctx.encode_func || !ctx.decode_func || ctx.serial_fd < 0) {
        fprintf(stderr, "Initialization failed.\n");
        return 1;
    }

    uint8_t device = 0x01;
    float position = 8.0f;
    uint16_t packet_id = POSITION;

    // sendPosition(&ctx, device, position, 100);
    //request(&ctx, device, packet_id, 50, 2, 3);

    uint8_t request_ids[] = {POSITION, VELOCITY, CURRENT, INTERNAL_TEMPERATURE, VOLTAGE};
    //requestPackets(&ctx, device, request_ids, 5, 50, 2, 3);
    requestPacketsLoop(&ctx, 2);
    close(ctx.serial_fd);
    return 0;
}

// gcc -I../include -o test test.c alpha_5_driver.c -ldl && ./test