#include "alpha_5_hardware/alpha_5_driver.h"
#include <stdio.h>
#include "alpha_5_hardware/packetID.h"

#define PORT "/dev/ttyUSB0"

int main() {
    struct driver_context ctx = init(PORT);
    if (!ctx.libhandle || !ctx.encode_func || !ctx.decode_func || ctx.serial_fd < 0) {
        fprintf(stderr, "Initialization failed.\n");
        return 1;
    }

    uint8_t device = 0x03;
    float position = 5.0f;
    uint16_t packet_id = POSITION;

    // sendPosition(&ctx, device, position, 100);
    request(&ctx, device, packet_id, 50, 2, 3);
    
    // close serial_fd
    close(ctx.serial_fd);
    return 0;
}

// gcc -I../include -o test test.c alpha_5_driver.c -ldl && ./test