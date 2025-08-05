#include "alpha_5_hardware/alpha_5_driver.h"
#include <stdio.h>

int main() {
    struct init_ ctx = init("/dev/ttyUSB0");
    if (!ctx.libhandle || !ctx.encode_func || !ctx.decode_func || ctx.serial_fd < 0) {
        fprintf(stderr, "Initialization failed.\n");
        return 1;
    }

    int device = 0x01;
    float position = 5.0f;
    int packet_id = 0x03;

    sendPosition(&ctx, device, position, 1, 100);
    request(&ctx, device, packet_id, 1, 50);

    return 0;
}

// gcc -I../include -o test test.c alpha_5_driver.c -ldl && ./test