#include <stdio.h>
#include <stdint.h>
#include <dlfcn.h>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <sys/time.h>

#include "alpha_5_hardware/read_master_arm.h"

#define PORT "/dev/ttyUSB0"


void* load_rs_protocol_library() {
    // const char* lib_path = "/home/michele/reach_ws/src/reach_robotics_sdk/rs_protocol/lib/librs_protocol_linux_x86_64.so";
    const char* lib_path = "../../../rs_protocol/lib/librs_protocol_linux_x86_64.so";

    void* libhandle = dlopen(lib_path, RTLD_LAZY);
    if (!libhandle) {
        fprintf(stderr, "Error loading library '%s': %s\n", lib_path, dlerror());
        return NULL;
    }

    return libhandle;
}

coms_encodePacket_fn load_coms_encodePacket(void* libhandle) {
    dlerror();  // Clear existing errors

    coms_encodePacket_fn encode_func = (coms_encodePacket_fn)dlsym(libhandle, "coms_encodePacket");
    char* error = dlerror();
    if (error != NULL) {
        fprintf(stderr, "Error finding function coms_encodePacket: %s\n", error);
        return NULL;
    }
    return encode_func;
}

coms_decodePacket_fn load_coms_decodePacket(void* libhandle) {
    dlerror();  // Clear existing errors

    coms_decodePacket_fn decode_func = (coms_decodePacket_fn)dlsym(libhandle, "coms_decodePacket");
    char* error = dlerror();
    if (error != NULL) {
        fprintf(stderr, "Error finding function coms_decodePacket: %s\n", error);
        return NULL;
    }
    return decode_func;
}


int open_serial_port(const char* device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        fprintf(stderr, "Error opening %s: %s\n", device, strerror(errno));
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "Error getting attributes: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, BAUDRATE);
    cfsetispeed(&tty, BAUDRATE);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                         // disable break processing
    tty.c_lflag = 0;                                // no signaling chars, no echo
    tty.c_oflag = 0;                                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;                            // read doesn't block
    tty.c_cc[VTIME] = 10;                           // 1 second read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls
    tty.c_cflag &= ~(PARENB | PARODD);              // no parity
    tty.c_cflag &= ~CSTOPB;                         // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                        // no hardware flow control

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "Error setting attributes: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    return fd;
}

ssize_t write_serial_data(int fd, const uint8_t* data, size_t length) {
    ssize_t bytes_written = write(fd, data, length);
    if (bytes_written < 0) {
        fprintf(stderr, "Error writing to serial port: %s\n", strerror(errno));
    }
    return bytes_written;
}

ssize_t read_serial_data(int fd, uint8_t *buffer, size_t max_length) {
    ssize_t bytes_read = read(fd, buffer, max_length);

    if (bytes_read < 0) {
        perror("Error reading from serial port");
    }

    return bytes_read;
}

bool extract_packet_from_buffer(uint8_t* buffer, size_t* buffer_len, uint8_t* packet_out, size_t* packet_len) {
    // Extract packets from buffer using COBS
    size_t i;
    for (i = 0; i < *buffer_len; ++i) {
        if (buffer[i] == 0x00) {
            break;
        }
    }

    if (i == *buffer_len) {
        // No full packet (delimiter not found)
        return false;
    }

    if (i == 0) {
        // Skip stray 0x00 
        memmove(buffer, buffer + 1, *buffer_len - 1);
        (*buffer_len)--;
        return false;
    }

    memcpy(packet_out, buffer, i);
    packet_out[i] = 0x00;
    *packet_len = i + 1;

    // printf("Raw COBS packet: ");
    // for (size_t j = 0; j < i; j++) {
    //     printf("\\x%02X", packet_out[j]);
    // }
    // printf("\n");

    // Shift remaining bytes in buffer (i + 1 to skip the delimiter)
    memmove(buffer, buffer + i + 1, *buffer_len - (i + 1));
    *buffer_len -= (i + 1);

    return true;
}

void sleepExec(int millisec) {
    struct timespec ts;
    ts.tv_sec = 0;         
    ts.tv_nsec = millisec * 1000000L; 
    nanosleep(&ts, NULL);
}

struct driver_context alpha_5_driver_init(const char* serial_device){

    struct driver_context ctx = {0};

    ctx.libhandle = load_rs_protocol_library();
    if(!ctx.libhandle) return ctx;

    ctx.encode_func = load_coms_encodePacket(ctx.libhandle);
    if (!ctx.encode_func) {
        dlclose(ctx.libhandle);
        ctx.libhandle = NULL;
        return ctx;
    }

    ctx.decode_func = load_coms_decodePacket(ctx.libhandle);
    if (!ctx.decode_func) {
        dlclose(ctx.libhandle);
        ctx.libhandle = NULL;
        return ctx;
    }

    ctx.serial_fd = open_serial_port(serial_device);
    if (ctx.serial_fd < 0) {
        dlclose(ctx.libhandle);
        ctx.libhandle = NULL;
        ctx.encode_func = NULL;
        ctx.decode_func = NULL;
    }

    return ctx;
}

long get_time_millis() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000L) + (tv.tv_usec / 1000L);
}


void readMasterArm(struct driver_context* ctx) {
    if (!ctx || !ctx->encode_func || ctx->serial_fd < 0) {
        fprintf(stderr, "Invalid context passed to request\n");
        return;
    }

    uint8_t serial_buffer[SERIAL_BUFFER_SIZE] = {0}; 
    size_t serial_buffer_len = 0;                    

    while (1) {
        uint8_t response[256] = {0};
        ssize_t received = read_serial_data(ctx->serial_fd, response, sizeof(response));

        if (received <= 0) {
            fprintf(stderr, "No data received\n");
            usleep(1000);
            continue;
        }

        if (serial_buffer_len + received <= SERIAL_BUFFER_SIZE) {
            memcpy(serial_buffer + serial_buffer_len, response, received);
            serial_buffer_len += received;

            uint8_t extracted_packet[64];
            size_t packet_len;

            while (extract_packet_from_buffer(serial_buffer, &serial_buffer_len, extracted_packet, &packet_len)) {
                // Print extracted packet
                printf("Extracted packet (%zu bytes): ", packet_len);
                for (size_t i = 0; i < packet_len; i++) {
                    printf("\\x%02X", extracted_packet[i]);
                }
                printf("\n");

                struct packet decodedResponse = {0};
                int8_t decode_status = ctx->decode_func(&decodedResponse, extracted_packet, packet_len);

                if (decode_status != 1) {
                    fprintf(stderr, "Error decoding packet: %d\n", decode_status);
                    continue;
                }

                // Print decoded data (excluding CRC if applicable)
                printf("Decoded data: ");
                float dataValue = 0.0f;
                memcpy(&dataValue, decodedResponse.data, sizeof(float));
                printf("    Address: 0x%02X\n", decodedResponse.address);
                printf("    Code:   0x%X\n", decodedResponse.code);
                for (int i = 0; i < decodedResponse.length - 4; i++) {
                    printf("\\x%02X", decodedResponse.data[i]);
                }
                printf("    = %f\n", dataValue);
            }
        } else {
            fprintf(stderr, "Serial buffer overflow, clearing\n");
            serial_buffer_len = 0;
        }

        usleep(1000);
    }
}



int main() {

    struct driver_context ctx = alpha_5_driver_init(PORT);
    if (!ctx.libhandle || !ctx.encode_func || !ctx.decode_func || ctx.serial_fd < 0) {
        fprintf(stderr, "Initialization failed.\n");
        return 1;
    }
    readMasterArm(&ctx);

    close(ctx.serial_fd);
    return 0;

}

// gcc -o readMasterArm read_master_arm.c -ldl && ./readMasterArm

