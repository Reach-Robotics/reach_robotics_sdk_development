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


#define SERIAL_BUFFER_SIZE 1024

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

// Function declarations
int open_serial_port(const char* device, speed_t baudrate);
typedef int8_t (*coms_decodePacket_fn)(struct packet* dest_packet, uint8_t* src_buffer, uint8_t src_length);


int open_serial_port(const char* device, speed_t baudrate) {
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

    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

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

coms_decodePacket_fn load_coms_decodePacket(void* libhandle) {
    dlerror();  // Clear existing errors

    coms_decodePacket_fn decode_func = (coms_decodePacket_fn)dlsym(libhandle, "coms_decodePacket");
    char* error = dlerror();
    if (error != NULL) {
        fprintf(stderr, "Error finding function coms_decodePacket: %s\n", error);
        return NULL;
    }

    printf("Successfully loaded function coms_decodePacket.\n");
    return decode_func;
}

void* load_rs_protocol_library() {
    const char* lib_path = "/home/michele/reach_ws/src/reach_robotics_sdk/rs_protocol/lib/librs_protocol_linux_x86_64.so";

    void* libhandle = dlopen(lib_path, RTLD_LAZY);
    if (!libhandle) {
        fprintf(stderr, "Error loading library '%s': %s\n", lib_path, dlerror());
        return NULL;
    }

    return libhandle;
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

void sleepExec(int millisec) {
    struct timespec ts;
    ts.tv_sec = 0;         
    ts.tv_nsec = millisec * 1000000L; 
    nanosleep(&ts, NULL);
}


bool extract_packet_from_buffer(uint8_t* buffer, size_t* buffer_len, uint8_t* packet_out, size_t* packet_len) {
    // Look for the delimiter (0x00), which marks the end of a COBS packet
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
        // Skip stray 0x00 (could happen in noisy lines)
        memmove(buffer, buffer + 1, *buffer_len - 1);
        (*buffer_len)--;
        return false;
    }

    // Copy the packet (excluding the delimiter)
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


int main() {
    const char* device_path = "/dev/ttyUSB0";  // Replace with your actual device path

    void* libhandle = load_rs_protocol_library();
    if (!libhandle) return 1;

    coms_decodePacket_fn decode_func = load_coms_decodePacket(libhandle);
    if (!decode_func) {
        dlclose(libhandle);
        return 1;
    }

    int serial = open_serial_port(device_path, B115200);

    if (serial < 0) {
        fprintf(stderr, "Failed to open serial port\n");
        return 1;
    }
    uint8_t serial_buffer[SERIAL_BUFFER_SIZE] = {0};
    size_t serial_buffer_len = 0;
    
    while (1) {
        
        uint8_t response[256] = {0};
        ssize_t received = read_serial_data(serial, response, sizeof(response));
        
        if (received <= 0) {
            fprintf(stderr, "No data received\n");
            continue;
        }
        
        if (serial_buffer_len + received <= SERIAL_BUFFER_SIZE) {
            memcpy(serial_buffer + serial_buffer_len, response, received);
            serial_buffer_len += received;

            // Try extract and decode packet
            uint8_t extracted_packet[64];
            size_t packet_len;

            while (extract_packet_from_buffer(serial_buffer, &serial_buffer_len, extracted_packet, &packet_len)) {

                float dataValue = 0.0f;
                struct packet decodedResponse;
                memset(&decodedResponse, 0, sizeof(decodedResponse));

                int8_t decode_status = decode_func(&decodedResponse, extracted_packet, packet_len);
                if (decode_status != 1) {
                    fprintf(stderr, "Error decoding packet: %d\n", decode_status);
                    continue;
                }

                printf("  Address: 0x%02X\n", decodedResponse.address);
                printf("  Code:    0x%X\n", decodedResponse.code);
                
                for (int i = 0; i < decodedResponse.length - 4; i++) {
                    printf("\\x%02X", decodedResponse.data[i]);
                }
                printf("\n");

                memcpy(&dataValue, decodedResponse.data, sizeof(float));
                printf("Float value %f\n", dataValue);
            }
        }
    }

    dlclose(libhandle);

    return 0;
}

// gcc -o readMasterArm read_master_arm.c -ldl && ./readMasterArm

