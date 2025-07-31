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

struct packet {
    uint8_t length;   // len(data) + 4
    uint8_t address;  // device_id
    uint16_t code;    // packet_id
    uint16_t crc;
    uint8_t data[64];
    uint8_t transmitData[64];
    uint8_t protocol;
    uint8_t option;
    uint8_t useOption;
    uint16_t receiveRegister;
    uint8_t totalFrames;
};

#define SERIAL_BUFFER_SIZE 256

// Function declarations
int open_serial_port(const char* device, speed_t baudrate);
ssize_t write_serial_data(int fd, const uint8_t* data, size_t length);

typedef int8_t (*coms_encodePacket_fn)(struct packet* packet, uint8_t address, uint16_t code, uint8_t length, uint8_t* buffer, uint8_t c_option);
typedef int8_t (*coms_decodePacket_fn)(struct packet* dest_packet, uint8_t* src_buffer, uint8_t src_length);

void* load_rs_protocol_library() {
    const char* lib_path = "/home/michele/reach_ws/src/reach_robotics_sdk/rs_protocol/lib/librs_protocol_linux_x86_64.so";

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

    printf("Successfully loaded function coms_encodePacket.\n");
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

    printf("Successfully loaded function coms_decodePacket.\n");
    return decode_func;
}


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

int loadLibFunc(){
    
}

bool extract_packet_from_buffer(uint8_t* buffer, size_t* buffer_len, uint8_t* packet_out, size_t* packet_len) {
    if (*buffer_len < 8) return false;  // Need at least 8 bytes to read buffer[7]
    uint8_t payload_len = buffer[7];
    uint8_t expected_len = 10;

    if (*buffer_len < expected_len) return false;  // Wait for full packet

    memcpy(packet_out, buffer, expected_len);
    *packet_len = expected_len;
    // Shift remaining buffer content
    memmove(buffer, buffer + expected_len, *buffer_len - expected_len);
    *buffer_len -= expected_len;

    return true;
}

void sleepExec(int millisec) {
    struct timespec ts;
    ts.tv_sec = 0;         
    ts.tv_nsec = millisec * 1000000L; 
    nanosleep(&ts, NULL);
}

int request (int deviceID, int requestPacketID, size_t length, int sleepDuration){
    // Load the shared library
    void* libhandle = load_rs_protocol_library();
    if (!libhandle) return 1;

    // Load the encode function
    coms_encodePacket_fn encode_func = load_coms_encodePacket(libhandle);
    if (!encode_func) {
        dlclose(libhandle);
        return 1;
    }

    // Load the decode function
    coms_decodePacket_fn decode_func = load_coms_decodePacket(libhandle);
    if (!decode_func) {
        dlclose(libhandle);
        return 1;
    }

    // Encode a request packet 
    struct packet myPacket;
    memset(&myPacket, 0, sizeof(myPacket)); // Initialize myPacket

    myPacket.code = 0x60;
    myPacket.address = deviceID;
    myPacket.length = length; 
    //myPacket.option = 0b111;
    int data = requestPacketID;

    // printf("Data to encode:\n");
    // printf("  Address: 0x%02X\n", myPacket.address);
    // printf("  Code:    0x%X\n", myPacket.code);
    // printf("  Length:  %d\n", myPacket.length);
    // printf("  Data:    ");
    // for (int i = 0; i < myPacket.length; i++) {
    //   printf("%f ", data);
    // }
    // printf("\n");

    uint8_t encoded_data[4];
    memcpy(encoded_data, &data, sizeof(data));
    int8_t status = encode_func(&myPacket, myPacket.address, myPacket.code, sizeof(encoded_data) + 4, encoded_data, 0);

    // Check if encoding was successful
    if (status != 1) {
        fprintf(stderr, "Error encoding packet: %d\n", status);
        dlclose(libhandle);
        return 0;
    } else {
        printf("Packet encoded successfully!\n");
        printf("Encoded values:\n");
        // Print encoded data (assuming it’s stored in myPacket.transmitData)
        printf("b'");
        for (int i = 0; i < myPacket.length; i++) {
            printf("\\x%02X", myPacket.transmitData[i]);
        }
        printf("'\n");

      } 

    int fd = open_serial_port("/dev/ttyUSB0", B115200);

    if (fd < 0) {
        dlclose(libhandle);
        return 1;
    }

  
    // Send encoded data
    ssize_t sent = write_serial_data(fd, myPacket.transmitData, myPacket.length);
    if (sent != myPacket.length) {
        fprintf(stderr, "Only wrote %zd of %d bytes to serial port\n", sent, myPacket.length);
    } else {
        printf("Successfully wrote %zd bytes to serial port\n", sent);
    }

    sleepExec(sleepDuration);
    /// NEW SECTION WITH PARSING LOGIC

    // Clear input buffer 
    tcflush(fd, TCIFLUSH);
    uint8_t serial_buffer[SERIAL_BUFFER_SIZE];
    size_t serial_buffer_len = 0;

    uint8_t response[64] = {0};
    ssize_t received = read_serial_data(fd, response, sizeof(response));
    if (received > 0) {
        printf("Received response from serial port:\n");
        for (ssize_t i = 0; i < received; i++) {
            printf("0x%02X ", response[i]);
        }
        printf("\n");
        if (serial_buffer_len + received <= SERIAL_BUFFER_SIZE) {
            memcpy(serial_buffer + serial_buffer_len, response, received);
            serial_buffer_len += received;

            // Try extracting packets
            uint8_t extracted_packet[64];
            size_t packet_len;
            printf("Serial buffer length: %zu\n", serial_buffer_len);
            while (extract_packet_from_buffer(serial_buffer, &serial_buffer_len, extracted_packet, &packet_len)) {
                printf("Extracted complete packet of length %zu:\n", packet_len);
                for (size_t i = 0; i < packet_len; i++) {
                    printf("0x%02X ", extracted_packet[i]);
                }
                printf("\n");

                float dataValue = 0.0f;
                struct packet decodedResponse;
                memset(&decodedResponse, 0, sizeof(decodedResponse));

                int8_t decode_status = decode_func(&decodedResponse, extracted_packet, packet_len);
                if (decode_status != 1) {
                    fprintf(stderr, "Error decoding packet: %d\n", decode_status);
                    continue;
                }

                printf("Decoded packet:\n");
                printf("  Address: 0x%02X\n", decodedResponse.address);
                printf("  Code:    0x%X\n", decodedResponse.code);
                printf("  Length:  %d\n", decodedResponse.length);
                printf("  Data:    b'");
                for (int i = 0; i < decodedResponse.length - 4; i++) {
                    printf("\\x%02X", decodedResponse.data[i]);
                }
                printf("  =  ");
                memcpy(&dataValue, decodedResponse.data, sizeof(float));
                printf("%f\n", dataValue);
            }
        } else {
            fprintf(stderr, "Serial buffer overflow!\n");
            serial_buffer_len = 0; // Clear to recover
        }
    }
}

int sendPosition(int deviceID, int posData, size_t length, int sleepDuration){
    // Load the shared library
    void* libhandle = load_rs_protocol_library();
    if (!libhandle) return 1;

    // Load the encode function
    coms_encodePacket_fn encode_func = load_coms_encodePacket(libhandle);
    if (!encode_func) {
        dlclose(libhandle);
        return 1;
    }

    // Load the decode function
    coms_decodePacket_fn decode_func = load_coms_decodePacket(libhandle);
    if (!decode_func) {
        dlclose(libhandle);
        return 1;
    }

    // Encode a request packet 
    struct packet myPacket;
    memset(&myPacket, 0, sizeof(myPacket)); // Initialize myPacket

    myPacket.code = 0x03;
    myPacket.address = deviceID;
    myPacket.length = length; 
    //myPacket.option = 0b111;
    float data = posData;

    // printf("Data to encode:\n");
    // printf("  Address: 0x%02X\n", myPacket.address);
    // printf("  Code:    0x%X\n", myPacket.code);
    // printf("  Length:  %d\n", myPacket.length);
    // printf("  Data:    ");
    // for (int i = 0; i < myPacket.length; i++) {
    //   printf("%f ", data);
    // }
    // printf("\n");

    uint8_t encoded_data[4];
    memcpy(encoded_data, &data, sizeof(data));
    int8_t status = encode_func(&myPacket, myPacket.address, myPacket.code, sizeof(encoded_data) + 4, encoded_data, 0);

    // Check if encoding was successful
    if (status != 1) {
        fprintf(stderr, "Error encoding packet: %d\n", status);
        dlclose(libhandle);
        return 0;
    } else {
        printf("Packet encoded successfully!\n");
        printf("Encoded values:\n");
        // Print encoded data (assuming it’s stored in myPacket.transmitData)
        printf("b'");
        for (int i = 0; i < myPacket.length; i++) {
            printf("\\x%02X", myPacket.transmitData[i]);
        }
        printf("'\n");

      } 

    int fd = open_serial_port("/dev/ttyUSB0", B115200);

    if (fd < 0) {
        dlclose(libhandle);
        return 1;
    }

  
    // Send encoded data
    ssize_t sent = write_serial_data(fd, myPacket.transmitData, myPacket.length);
    if (sent != myPacket.length) {
        fprintf(stderr, "Only wrote %zd of %d bytes to serial port\n", sent, myPacket.length);
    } else {
        printf("Successfully wrote %zd bytes to serial port\n", sent);
    }

    sleepExec(sleepDuration);

    // Clear input buffer 
    tcflush(fd, TCIFLUSH);

    // Read response from serial port
    uint8_t response[10] = {0};   // TODO: implementare la logica di parsing dei packets 
    ssize_t received = read_serial_data(fd, response, sizeof(response));
    if (received > 0) {
        printf("Received response from serial port:\n");
        for (ssize_t i = 0; i < received; i++) {
            printf("0x%02X ", response[i]);
        }
        printf("\n");
    }

    close(fd);

    struct packet decodedResponse;
        memset(&decodedResponse, 0, sizeof(decodedResponse)); // Initialize decodedResponse
        //uint8_t destData[64] = {0};
        int8_t decode_status = decode_func(&decodedResponse, response, received);
        if (decode_status != 1) {
            fprintf(stderr, "Error decoding packet: %d\n", decode_status);
            dlclose(libhandle);
            return 0;
        }

        printf("Response packet decoded successfully!\n");
        printf("Decoded values:\n");
        printf("  Address: 0x%02X\n", decodedResponse.address);
        printf("  Code:    0x%X\n", decodedResponse.code);
        printf("  Length:  %d\n", decodedResponse.length);
        printf("  Data:    ");
        printf("b'");
        for (int i = 0; i < decodedResponse.length - 4; i++) {
            printf("\\x%02X", decodedResponse.data[i]);
        }
        printf("\n");
}

int main() {
    int device = 0x01;
    float position = 5.0f;
    int packet_id = 0.03; // position id
    // sendPosition(device, position, 1, 50);
    request(device, packet_id, 1, 500);
}


