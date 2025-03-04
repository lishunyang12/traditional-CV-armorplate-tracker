#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <stdexcept>

class Serial {
public:
    Serial(const char* port, speed_t baudrate);
    ~Serial();
    void sendData(const char* data, size_t length);

private:
    int fd;
    termios tty;
};

#endif // SERIAL_HPP