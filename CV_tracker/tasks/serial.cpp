#include "serial.hpp"

Serial::Serial(const char* port, speed_t baudrate) {
    fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        throw std::runtime_error("Error opening serial port");
    }

    if (tcgetattr(fd, &tty) != 0) {
        throw std::runtime_error("Error getting serial port attributes");
    }

    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= 0;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        throw std::runtime_error("Error setting serial port attributes");
    }
}

Serial::~Serial() {
    close(fd);
}

void Serial::sendData(const char* data, size_t length) {
    if (write(fd, data, length) != length) {
        throw std::runtime_error("Error writing to serial port");
    }
}