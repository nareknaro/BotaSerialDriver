// C library headers
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>


#include <fstream>
#include <iostream>
#include <iomanip>
#include <ctime>

// Linux headers
#include <fcntl.h>      // Contains file controls like O_RDWR
#include <errno.h>      // Error integer and strerror() function
#include <termios.h>    // Contains POSIX terminal control definitions
#include <unistd.h>     // write(), read(), close()
#include <sys/ioctl.h>

#include "../BotaForceTorqueSensorComm.h"

int serial_port0;
int serial_port1;
int serial_port2;
int serial_port3;

class myBotaForceTorqueSensorComm : public BotaForceTorqueSensorComm
{
  public:
  int serialReadBytes(int *serial_port, uint8_t* data, size_t len) override {
    return read(*serial_port, data, len);
  }
  int serialAvailable(int *serial_port) override {
    int bytes;
    ioctl(*serial_port, FIONREAD, &bytes);
    return bytes;
  }
} sensor;

int main(int argc, char** argv)
{
    bool log = false;
    std::ofstream file;

    if (argc > 1)
    {
        log = true;
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::string filepath;

        if (strcmp(argv[1],"log") == 0)
        {
            filepath = "/home/narek/BotaSerialDriver/logs/log.csv";
        }
        else if (strcmp(argv[1],"log_datetime") == 0)
        {
            std::ostringstream oss;
            oss <<  "/home/narek/BotaSerialDriver/logs/log_"
                << std::put_time(&tm, "%d-%m-%y_%H-%M-%S") << ".csv";
            filepath = oss.str();
        }
        else
        {
            printf("Argument should be either \"log\" (save to log.csv)\nor \"log_datetime\" "
                   "(save to log_dd-mm-yy_hh-mm-ss.csv).\n");
            return 1;
        }

        file.open(filepath);
        file << "timestamp,f1,f2,f3,f4,f5,f6\n";
    }


    /* Open the serial port. Change device path as needed.
     */
    printf("Opening serial ports.\n");
    serial_port0 = open("/dev/ttyUSB0", O_RDWR);
    serial_port1 = open("/dev/ttyUSB1", O_RDWR);
    serial_port2 = open("/dev/ttyUSB2", O_RDWR);
    serial_port3 = open("/dev/ttyUSB3", O_RDWR);
    printf("Opened port %i.\n",serial_port);

    if (serial_port0 < 0) {
      printf("Error %i from opening device: %s\n", errno, strerror(errno));
      if (errno == 13) {
        printf("Add the current user to the dialout group");
      }
      return 1;
    }

    // Create new termios struct, we call it 'tty' for convention
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port0, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB; // Disable parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag |= CS8; // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON; // Disable canonical mode
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 460800
    cfsetispeed(&tty, B460800);
    cfsetospeed(&tty, B460800);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port0, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    int iterations = 0;
//    const int MAX_ITERATIONS = 10000000;

    while (1) {


//      if(iterations == MAX_ITERATIONS)
//      {
//          file.close();
//          break;
//      }

      switch(sensor.readFrame(&serial_port))
      {
        case BotaForceTorqueSensorComm::VALID_FRAME:
          if (sensor.frame.data.status.val>0)
          {
            printf("No valid forces:\n");
            printf(" app_took_too_long: %i\n",sensor.frame.data.status.app_took_too_long);
            printf(" overrange: %i\n",sensor.frame.data.status.overrange);
            printf(" invalid_measurements: %i\n",sensor.frame.data.status.invalid_measurements);
            printf(" raw_measurements: %i\n",sensor.frame.data.status.raw_measurements);
          }
          else
          {

            if(log)
            {
                file << sensor.frame.data.timestamp << ",";
//                file << sensor.frame.data.temperature << ",";
            }


            printf("%u\t", sensor.frame.data.timestamp);
            printf("%f\t", sensor.frame.data.temperature);
            bool last_col;

            for (uint8_t i=0; i<6; i++)
            {
              last_col = i == 5;
              if(last_col && log)
                  file <<  sensor.frame.data.forces[i];
              else if(log)
                  file <<  sensor.frame.data.forces[i] << ",";

              printf("%f",sensor.frame.data.forces[i]);
              printf("\t");
            }
            if(log)
                file << std::endl; // new line and flush buffer

            printf("\n");

            ++iterations;
          }

          break;
        case BotaForceTorqueSensorComm::NOT_VALID_FRAME:
          printf("No valid frame: %i\n",sensor.get_crc_count());
          break;
        case BotaForceTorqueSensorComm::NOT_ALLIGNED_FRAME:
          printf("lost sync, trying to reconnect\n");
          break;
        case BotaForceTorqueSensorComm::NO_FRAME:
          break;
      }
    }// while app run

    printf("close serial port.\n");
    close(serial_port0);
}
