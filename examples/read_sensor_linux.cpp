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
};

int serial_ports[4]; // sensors [0]: 231, [1]: 229, [2]: 243, [3]: 230
myBotaForceTorqueSensorComm sensors[4];

int main(int argc, char** argv)
{
    bool log = false;
    std::ofstream file0;
    std::ofstream file1;
    std::ofstream file2;
    std::ofstream file3;

    if (argc > 1)
    {
        log = true;
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::string filepaths[4];


        if (strcmp(argv[1],"log") == 0)
        {
            filepaths[0] = "/home/narek/BotaSerialDriver/logs/log0.csv";
            filepaths[1] = "/home/narek/BotaSerialDriver/logs/log1.csv";
            filepaths[2] = "/home/narek/BotaSerialDriver/logs/log2.csv";
            filepaths[3] = "/home/narek/BotaSerialDriver/logs/log3.csv";
        }
        else if (strcmp(argv[1],"log_datetime") == 0)
        {
//            std::ostringstream oss;
//            oss <<  "/home/narek/BotaSerialDriver/logs/log_"
//                << std::put_time(&tm, "%d-%m-%y_%H-%M-%S") << ".csv";
//            filepath = oss.str();
        }
        else
        {
            printf("Argument should be either \"log\" (save to log.csv)\nor \"log_datetime\" "
                   "(save to log_dd-mm-yy_hh-mm-ss.csv).\n");
            return 1;
        }

        file0.open(filepaths[0]);
        file0 << "timestamp,f1,f2,f3,f4,f5,f6\n";
        file1.open(filepaths[1]);
        file1 << "timestamp,f1,f2,f3,f4,f5,f6\n";
        file2.open(filepaths[2]);
        file2 << "timestamp,f1,f2,f3,f4,f5,f6\n";
        file3.open(filepaths[3]);
        file3 << "timestamp,f1,f2,f3,f4,f5,f6\n";
    }


    /* Open the serial port. Change device path as needed.
     */
    printf("Opening serial ports.\n");
    serial_ports[0] = open("/dev/ttyUSB0", O_RDWR);
    serial_ports[1] = open("/dev/ttyUSB1", O_RDWR);
    serial_ports[2] = open("/dev/ttyUSB2", O_RDWR);
    serial_ports[3] = open("/dev/ttyUSB3", O_RDWR);
    printf("Opened ports %i, %i, %i, %i.\n",serial_ports[0], serial_ports[1], serial_ports[2], serial_ports[3]);

    if (serial_ports[0] < 0 || serial_ports[1] < 0 || serial_ports[2] < 0 || serial_ports[3] < 0) {
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
    if(tcgetattr(serial_ports[0], &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    } else if (tcgetattr(serial_ports[1], &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    } else if (tcgetattr(serial_ports[2], &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    } else if (tcgetattr(serial_ports[3], &tty) != 0) {
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
    if (tcsetattr(serial_ports[0], TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    } else if (tcsetattr(serial_ports[1], TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    } else if (tcsetattr(serial_ports[2], TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    } else if (tcsetattr(serial_ports[3], TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    int iterations = 0;
//    size_t num_ports = sizeof(serial_ports)/sizeof(*serial_ports);
//    num_ports = size_t(1);
//    const int MAX_ITERATIONS = 10000000;

    while (1) {


//      if(iterations == MAX_ITERATIONS)
//      {
//          file.close();
//          break;
//      }

        for(int i = 0; i < 4; ++i) {
            BotaForceTorqueSensorComm::ReadFrameRes res = sensors[i].readFrame(&serial_ports[i]);
            switch (res) {
                case BotaForceTorqueSensorComm::VALID_FRAME:
                    if (sensors[i].frame.data.status.val > 0) {
                        printf("No valid forces:\n");
                        printf(" app_took_too_long: %i\n", sensors[i].frame.data.status.app_took_too_long);
                        printf(" overrange: %i\n", sensors[i].frame.data.status.overrange);
                        printf(" invalid_measurements: %i\n", sensors[i].frame.data.status.invalid_measurements);
                        printf(" raw_measurements: %i\n", sensors[i].frame.data.status.raw_measurements);
                    } else {

                        if (log) {
                            switch(i){
                                case 0:
                                    file0 << sensors[i].frame.data.timestamp << ",";
                                    break;
                                case 1:
                                    file1 << sensors[i].frame.data.timestamp << ",";
                                    break;
                                case 2:
                                    file2 << sensors[i].frame.data.timestamp << ",";
                                    break;
                                case 3:
                                    file3 << sensors[i].frame.data.timestamp << ",";
                                    break;
                            }

//                            file << sensor.frame.data.timestamp << ",";
                            //                file << sensor.frame.data.temperature << ",";
                        }


                        printf("%u\t", sensors[i].frame.data.timestamp);
//                        printf("%f\t", sensor.frame.data.temperature);


                        for (uint8_t j = 0; j < 6; ++j) {
                            if (j == 5 && log) { // last csv column, no comma at end
                                switch (i) {
                                    case 0:
                                        file0 << sensors[i].frame.data.forces[j];
                                        break;
                                    case 1:
                                        file1 << sensors[i].frame.data.forces[j];
                                        break;
                                    case 2:
                                        file2 << sensors[i].frame.data.forces[j];
                                        break;
                                    case 3:
                                        file3 << sensors[i].frame.data.forces[j];
                                        break;
                                }
                            }

                            else if (log) {
                                switch (i) {
                                    case 0:
                                        file0 << sensors[i].frame.data.forces[j] << ",";
                                        break;
                                    case 1:
                                        file1 << sensors[i].frame.data.forces[j] << ",";
                                        break;
                                    case 2:
                                        file2 << sensors[i].frame.data.forces[j] << ",";
                                        break;
                                    case 3:
                                        file3 << sensors[i].frame.data.forces[j] << ",";
                                        break;
                                }
                            }

                            printf("%f", sensors[i].frame.data.forces[j]);
                            printf("\t");
                        }
                        if (log)
                            switch(i){
                                case 0:
                                    file0 << std::endl;
                                    break;
                                case 1:
                                    file1 << std::endl;
                                    break;
                                case 2:
                                    file2 << std::endl;
                                    break;
                                case 3:
                                    file3 << std::endl;
                                    break;
                            }

                        printf("%i\n", i);

                        ++iterations;
                    }

                    break;
                case BotaForceTorqueSensorComm::NOT_VALID_FRAME:
                    printf("No valid frame: %i\n", sensors[i].get_crc_count());
                    break;
                case BotaForceTorqueSensorComm::NOT_ALLIGNED_FRAME:
                    printf("lost sync, trying to reconnect\n");
                    break;
                case BotaForceTorqueSensorComm::NO_FRAME:
                    break;
            }
            if (res==BotaForceTorqueSensorComm::NO_FRAME) --i; //retry
//            if (res != BotaForceTorqueSensorComm::NO_FRAME && i == 3)
//                printf("\n");
        }
    }// while app run

    printf("close serial port.\n");
    for(size_t i = 0; i < sizeof(serial_ports)/sizeof(*serial_ports); ++i)
        close(serial_ports[i]);
}
