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
  int serialReadBytes(int serial_port, uint8_t* data, size_t len) override {
    return read(serial_port, data, len);
  }
  int serialAvailable(int serial_port) override {
    int bytes;
    ioctl(serial_port, FIONREAD, &bytes);
    return bytes;
  }
};

int serial_ports[4]; // sensors [0]: 231, [1]: 229, [2]: 243, [3]: 230
myBotaForceTorqueSensorComm sensors[4];

int main(int argc, char** argv)
{
    bool log = false;
    std::ofstream file;
//    std::ofstream file1;
//    std::ofstream file2;
//    std::ofstream file3;

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
        file << "timestamp0,f0,timestamp1,f1,timestamp2,f2,timestamp3,f3\n";

        /* IF LOGGING ALL 6 COMPONENTS OF FORCES */
//        file << "timestamp0,f0_0,f0_1,f0_2,f0_3,f0_4,f0_5,timestamp1,f1_0,f1_1,f1_2,f1_3,f1_4,f1_5,"
//                "timestamp2,f2_0,f2_1,f2_2,f2_3,f2_4,f2_5,timestamp3,f3_0,f3_1,f3_2,f3_3,f3_4,f3_5\n";

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
    bool ready = false; // is set to true once all sensors have been initialized and are transmitting correct data
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
            BotaForceTorqueSensorComm::ReadFrameRes res = sensors[i].readFrame(serial_ports[i]);
            switch (res) {
                case BotaForceTorqueSensorComm::VALID_FRAME:
                    if (sensors[i].frame.data.status.val > 0) {
                        printf("No valid forces:\n");
                        printf(" app_took_too_long: %i\n", sensors[i].frame.data.status.app_took_too_long);
                        printf(" overrange: %i\n", sensors[i].frame.data.status.overrange);
                        printf(" invalid_measurements: %i\n", sensors[i].frame.data.status.invalid_measurements);
                        printf(" raw_measurements: %i\n", sensors[i].frame.data.status.raw_measurements);
                        --i; //retry same sensor
                    } else {

                        if(ready) {
                            printf("%u\t", sensors[i].frame.data.timestamp);
                            printf("%f\t", sensors[i].frame.data.forces[2]);
                            if (i==3 && log) // last csv column, no comma at end
                                file << sensors[i].frame.data.timestamp << ","
                                << sensors[i].frame.data.forces[2];
                            else if (log)
                                file << sensors[i].frame.data.timestamp << ","
                                << sensors[i].frame.data.forces[2] << ",";
                        }


/* FOR READING/LOGGING ALL 6 FORCES VALUES */
//                        if(ready) {
//                            printf("%u\t", sensors[i].frame.data.timestamp);
//                            if (log)
//                                file << sensors[i].frame.data.timestamp << ",";
//                        }
//
////                                file << sensor.frame.data.temperature << ",";
////                        printf("%f\t", sensor.frame.data.temperature);
//
//
//                        for (uint8_t j = 0; j < 6; ++j) {
//                            if (ready) {
//                                printf("%f\t", sensors[i].frame.data.forces[j]);
//                                if (i == 3 && j == 5 && log)   // last csv column, no comma at end
//                                    file << sensors[i].frame.data.forces[j];
//                                else if (log)
//                                    file << sensors[i].frame.data.forces[j] << ",";
//                            }
//
//                        }
//
//                        if(ready)
//                            printf("sensor %i\t", i);

                        ++iterations;
                    }
                        break;

                case BotaForceTorqueSensorComm::NOT_VALID_FRAME:
                    printf("No valid frame: %i\n", sensors[i].get_crc_count());
                    --i; //retry same sensor
                    break;
                case BotaForceTorqueSensorComm::NOT_ALLIGNED_FRAME:
                    printf("lost sync, trying to reconnect\n");
                    --i; //retry same sensor
                    break;
                case BotaForceTorqueSensorComm::NO_FRAME:
                    --i; //retry on same sensor
                    break;
            }
            if (res == myBotaForceTorqueSensorComm::VALID_FRAME && i == 3 && sensors[i].frame.data.status.val <= 0) {
                if (ready) {
                    printf("\n");
                    if (log)
                        file << std::endl;
                }
                if (!ready)
                    ready = true; //all sensors initialized}

            }
        }
    }// while app run

    printf("close serial port.\n");
    for(size_t i = 0; i < sizeof(serial_ports)/sizeof(*serial_ports); ++i)
        close(serial_ports[i]);
}
