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
#include <linux/serial.h>

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

size_t const NUM_SENSORS = 4;
int serial_ports[NUM_SENSORS]; // sensors [0]: 231, [1]: 229, [2]: 243, [3]: 230
std::string ports[NUM_SENSORS] ={"/dev/ttyUSB0",
                                 "/dev/ttyUSB1",
                                 "/dev/ttyUSB2",
                                 "/dev/ttyUSB3"};

//std::string ports[NUM_SENSORS] ={"/dev/ttyUSB3"};
myBotaForceTorqueSensorComm sensors[NUM_SENSORS];


int main(int argc, char** argv)
{
    bool log = false;
    std::ofstream file;
    std::string filepath;

    if (argc > 1)
    {
        log = true;
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);

        if (strcmp(argv[1],"log") == 0)
        {
            filepath = "/home/narek/MA/BotaSerialDriver/logs/log.csv";
        }
        else if (strcmp(argv[1],"log_datetime") == 0)
        {
            std::ostringstream oss;
            oss <<  "/home/narek/MA/BotaSerialDriver/logs/log_"
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
        std::ostringstream header;
        for (size_t i = 0; i < NUM_SENSORS; ++i) {
            if(i == NUM_SENSORS-1){
                header << "timestamp" << i << "," << "f" << i << "\n";
                continue;
            }
            header << "timestamp" << i << "," << "f" << i << ",";
        }

        file << header.str();

        /* IF LOGGING ALL 6 COMPONENTS OF FORCES */
//        file << "timestamp0,f0_0,f0_1,f0_2,f0_3,f0_4,f0_5,timestamp1,f1_0,f1_1,f1_2,f1_3,f1_4,f1_5,"
//                "timestamp2,f2_0,f2_1,f2_2,f2_3,f2_4,f2_5,timestamp3,f3_0,f3_1,f3_2,f3_3,f3_4,f3_5\n";

    }


    /* Open the serial port. Change device path as needed.
     */
    printf("Opening serial ports.\n");

    for (size_t i=0; i < NUM_SENSORS; ++i) {
        serial_ports[i] = open(ports[i].c_str(), O_RDWR);
        if (serial_ports[i] < 0) {
            printf("Error %i from opening device %zu: %s\n", errno, i, strerror(errno));
            std::remove(filepath.c_str());
            if (errno == 13) {
                printf("Add the current user to the dialout group");
            }
            return 1;
        }
    }


    std::ostringstream oss;
    oss <<  "Opened ports: ";
    for(size_t i = 0; i < NUM_SENSORS; ++i) {
        oss << serial_ports[i] << " ";
    }
    printf("%s", oss.str().c_str());


    // Create new termios struct, we call it 'tty' for convention
    struct termios tty[NUM_SENSORS];
    struct serial_struct ser_info[NUM_SENSORS];
    memset(&tty, 0, sizeof(tty));

    for (size_t i = 0; i < NUM_SENSORS; ++i) {
        // Read in existing settings, and handle any error
        if (tcgetattr(serial_ports[i], &tty[i]) != 0) {
            printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
            return false;
        }

        tty[i].c_cflag &= ~PARENB;   // Disable parity
        tty[i].c_cflag &= ~CSTOPB;   // 1 stop bit
        tty[i].c_cflag |= CS8;       // 8 bits per byte
        tty[i].c_cflag &= ~CRTSCTS;  // Disable RTS/CTS hardware flow control
        tty[i].c_cflag |= (CREAD | CLOCAL);  // Turn on READ & ignore ctrl lines (CLOCAL = 1)

        tty[i].c_lflag &= ~ICANON;  // Disable canonical mode
        tty[i].c_lflag &= ~ECHO;    // Disable echo
        tty[i].c_lflag &= ~ECHOE;   // Disable erasure
        tty[i].c_lflag &= ~ECHONL;  // Disable new-line echo
        tty[i].c_lflag &= ~ISIG;    // Disable interpretation of INTR, QUIT and SUSP
        tty[i].c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
        tty[i].c_iflag &=
                ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

        tty[i].c_oflag &= ~OPOST;  // Prevent special interpretation of output bytes (e.g. newline chars)
        tty[i].c_oflag &= ~ONLCR;  // Prevent conversion of newline to carriage return/line feed

        tty[i].c_cc[VTIME] = 10;  // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
        tty[i].c_cc[VMIN] = 0;

        // Set in/out baud rate to be 460800
        cfsetispeed(&tty[i], B460800);
        cfsetospeed(&tty[i], B460800);

        if (tcsetattr(serial_ports[i], TCSANOW, &tty[i]) != 0) {
            printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
            return false;
        }

        // Enable linux FTDI low latency mode
        ioctl(serial_ports[i], TIOCGSERIAL, &ser_info[i]);
        ser_info[i].flags |= ASYNC_LOW_LATENCY;
        ioctl(serial_ports[i], TIOCSSERIAL, &ser_info[i]);
    }



    int iterations = 0;
    bool ready = false; // is set to true once all sensors have been initialized and are transmitting correct data
//    const int MAX_ITERATIONS = 10000000;

    while (1) {


//      if(iterations == MAX_ITERATIONS)
//      {
//          file.close();
//          break;
//      }

        for(size_t i = 0; i < NUM_SENSORS; ++i) {
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
                            if (i==NUM_SENSORS-1 && log) // last csv column, no comma at end
                                file << std::setprecision(64) << sensors[i].frame.data.timestamp << ","
                                << std::setprecision(64) << sensors[i].frame.data.forces[2];
                            else if (log)
                                file << std::setprecision(64) << sensors[i].frame.data.timestamp << ","
                                << std::setprecision(64) << sensors[i].frame.data.forces[2] << ",";
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
//                                if (i == NUM_SENSORS-1 && j == 5 && log)   // last csv column, no comma at end
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
            if (res == myBotaForceTorqueSensorComm::VALID_FRAME && i == NUM_SENSORS-1 && sensors[i].frame.data.status.val <= 0) {
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
