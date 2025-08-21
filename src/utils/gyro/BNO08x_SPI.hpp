/*
This library is used to communicate with the BNO08x IMU over SPI.
It handles SPI communication, GPIO pin management for wake, interrupt, and reset functionalities,
and provides methods for reading and writing data to the device.

The actual SHTP packet handling code is adapted from the Adafruit BNO08x library for Arduino.
*/


#include <iostream>
#include <unistd.h>
#include <gpiod.hpp>
#include <filesystem>
#include <thread>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <string.h>

class BNO08x_SPI {
    int fd;
    uint32_t speed;
    uint8_t mode, bits;
    gpiod::line::offset wake_offset, int_offset, rst_offset;
    std::filesystem::path chip_path;
    gpiod::line_request request;
    char* spidev;

public:
    BNO08x_SPI(char* spidev, char* gpiochip, int rst_pin, int wake_pin, int int_pin, uint32_t speed = 1000000)
        :
        spidev(spidev), chip_path(gpiochip), rst_offset(rst_pin), wake_offset(wake_pin), int_offset(int_pin), speed(speed),
        request(gpiod::chip(chip_path)
			.prepare_request()
			.set_consumer("BNO085x")
			.add_line_settings(
				rst_offset,
				gpiod::line_settings()
                    .set_direction(
					    gpiod::line::direction::OUTPUT)
                    .set_active_low(true)
                    .set_output_value(gpiod::line::value::INACTIVE)
                    .set_drive(gpiod::line::drive::PUSH_PULL)
                )
            .add_line_settings(
                wake_offset,
                gpiod::line_settings()
                    .set_direction(
                        gpiod::line::direction::OUTPUT)
                    .set_active_low(true)
                    .set_output_value(gpiod::line::value::INACTIVE)
                    .set_drive(gpiod::line::drive::PUSH_PULL)
            )
            .add_line_settings(
                int_offset,
                gpiod::line_settings()
                    .set_direction(
                        gpiod::line::direction::INPUT)
                    .set_bias(gpiod::line::bias::PULL_UP)
                    .set_active_low(true)
            )
			.do_request())
        {
            std::cout << gpiochip << "\n";
        }

    // Destructor
    ~BNO08x_SPI() {
        close(fd); // Close SPI dev
    }

    bool begin() {
        fd = open(spidev, O_RDWR);
        if (fd < 0) {
            perror("Failed to open SPI device!");
            return false;
        }

        mode = SPI_MODE_3;
        bits = 8;

        // Set SPI parameters
        if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
            perror("Can't set mode");
            return false;
        }
        if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
            perror("Can't set bits");
            return false;
        }
        if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) { 
            perror("Can't set speed");
            return false;
        }

        std::cout << std::showbase       // adds "0x" prefix
              << std::hex;
        
        return true;
    }


    int write(uint8_t *buffer, int len) {
        
        std::cout << "writing ";
        for (unsigned int i = 0; i < len; i++) {
            std::cout << (unsigned int) buffer[i] << " ";
        }
        std::cout << "\n";

        // Set up transfer
        struct spi_ioc_transfer tr = {
            .tx_buf = (unsigned long) buffer,
            .rx_buf = 0,
            .len = len,
            .speed_hz = speed,
            .bits_per_word = 8,
        };

        // Wake up device
        if (wait_for_int() < 0) {
            std::cerr << "Could not wake up device!\n";
            return -1;
        }

        // Send data
        if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
            perror("SPI transfer failed");
            return -1;
        }

        return 0;
    }

    int read(uint8_t *buffer, unsigned int len, int sendValue) {
        memset(buffer, sendValue, len); // Clear buffer

        // Setup transfer
        struct spi_ioc_transfer tr = {
            .tx_buf = 0,
            .rx_buf = (unsigned long) buffer,
            .len = len,
            .speed_hz = speed,
            .bits_per_word = 8,
        };
        
        // Wake up device
        if (wait_for_int() < 0) {
            std::cerr << "Could not wake up device!\n";
            return -1;
        }

        if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
            perror("SPI transfer failed");
            return -1;
        }

        /*std::cout << "Received packet: ";
        for (unsigned int i = 0; i < buffer[0]; i++) {
            std::cout << (unsigned int) buffer[i] << " ";
        }
        std::cout << "\n";*/
    
        return 0;
    }

    int wait_for_int(bool wake = true) {
        if (request.get_value(int_offset) == gpiod::line::value::ACTIVE) {
            return 0;
        }
        if (wake) {
            // Activate wake pin
            request.set_value(wake_offset, gpiod::line::value::ACTIVE);
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            request.set_value(wake_offset, gpiod::line::value::INACTIVE);
        }
        
        // Wait for interrupt from the device
        auto start_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> dt(std::chrono::high_resolution_clock::now() - start_time);
        while (dt.count() < 3000.0) {
            if (request.get_value(int_offset) == gpiod::line::value::ACTIVE) {
                return 0;
            }
            dt = std::chrono::high_resolution_clock::now() - start_time;
        }

        // Throw error if time limit is exceeded
        return -1;
    }
    
    
    // Reset device by activating reset pin
    int hard_reset() {
        std::cout << "Hard resetting...\n";

        // Send pulse on reset line
        request.set_value(rst_offset, gpiod::line::value::ACTIVE);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        request.set_value(rst_offset, gpiod::line::value::INACTIVE);
        
        // Wait for device to respond
        if (wait_for_int(false) >= 0) {
            std::cout << "Done!\n";
            //software_reset();
            return 0;
        } else {
            return -1;
        }

    }
    int software_reset() {
        uint8_t buffer[] = {5, 0, 1, 1, 1};
        write(buffer, 5);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        uint8_t buff[6];
        if (read(buff, 6, 0x00) < 0) {
            exit(-69);
        }

        return 0;
    }

};
