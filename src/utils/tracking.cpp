#include <iostream>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <thread>
#include <chrono>
#include <queue>
#include "gyro/Adafruit_BNO08x.h"
#include <pybind11/pybind11.h>

#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define GYRO_XOUT_H 0x43
#define ACCEL_XOUT_H 0x3B

namespace py = pybind11;

struct coord_3d {
    double x, y, z;
};

struct mouse_data {
    double x, y;
};

struct eulerAngles {
    double roll;
    double pitch;
    double yaw;
};

// class for tracking the car's position in real time
class PositionTracker {
    // Gyro returns quaternions, so we need to convert them to Euler angles
    eulerAngles calculateEulerAngles(double r, double i, double j, double k) {
        eulerAngles angles;

        angles.yaw = std::atan2(2.0 * (i * j + k * r), (i*i - j*j - k*k + r*r));
        angles.pitch = std::asin(-2.0 * (i * k - j * r) / (i*i + j*j + k*k + r*r));
        angles.roll = std::atan2(2.0 * (j * k + i * r), (-(i*i) - j*j + k*k + r*r));

        return angles;
    }

    // Process the gyro data in a separate thread
    // This will run in parallel with the main tracking process
    double gyroProcesss() {
        auto prev_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> dt;

        while (!kill_switch) {
            
            // Wait for the next sensor event (this is blocking, and this is what actually reads the data)
            if (!gyro.getSensorEvent(&sensorValue)) {
                // The gyro's data rate should go up to 400Hz, but I could only get it to 200Hz :/
                std::this_thread::sleep_for(std::chrono::microseconds(5000));
                continue;
            }

            // If the sensor event is not a game rotation vector, skip it
            if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
                double r = sensorValue.un.gameRotationVector.real;
                double i = sensorValue.un.gameRotationVector.i;
                double j = sensorValue.un.gameRotationVector.j;
                double k = sensorValue.un.gameRotationVector.k;

                eulerAngles angles = calculateEulerAngles(r, i, j, k);
                
                // Update variables with the calculated angles
                angle_rad = angles.yaw;
                angle = angle_rad * 180 / M_PI;

                /*dt = std::chrono::high_resolution_clock::now() - prev_time;
                std::cout << "time: " << dt.count() << "\n";
                
                std::cout << "angle: " << angle << "\n";
                prev_time = std::chrono::high_resolution_clock::now();*/
            }
     
        }
    }

    void process() {
        std::cout << "Started tracking\n";
        unsigned char data[3];
        while (!kill_switch) {
            ssize_t bytes = read(fd, data, sizeof(data));
            
            if (bytes < 0) {
                perror("Failed to read from /dev/mice");
                break;
            } else if (bytes == 3) {
                double dx = static_cast<int8_t>(data[1]) / conversion_factor;
                double dy = static_cast<int8_t>(data[2]) / conversion_factor;
                //std::cout << "Movement: " << dx << " " << dy << "\n";
                //std::cout << "Gyro angle: " << angle << "\n";

                x += dx * cos(angle_rad) - dy * sin(angle_rad);
                y += dx * sin(angle_rad) + dy * cos(angle_rad);

                //std::cout << "Position: x: " << x << " y: " << y << "\n";
            }
        }
    }
    double conversion_factor, angle_rad, angle, speed;
    double x = 0, y = 0;
    bool kill_switch = false;
    int fd;


public:
    Adafruit_BNO08x gyro;
    sh2_SensorValue_t sensorValue;

    //Starting I2C communication
    PositionTracker(double conversion_factor = 100)
    :
    conversion_factor(conversion_factor)
    {
        // Initialize the gyro sensor and pray that it doesn't randomly crash (it does that way too often)
        // First try after reboot fails... I have no idea why
        if (!gyro.begin_SPI("/dev/spidev0.0", "/dev/gpiochip0", 26, 25, 19, 4000000, 0)) {
            std::cerr << "Failed to initialize sensor\n";
            exit(-1);
        }

        // Open mice device; this works reliably (something at at last)
        fd = open("/dev/input/mice", O_RDONLY);
        if (fd == -1) {
            perror("Failed to open /dev/mice");
            exit(-1);
        }

        gyro.enableReport(SH2_GAME_ROTATION_VECTOR, 10000U); // Enable rotation vector report every 10ms ONLY 100Hz...
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Sleep because like this it fails less often
        std::thread gyro_t(&PositionTracker::gyroProcesss, this); // Separate thread for gyro data handling
        gyro_t.detach();

        std::thread t(&PositionTracker::process, this); // Another thread for the main tracking process
        t.detach();
    }

    // end tracking process
    ~PositionTracker() {
        kill();
    }

    void kill() {
        kill_switch = true;
    }

    // geting specific values from tracker
    mouse_data get() {
        mouse_data data;
        data.x = x;
        data.y = y;

        return data;
    }
    
    double get_x() {
        return x;
    }
    double get_y() {
        return y;
    }
    double get_speed() {
        return speed;
    }

    void reset() {
        x = 0;
        y = 0;
    }

    double get_angle_rad() {
        return angle_rad;
    }

    double get_angle() {
        return angle;
    }
};

PYBIND11_MODULE(tracker, m) {
    py::class_<PositionTracker>(m, "PositionTracker")
        .def(py::init<double>())
        .def("get", &PositionTracker::get)
        .def("get_x", &PositionTracker::get_x)
        .def("get_y", &PositionTracker::get_y)
        .def("kill", &PositionTracker::kill)
        .def("reset", &PositionTracker::reset)
        .def("get_angle", &PositionTracker::get_angle)
        .def("get_angle_rad", &PositionTracker::get_angle_rad)
        .def("get_speed", &PositionTracker::get_speed)
        .def_readwrite("gyro", &PositionTracker::gyro);
};

// starting the tracking and the communication with gyro
/*int main() {
    PositionTracker tracker("/dev/i2c-1", 0x69 - 1, 1, 0, 496.5);
    //Gyro gyro("/dev/i2c-1", 0x68, 1, 0);

    while (true) {
        //std::cout << gyro.get_z() << "\n";
        std::cout << tracker.get_speed() << "\n";
    }
}*/