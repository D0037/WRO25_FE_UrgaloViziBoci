#include <iostream>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <thread>
#include <chrono>
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

class Gyro {
    public:
        Gyro(char* dev, int addr, int fs_sel, int afs_sel)
        :
        dev(dev), addr(addr), fs_sel(fs_sel), afs_sel(afs_sel)
        {
            file = open(dev, O_RDWR);
            
            // check for errors
            if (file < 0) {
                perror("Failed to open I2C bus");
                exit(1);
            }

            if (ioctl(file, I2C_SLAVE, addr) < 0) {
                perror("Failed to acquire bus access and/or talk to slave");
                exit(1);
            }

            g_scale_factor = 131.0 / (fs_sel + 1);
            a_scale_factor = 16384 / pow(2, afs_sel);

            // set fs and afs select on the gyro
            int current = readByte(GYRO_CONFIG);
            int new_cfg = (current & ~0x18) | (fs_sel << 3);
            writeByte(GYRO_CONFIG, new_cfg);

            current = readByte(ACCEL_CONFIG);
            new_cfg = (current & ~0x18) | (afs_sel << 3);
            writeByte(ACCEL_CONFIG, new_cfg);
            
            std::cout << "Calibration started (for real)\n";
            calibrate();
            std::cout << "Calibration finished (for real)\n";

            std::thread t(&Gyro::gyro_process, this);
            t.detach();
        }

        ~Gyro() {
            kill();
            if (file >= 0) close(file);
        }

        coord_3d get() {
            coord_3d gyro;
            gyro.x = x;
            gyro.y = y;
            gyro.z = z;

            return gyro;
        }

        double get_x_vel() {
            return read_i16(GYRO_XOUT_H) / g_scale_factor;
        }

        double get_y_vel() {
            return read_i16(GYRO_XOUT_H + 2) / g_scale_factor;
        }

        double get_z_vel() {
            return read_i16(GYRO_XOUT_H + 4) / g_scale_factor;
        }

        coord_3d get_vel() {
            coord_3d vel;
            vel.x = get_x_vel() / g_scale_factor;
            vel.y = get_y_vel() / g_scale_factor;
            vel.z = get_z_vel() / g_scale_factor;

            return vel;
        }

        double get_x() {
            return x;
        }

        double get_y() {
            return y;
        }

        double get_z() {
            return z;
        }

        void reset() {
            x = 0;
            y = 0;
            z = 0;
        }

        double get_ax() {
            return read_i16(ACCEL_XOUT_H) / a_scale_factor;
        }

        double get_ay() {
            return read_i16(ACCEL_XOUT_H + 2) / a_scale_factor;
        }

        double get_az() {
            return read_i16(ACCEL_XOUT_H + 4) / a_scale_factor;
        }

        coord_3d get_accel() {
            coord_3d accel;
            accel.x = x;
            accel.y = y;
            accel.z = z;

            return accel;
        }

        void calibrate(int samples = 5000) {
            double total_x = 0.0, total_y = 0.0, total_z = 0.0;

            py::gil_scoped_release release;
            for (int i = 0; i < samples; i++) {
                total_x += get_x_vel();
                total_y += get_y_vel();
                total_z += get_z_vel();
            }

            bias_x = total_x / samples;
            bias_y = total_y / samples;
            bias_z = total_z / samples;
        }

        void gyro_process() {
            std::cout << "started\n";
            auto prev_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> dt;
            while (!kill_switch) {
                int x_vel = get_x_vel() - bias_x;
                int y_vel = get_y_vel() - bias_y;
                int z_vel = get_z_vel() - bias_z;

                dt = std::chrono::high_resolution_clock::now() - prev_time;
                prev_time = std::chrono::high_resolution_clock::now();

                speed = dt.count();

                x += x_vel * (dt.count() / 1000);
                y += y_vel * (dt.count() / 1000);
                z += z_vel * (dt.count() / 1000);
            }
        }

        void kill() {
            kill_switch = true;
        }

        double get_speed() {
            return speed;
        }
    
        double bias_x, bias_y, bias_z;
    private:
        void writeByte(uint8_t reg, uint8_t data) {
            uint8_t buf[2] = {reg, data};
            if (write(file, buf, 2) != 2) {
                perror("Failed to write to the I2C device"); 
            }
        }

        uint8_t readByte(uint8_t reg) {
            if (write(file, &reg, 1) != 1) {
                perror("Failed to write to the I2C device0");
                return 0;
            }

            uint8_t data;
            if (read(file, &data, 1) != 1) {
                perror("Failed to read from the I2C device1");
                return 0;
            }


            return data;
        }

        int16_t read_i16(uint8_t reg) {
            uint8_t high = readByte(reg);
            uint8_t low = readByte(reg + 1);

            int16_t value = (high << 8) | low;

            if (value & 0x8000) {
                value -= 0x10000;
            }

            return value;
        }

        char* dev;
        int addr, file, fs_sel, afs_sel;
        double g_scale_factor;
        int a_scale_factor;
        double x = 0, y = 0, z = 0;
        bool kill_switch = false;
        double speed;
};

class PositionTracker {
public:
    Gyro gyro;

    PositionTracker(char* gyro_i2c = "/dev/i2c-1", int gyro_addr = 0x68, int fs_sel = 0, int afs_sel = 0, double conversion_factor = 100)
    :
    conversion_factor(conversion_factor), gyro(Gyro(gyro_i2c, gyro_addr, fs_sel, afs_sel))
    {
        //Initialize gyro
        std::cout << gyro.bias_x;

        // Open mice device
        fd = open("/dev/input/mice", O_RDONLY);
        if (fd == -1) {
            perror("Failed to open /dev/mice");
            exit(-1);
        }

        std::thread t(&PositionTracker::process, this);
        t.detach();
    }

    ~PositionTracker() {
        kill();
    }

    void kill() {
        kill_switch = true;
        gyro.kill();
    }

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
    double get_gyro_z() {
        return gyro.get_z();
    }

private:
    void process() {
        unsigned char data[3];
        while (!kill_switch) {
            ssize_t bytes = read(fd, data, sizeof(data));
            if (bytes < 0) {
                perror("Failed to read from /dev/mice");
                break;
            } else if (bytes == 3) {
                double dx = static_cast<int8_t>(data[1]) / conversion_factor;
                double dy = static_cast<int8_t>(data[2]) / conversion_factor;

                angle = gyro.get_z();
                angle_rad = (angle / 180) * M_PI;

                x += dx * cos(angle_rad) - dy * sin(angle_rad);
                y += dx * sin(angle_rad) + dy * cos(angle_rad);
            }
        }
    }
    double conversion_factor, angle_rad, angle;
    double x = 0, y = 0;
    bool kill_switch = false;
    int fd;
};

PYBIND11_MODULE(tracker, m) {
    py::class_<Gyro>(m, "Gyro")
        .def(py::init<char*, int, int, int>())
        .def("get", &Gyro::get)
        .def("get_x", &Gyro::get_x)
        .def("get_y", &Gyro::get_y)
        .def("get_z", &Gyro::get_z)
        .def("get_x_vel", &Gyro::get_x_vel)
        .def("get_y_vel", &Gyro::get_y_vel)
        .def("get_z_vel", &Gyro::get_z_vel)
        .def("get_ax", &Gyro::get_ax)
        .def("get_ay", &Gyro::get_ay)
        .def("get_az", &Gyro::get_az)
        .def("calibrate", &Gyro::calibrate)
        .def("reset", &Gyro::reset)
        .def("kill", &Gyro::kill)
        .def("get_speed", &Gyro::get_speed)
        .def_readwrite("bias_x", &Gyro::bias_x);
    
    py::class_<PositionTracker>(m, "PositionTracker")
        .def(py::init<char*, int, int, int, double>())
        .def("get", &PositionTracker::get)
        .def("get_x", &PositionTracker::get_x)
        .def("get_y", &PositionTracker::get_y)
        .def("kill", &PositionTracker::kill)
        .def("reset", &PositionTracker::reset)
        .def("get_angle", &PositionTracker::get_angle)
        .def("get_angle_rad", &PositionTracker::get_angle_rad)
        .def("get_gyro_z", &PositionTracker::get_gyro_z)
        .def_readwrite("gyro", &PositionTracker::gyro);
}

/*int main() {
    PositionTracker tracker("/dev/i2c-1", 0x69 - 1, 1, 0, 496.5);
    //Gyro gyro("/dev/i2c-1", 0x68, 1, 0);

    while (true) {
        //std::cout << gyro.get_z() << "\n";
        std::cout << tracker.get_x() << ", " << tracker.get_y() << " " << tracker.gyro.get_x() << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}*/