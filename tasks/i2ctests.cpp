/** Standalone test program
 */

#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <cstdint>
#include <iostream>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <chrono>
#include <bitset>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

static constexpr int ADDRESS = 0x20;
static const string BUS = "/dev/i2c-2";

enum registerAddress {
    POLL_ADDR  = 0x00,
    CMM_ADDR = 0x01,
    STATUS_ADDR = 0x34,
    HANDSHAKE_ADDR = 0x35,
    MEASUREMENT_X_AXIS_2_ADDR = 0x24,
    MEASUREMENT_X_AXIS_1_ADDR = 0x25,
    MEASUREMENT_X_AXIS_0_ADDR = 0x26,
    MEASUREMENT_Y_AXIS_2_ADDR = 0x27,
    MEASUREMENT_Y_AXIS_1_ADDR = 0x28,
    MEASUREMENT_Y_AXIS_0_ADDR = 0x29,
};

bool writeRegister(int fd, uint8_t register_adress, uint8_t* data, int size);
void readRegister(int fd, uint8_t register_address, uint8_t* buffer, int size);
Eigen::Vector2d ellipseToCircle(double major_axis, double minor_axis, double angle,
                                Eigen::Vector2d ellipse_center, Eigen::Vector2d ellipse_point);

int32_t read_int24(uint8_t* msb);

int main(int argc, char** argv) {
    int fd = open(BUS.c_str(), O_RDWR);
    if (fd == -1) {
        cerr
            << "Cannot open " << BUS
            << " for reading and writing" << endl;
        return 1;
    }

    int ret = ioctl(fd, I2C_TIMEOUT, 10);
    if (ret == -1) {
	cerr << "timeout ioctl failed" << endl;
        close(fd);
        return 1;
    }

    uint8_t cmm_value = 0;
    writeRegister(fd, registerAddress::CMM_ADDR, &cmm_value, 1);
    uint8_t poll_value = 0b01110000;
    writeRegister(fd, registerAddress::POLL_ADDR, &poll_value, 1);

    uint8_t handshake = 3;
    writeRegister(fd, registerAddress::HANDSHAKE_ADDR, &handshake, 1);
    handshake = 0;
    readRegister(fd, registerAddress::HANDSHAKE_ADDR, &handshake, 1);
    std::cout << "handshake: " << hex << static_cast<int>(handshake) << std::endl;

    while(true) {
        uint8_t poll_value = 0b01110000;
        writeRegister(fd, registerAddress::POLL_ADDR, &poll_value, 1);

        while(true) {
            uint8_t status;
            readRegister(fd, registerAddress::STATUS_ADDR, &status, 1);
            if (status & 0x80) {
                break;
            }
        }

        uint8_t mag[9];
        readRegister(fd, registerAddress::MEASUREMENT_X_AXIS_2_ADDR, mag, 9);
        Eigen::Affine3d a;
        int32_t mag_x = read_int24(mag);
        int32_t mag_y = read_int24(mag + 3);
        int32_t mag_z = read_int24(mag + 6);

        std::cout << "Before calib - mag_x:"<< dec << mag_x << " mag_y:" << mag_y << std::endl;
        double major_axis = 5994.242;
        double minor_axis = 4381.799; 
        double angle = 0.406267; 
        Eigen::Vector2d ellipse_center(6769.717, -2303.238); 
        Eigen::Vector2d ellipse_point(mag_x, mag_y) ; 
        Eigen::Vector2d circle_point = ellipseToCircle(minor_axis, major_axis,angle, ellipse_center, ellipse_point);
        std::cout << "After calib - mag_x:"<< dec << circle_point[0] << " mag_y:" << circle_point[1] << std::endl;
        std::cout << "Final yaw : " << atan2(circle_point[1],circle_point[0]) << std::endl;
        //std::cout << dec << circle_point[0] << " " << mag_z << " " << circle_point[1] << " " << atan2(circle_point[1], circle_point[0]) * 180 / M_PI << std::endl;
        usleep(10000);
    }

    return 0;
}

bool writeRegister(int fd, uint8_t register_address, uint8_t* data, int size) {
    std::vector<uint8_t> buffer;
    buffer.push_back(register_address);
    buffer.insert(buffer.end(), data, data + size);

    i2c_msg config_msg;
    config_msg.flags = 0;
    config_msg.addr = ADDRESS;
    config_msg.len = buffer.size();
    config_msg.buf = buffer.data();

    i2c_rdwr_ioctl_data query;
    query.msgs = &config_msg;
    query.nmsgs = 1;
    if (ioctl(fd, I2C_RDWR, &query) == -1) {
        cerr << strerror(errno) << std::endl;
        return false;
    }

    return true;
}

void readRegister(int fd, uint8_t register_address, uint8_t* buffer, int size) {
    i2c_msg msgs[2];

    msgs[0].flags = 0;
    msgs[0].addr = ADDRESS;
    msgs[0].len = 1;
    msgs[0].buf = &register_address;

    uint8_t result[2];
    msgs[1].flags = 1;
    msgs[1].addr = ADDRESS;
    msgs[1].len = size;
    msgs[1].buf = buffer;

    i2c_rdwr_ioctl_data query;
    query.msgs = msgs;
    query.nmsgs = 2;
    if (ioctl(fd, I2C_RDWR, &query) == -1) {
        throw std::runtime_error(string("failed to read: ") + strerror(errno));
    }
}

int32_t read_int24(uint8_t* msb) {
    uint32_t uvalue =
        static_cast<uint32_t>(msb[0]) << 16 |
        static_cast<uint32_t>(msb[1]) << 8 |
        static_cast<uint32_t>(msb[2]) << 0;
    if (uvalue & 0x800000) { // Negative value, extend to 32bit
        uvalue |= 0xff000000;
    }

    return reinterpret_cast<int32_t&>(uvalue);
}


Eigen::Vector2d ellipseToCircle(double minor_axis,double major_axis, double angle,
                                Eigen::Vector2d ellipse_center, Eigen::Vector2d ellipse_point)
{
    //put center ellipse point
    ellipse_point-=ellipse_center;

    // Compute the scaling factor to transform the ellipse to a circle
    Eigen::Vector2d scale(1/major_axis, 1/minor_axis);


    Eigen::Vector2d circle_point;
    // Rotate the point
    circle_point = Eigen::Rotation2D<double>(angle) * ellipse_point ;
    // Scale it
    circle_point =   circle_point.array() * scale.array(); 
    // Rotate it back
    circle_point = Eigen::Rotation2D<double>(-angle) * circle_point;
    return circle_point;
}