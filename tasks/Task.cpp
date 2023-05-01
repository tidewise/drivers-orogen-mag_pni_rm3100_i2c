/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base-logging/Logging.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <base/Angle.hpp>
#include <bitset>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>

using namespace mag_pni_rm3100_i2c;

enum RegisterAddress {
    POLL_ADDR = 0x00,
    CMM_ADDR = 0x01,
    STATUS_ADDR = 0x34,
    HANDSHAKE_ADDR = 0x35,
    MEASUREMENT_X_AXIS_2_ADDR = 0x24,
    MEASUREMENT_X_AXIS_1_ADDR = 0x25,
    MEASUREMENT_X_AXIS_0_ADDR = 0x26,
    MEASUREMENT_Y_AXIS_2_ADDR = 0x27,
    MEASUREMENT_Y_AXIS_1_ADDR = 0x28,
    MEASUREMENT_Y_AXIS_0_ADDR = 0x29
};

Task::Task(std::string const& name)
    : TaskBase(name)
{
    _nwu_magnetic2nwu.set(base::Angle::fromRad(0));
}

Task::~Task()
{
}

static bool writeRegister(int fd,
    int i2c_address,
    uint8_t register_address,
    uint8_t* data,
    int size)
{
    std::vector<uint8_t> buffer;
    buffer.push_back(register_address);
    buffer.insert(buffer.end(), data, data + size);

    i2c_msg config_msg;
    config_msg.flags = 0;
    config_msg.addr = i2c_address;
    config_msg.len = buffer.size();
    config_msg.buf = buffer.data();

    i2c_rdwr_ioctl_data query;
    query.msgs = &config_msg;
    query.nmsgs = 1;
    if (ioctl(fd, I2C_RDWR, &query) == -1) {
        std::cerr << strerror(errno) << std::endl;
        return false;
    }

    return true;
}

static void readRegister(int fd,
    int i2c_address,
    uint8_t register_address,
    uint8_t* buffer,
    int size)
{
    i2c_msg msgs[2];

    msgs[0].flags = 0;
    msgs[0].addr = i2c_address;
    msgs[0].len = 1;
    msgs[0].buf = &register_address;

    msgs[1].flags = 1;
    msgs[1].addr = i2c_address;
    msgs[1].len = size;
    msgs[1].buf = buffer;

    i2c_rdwr_ioctl_data query;
    query.msgs = msgs;
    query.nmsgs = 2;
    if (ioctl(fd, I2C_RDWR, &query) == -1) {
        throw std::runtime_error(std::string("failed to read: ") + strerror(errno));
    }
}

static int32_t read_int24(uint8_t* msb)
{
    uint32_t uvalue = static_cast<uint32_t>(msb[0]) << 16 |
                      static_cast<uint32_t>(msb[1]) << 8 |
                      static_cast<uint32_t>(msb[2]) << 0;
    if (uvalue & 0x800000) { // Negative value, extend to 32bit
        uvalue |= 0xff000000;
    }

    return reinterpret_cast<int32_t&>(uvalue);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (!TaskBase::configureHook())
        return false;

    m_i2c_bus = _i2c_bus.get();
    m_i2c_address = _i2c_address.get();
    m_distortion = _magnetic_distortion_compensation.get();
    m_distortion_rot = Eigen::Rotation2D<float>(m_distortion.angle.getRad());
    m_distortion_rot_inverse = m_distortion_rot.inverse();

    m_fd = open(m_i2c_bus.c_str(), O_RDWR);
    if (m_fd == -1) {
        std::cerr << "Cannot open " << m_i2c_bus << " for reading and writing"
                  << std::endl;
        return false;
    }

    int ret = ioctl(m_fd, I2C_TIMEOUT, 10);
    if (ret == -1) {
        std::cerr << "timeout ioctl failed" << std::endl;
        close(m_fd);
        return false;
    }
    uint8_t cmm_value = 0;
    writeRegister(m_fd, m_i2c_address, RegisterAddress::CMM_ADDR, &cmm_value, 1);
    uint8_t poll_value = 0b01110000;
    writeRegister(m_fd, m_i2c_address, RegisterAddress::POLL_ADDR, &poll_value, 1);

    uint8_t handshake = 3;
    writeRegister(m_fd, m_i2c_address, RegisterAddress::HANDSHAKE_ADDR, &handshake, 1);
    handshake = 0;
    readRegister(m_fd, m_i2c_address, RegisterAddress::HANDSHAKE_ADDR, &handshake, 1);
    LOG_INFO_S << "handshake: " << std::hex << static_cast<int>(handshake) << std::endl;

    return true;
}
bool Task::startHook()
{
    if (!TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    uint8_t poll_value = 0b01110000;
    writeRegister(m_fd, m_i2c_address, RegisterAddress::POLL_ADDR, &poll_value, 1);

    while (true) {
        uint8_t status;
        readRegister(m_fd, m_i2c_address, RegisterAddress::STATUS_ADDR, &status, 1);
        if (status & 0x80) {
            break;
        }
    }

    uint8_t mag[9];
    readRegister(m_fd, m_i2c_address, RegisterAddress::MEASUREMENT_X_AXIS_2_ADDR, mag, 9);
    Eigen::Affine3d a;
    int32_t mag_x = read_int24(mag);
    int32_t mag_y = read_int24(mag + 3);
    // int32_t mag_z = read_int24(mag + 6);

    Eigen::Vector2f compensated_xy = compensateDistortion(Eigen::Vector2f(mag_x, mag_y));

    auto sensor2nwu_magnetic_heading =
        base::Angle::fromRad(atan2(compensated_xy.y(), compensated_xy.x()));
    auto sensor2nwu_heading = _nwu_magnetic2nwu.get() + sensor2nwu_magnetic_heading;

    base::samples::RigidBodyState rbs;
    rbs.time = base::Time::now();
    rbs.orientation =
        Eigen::AngleAxisd(sensor2nwu_heading.getRad(), Eigen::Vector3d::UnitZ());
    _sensor2nwu_yaw.write(rbs);
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    close(m_fd);
    m_fd = -1;
}

Eigen::Vector2f Task::compensateDistortion(Eigen::Vector2f const& measurement)
{
    // put center ellipse point
    Eigen::Vector2f p = measurement - m_distortion.center.cast<float>();

    // Compute the scaling factor to transform the ellipse to a circle
    Eigen::Vector2f scale(1.0f / m_distortion.major_axis, 1.0f / m_distortion.minor_axis);

    // Rotate the point
    p = m_distortion_rot * p;
    // Scale it
    p = p.array() * scale.array();
    // Rotate it back
    p = m_distortion_rot_inverse * p;
    return p;
}
