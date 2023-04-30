/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include "mag_pni_rm3100_i2cTypes.hpp"
#include <base-logging/Logging.hpp>
#include <base/samples/RigidBodyState.hpp>

using namespace mag_pni_rm3100_i2c;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::~Task()
{
}

static bool writeRegister(int fd,
    int mag_address,
    uint8_t register_address,
    uint8_t* data,
    int size)
{
    std::vector<uint8_t> buffer;
    buffer.push_back(register_address);
    buffer.insert(buffer.end(), data, data + size);

    i2c_msg config_msg;
    config_msg.flags = 0;
    config_msg.addr = mag_address;
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
    int mag_address,
    uint8_t register_address,
    uint8_t* buffer,
    int size)
{
    i2c_msg msgs[2];

    msgs[0].flags = 0;
    msgs[0].addr = mag_address;
    msgs[0].len = 1;
    msgs[0].buf = &register_address;

    msgs[1].flags = 1;
    msgs[1].addr = mag_address;
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

static Eigen::Vector2d ellipseToCircle(double minor_axis,
    double major_axis,
    Eigen::Rotation2D<double> rot,
    Eigen::Vector2d ellipse_center,
    Eigen::Vector2d ellipse_point)
{
    // put center ellipse point
    ellipse_point -= ellipse_center;

    // Compute the scaling factor to transform the ellipse to a circle
    Eigen::Vector2d scale(1 / major_axis, 1 / minor_axis);

    Eigen::Vector2d circle_point;
    // Rotate the point
    circle_point = rot * ellipse_point;
    // Scale it
    circle_point = circle_point.array() * scale.array();
    // Rotate it back
    circle_point = rot.inverse() * circle_point;
    return circle_point;
}

static base::samples::RigidBodyState convertToOrientationRBS(int32_t mag_x,
    int32_t mag_y,
    int32_t mag_z)
{
    base::samples::RigidBodyState rbs;
    rbs.time = base::Time::now();
    rbs.orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(atan2(mag_y, mag_x), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(atan2(mag_z, mag_x), // TODO verify the pitch
            Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(atan2(mag_z, mag_y), // TODO verify the roll
            Eigen::Vector3d::UnitX()));
    return rbs;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (!TaskBase::configureHook())
        return false;

    m_i2c_bus = _i2c_bus.get();
    m_mag_address = _mag_address.get();
    m_major_axis = _major_axis.get();
    m_minor_axis = _minor_axis.get();
    m_ellipse_angle = _ellipse_angle.get();
    m_ellipse_center = _ellipse_center.get();

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
    writeRegister(m_fd, m_mag_address, registerAddress::CMM_ADDR, &cmm_value, 1);
    uint8_t poll_value = 0b01110000;
    writeRegister(m_fd, m_mag_address, registerAddress::POLL_ADDR, &poll_value, 1);

    uint8_t handshake = 3;
    writeRegister(m_fd, m_mag_address, registerAddress::HANDSHAKE_ADDR, &handshake, 1);
    handshake = 0;
    readRegister(m_fd, m_mag_address, registerAddress::HANDSHAKE_ADDR, &handshake, 1);
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
    writeRegister(m_fd, m_mag_address, registerAddress::POLL_ADDR, &poll_value, 1);

    while (true) {
        uint8_t status;
        readRegister(m_fd, m_mag_address, registerAddress::STATUS_ADDR, &status, 1);
        if (status & 0x80) {
            break;
        }
    }

    uint8_t mag[9];
    readRegister(m_fd, m_mag_address, registerAddress::MEASUREMENT_X_AXIS_2_ADDR, mag, 9);
    Eigen::Affine3d a;
    int32_t mag_x = read_int24(mag);
    int32_t mag_y = read_int24(mag + 3);
    int32_t mag_z = read_int24(mag + 6);

    LOG_INFO_S << "Before calibration - mag_x:" << std::dec << mag_x << " mag_y:" << mag_y
               << std::endl;
    Eigen::Vector2d circle_point = ellipseToCircle(m_minor_axis,
        m_major_axis,
        Eigen::Rotation2D<double>(m_ellipse_angle.getRad()),
        m_ellipse_center,
        Eigen::Vector2d(mag_x, mag_y));
    LOG_INFO_S << "After calibration - mag_x:" << std::dec << circle_point[0]
               << " mag_y:" << circle_point[1] << std::endl;
    LOG_INFO_S << "Final calculated yaw : " << atan2(circle_point[1], circle_point[0])
               << std::endl;

    auto rbs_reference = convertToOrientationRBS(mag_x, mag_y, mag_z);
    _mag2nwu_orientation.write(rbs_reference);
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
