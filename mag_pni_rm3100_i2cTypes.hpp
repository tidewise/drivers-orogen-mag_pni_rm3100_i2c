#ifndef mag_pni_rm3100_i2c_TYPES_HPP
#define mag_pni_rm3100_i2c_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace mag_pni_rm3100_i2c {
    enum registerAddress {
        POLL_ADDR = 0x00,
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
}

#endif
