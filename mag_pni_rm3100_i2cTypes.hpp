#ifndef mag_pni_rm3100_i2c_TYPES_HPP
#define mag_pni_rm3100_i2c_TYPES_HPP

#include <base/Angle.hpp>
#include <base/Float.hpp>

namespace mag_pni_rm3100_i2c {
    /** Calibration data
     *
     * Magnetic calibration is done by doing a 360 degree rotation along the
     * X/Y plane and fitting an ellipse to the resulting mag-x/mag-y plot
     *
     * The component only compensates for distortions in X/Y
     */
    struct MagneticDistortionCompensationConfig {
        /** The major_axis of the ellipse before calibration */
        float major_axis = 1;
        /** The minor_axis of the ellipse before calibration */
        float minor_axis = 1;
        /** TBD */
        base::Angle angle = base::Angle::fromRad(0);
        /** The center of the ellipse before calibration */
        base::Vector2d center = base::Vector2d(0, 0);
    };
}

#endif
