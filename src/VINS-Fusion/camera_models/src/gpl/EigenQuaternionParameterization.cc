#include "camodocal/gpl/EigenQuaternionParameterization.h"

#include <cmath>
#include <algorithm>

namespace {
// 右手系四元数乘法：z = y * x（与原库一致的顺序）
inline void EigenQuaternionProduct(const double y[4],
                                                                     const double x[4],
                                                                     double z[4]) {
    z[0] =  y[3] * x[0] + y[0] * x[3] + y[1] * x[2] - y[2] * x[1];
    z[1] =  y[3] * x[1] - y[0] * x[2] + y[1] * x[3] + y[2] * x[0];
    z[2] =  y[3] * x[2] + y[0] * x[1] - y[1] * x[0] + y[2] * x[3];
    z[3] =  y[3] * x[3] - y[0] * x[0] - y[1] * x[1] - y[2] * x[2];
}
}  // namespace

namespace camodocal {

bool EigenQuaternionParameterization::Plus(const double* x,
                                                                                     const double* delta,
                                                                                     double* x_plus_delta) const {
    const double norm_delta = std::sqrt(delta[0] * delta[0] +
                                                                            delta[1] * delta[1] +
                                                                            delta[2] * delta[2]);
    if (norm_delta > 0.0) {
        const double sin_delta_by_delta = std::sin(norm_delta) / norm_delta;
        double q_delta[4];
        q_delta[0] = sin_delta_by_delta * delta[0];
        q_delta[1] = sin_delta_by_delta * delta[1];
        q_delta[2] = sin_delta_by_delta * delta[2];
        q_delta[3] = std::cos(norm_delta);
        EigenQuaternionProduct(q_delta, x, x_plus_delta);
    } else {
        std::copy(x, x + 4, x_plus_delta);
    }
    return true;
}

bool EigenQuaternionParameterization::ComputeJacobian(const double* x,
                                                                                                            double* jacobian) const {
    // 行优先 4x3 Jacobian，在 delta=0 处
    jacobian[0]  =  x[3]; jacobian[1]  =  x[2]; jacobian[2]  = -x[1];
    jacobian[3]  = -x[2]; jacobian[4]  =  x[3]; jacobian[5]  =  x[0];
    jacobian[6]  =  x[1]; jacobian[7]  = -x[0]; jacobian[8]  =  x[3];
    jacobian[9]  = -x[0]; jacobian[10] = -x[1]; jacobian[11] = -x[2];
    return true;
}

}  // namespace camodocal
