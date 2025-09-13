#pragma once
#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>

namespace camodocal {

// Ceres 1.x 风格的四元数局部参数化
class EigenQuaternionParameterization : public ceres::LocalParameterization {
public:
	virtual ~EigenQuaternionParameterization() {}

	virtual bool Plus(const double* x,
										const double* delta,
										double* x_plus_delta) const;

	virtual bool ComputeJacobian(const double* x,
															 double* jacobian) const;

	virtual int GlobalSize() const { return 4; }
	virtual int LocalSize() const { return 3; }
};

}  // namespace camodocal

