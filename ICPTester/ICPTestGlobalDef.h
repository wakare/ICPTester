#pragma once
#include "GlobalDef.h"
#include <iostream>
#include <Eigen/Dense>

namespace ICPTest
{
	#define ICPTesterOutStream std::cout
	#define ICPScalar double
	/*typedef Eigen::Matrix4f ICPMatrix4;*/
	/*typedef Eigen::Vector4f ICPVector4;*/
	typedef Eigen::Matrix<ICPScalar, 4, 4> ICPMatrix4;
	typedef Eigen::Matrix<ICPScalar, 3, 3> ICPMatrix3;
	typedef Eigen::Matrix<ICPScalar, 4, 1> ICPVector4;
	typedef Eigen::Matrix<ICPScalar, 3, 1> ICPVector3;
}
