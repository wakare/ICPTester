#pragma once
#include "GlobalDef.h"
#include <iostream>
#include <Eigen/Dense>

namespace ICPTest
{
	#define ICPTesterOutStream std::cout
	typedef Eigen::Matrix4f ICPMatrix4;
	typedef Eigen::Vector4f ICPVector4;
}
