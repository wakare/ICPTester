#include "ICPErrorEstimator.h"

namespace ICPTest
{
	ICPErrorEstimator::ICPErrorEstimator(EErrorEstimateType type /*= EEET_USER_DEFINE*/):
		m_errorEstimateType(type)
	{

	}

	bool ICPErrorEstimator::Estimate(const ICPResult& icpResult, const char* correctResultFile)
	{
		std::ofstream outEstimateFile("Estimate", std::ios::out);
		std::ifstream correctResult(correctResultFile, std::ios::in);

		EXIT_GET_FALSE(outEstimateFile.is_open());
		EXIT_GET_FALSE(correctResult.is_open());

		const auto& icpResultVec = icpResult.GetICPResultData();
		for (unsigned i = 0; i < icpResultVec.size(); i++)
		{
			ICPMatrix4 icpMatrix = icpResultVec[i].second.transpose();
			ICPMatrix4 correctMatrix;
			_getMatrix(correctResult, correctMatrix);

			float RError, TError;
			_estimate(icpMatrix, correctMatrix, RError, TError);

			outEstimateFile << "Frame index: " << i << ",RError: " << RError << ", TError: " << TError << std::endl;
		}

		outEstimateFile.close();
		correctResult.close();
		ICPTesterOutStream << "[INFO] Calculate transform error finish." << std::endl;

		return true;
	}

	void ICPErrorEstimator::_getMatrix(std::ifstream& stream, ICPMatrix4& outMatrix)
	{
		ICPScalar matrix[16];
		for (unsigned i = 0; i < 16; i++)
		{
			stream >> matrix[i];
		}

		outMatrix = ICPMatrix4(matrix);
	}

	void ICPErrorEstimator::_estimate(const ICPMatrix4& input, const ICPMatrix4& correct, float& outRotationError, float& outTranslationError)
	{
		ICPMatrix4 diff = input.inverse() * correct;
		ICPMatrix3 diff_rotation = diff.block<3, 3>(0, 0);
		ICPVector3 diff_translation = diff.block<1, 3>(3, 0);
		ICPVector3 euler_angles = diff_rotation.eulerAngles(2, 1, 0);

		outRotationError = euler_angles.mean();
		outTranslationError = diff_translation.norm();
	}

}