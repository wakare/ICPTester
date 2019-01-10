#pragma once
#include "ICPTestGlobalDef.h"
#include "ICPResult.h"

namespace ICPTest
{
	enum EErrorEstimateType
	{
		EEET_USER_DEFINE,
		EEET_UNKNOWN
	};

	class ICPErrorEstimator
	{
	public:
		ICPErrorEstimator(EErrorEstimateType type = EEET_USER_DEFINE);
		bool Estimate(const ICPResult& icpResult, const char* correctResultFile);

	private:
		void _getMatrix(std::ifstream& stream, ICPMatrix4& outMatrix);
		void _estimate(const ICPMatrix4& input, const ICPMatrix4& correct, float& outRotationError, float& outTranslationError);
		EErrorEstimateType m_errorEstimateType;
	};
}