#pragma once

#include "ICPTestGlobalDef.h"
#include "ICPConfig.h"

namespace ICPTest
{
	class ICPResult;
	class IFrame;

	enum ICPSetDataType
	{
		ICPSDT_SOURCE,
		ICPSDT_TARGET,
	};

	class IICP
	{
	public:
		virtual unsigned DoICP(const IFrame& newFrame, ICPMatrix4& outPose) = 0;
		virtual void ApplyICPConfig(ICPConfig& config) = 0;
		virtual const ICPResult& GetICPResult() = 0;
	};
}