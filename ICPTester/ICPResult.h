#pragma once
#include "ICPTestGlobalDef.h"
#include <vector>
#include <utility>
#include "Frame.h"

namespace ICPTest
{
	typedef std::pair<const IFrame*, ICPMatrix4> FrameResult;

	class ICPResult
	{
	public:
		ICPResult();
		bool AddFrameResult(FrameResult fResult);
	private:
		std::vector<FrameResult> m_frameResultVec;
	};

	inline ICPResult::ICPResult()
	{

	}

	inline bool ICPResult::AddFrameResult(FrameResult fResult)
	{
		m_frameResultVec.push_back(fResult);
		return true;
	}
}