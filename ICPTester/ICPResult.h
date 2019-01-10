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
		const std::vector<FrameResult>& GetICPResultData() const;
	
		unsigned m_totalVertexCount;
	
	private:
		std::vector<FrameResult> m_frameResultVec;
	};

	inline ICPResult::ICPResult():
		m_totalVertexCount(0)
	{

	}

	inline bool ICPResult::AddFrameResult(FrameResult fResult)
	{
		m_frameResultVec.push_back(fResult);
		m_totalVertexCount += fResult.first->VertexCount();
		return true;
	}


	inline const std::vector<ICPTest::FrameResult>& ICPResult::GetICPResultData() const
	{
		return m_frameResultVec;
	}

}