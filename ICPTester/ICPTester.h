#pragma once
#include <functional>
#include <memory>

#include "ICPTestGlobalDef.h"
#include "ICPResult.h"
#include "ICPConfig.h"

namespace ICPTest
{
	#define SPTR std::shared_ptr

	class IICP;
	class DataLoader;
	class ICPResult;
	class IVisualizator;
	class IFrame;

	enum EICPTestType
	{
		EICPTT_PCL,
		EICPTT_TRIMESH,
	};

	class ICPTester
	{
	public:
		typedef std::function<void(const ICPResult&)> PostICPFunc;
		typedef std::function<void(const IFrame&, const ICPMatrix4&)> UpdateNewFrameFunc;
		ICPTester(const char* pICPDataPath, EICPTestType ICPType);
		void DoICP();

	private:
		SPTR<IICP> m_pICPImpl;
		SPTR<DataLoader> m_pDataLoader;
		SPTR<IVisualizator> m_pVisualizator;
		SPTR<ICPConfig> m_pICPConfig;
		ICPResult m_ICPResult;
		PostICPFunc m_postICPCallback;
		UpdateNewFrameFunc m_updateCallback;
	};
}