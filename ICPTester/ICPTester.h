#pragma once
#include <functional>
#include <memory>

#include "ICPTestGlobalDef.h"
#include "ICPErrorEstimator.h"
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
		ICPTester(const char* pICPDataPath, EICPTestType ICPType, int startIndex = -1, int endIndex = -1);
		void DoICP(int startIndex = -1, int endIndex = -1);
		bool ExportICPResultToFile(const char* outputFilePath);

	private:
		bool _exportPLYFile(std::ofstream& outFile);
		bool _exportTransformFile(std::ofstream& outFile);

		SPTR<IICP> m_pICPImpl;
		SPTR<DataLoader> m_pDataLoader;
		SPTR<IVisualizator> m_pVisualizator;
		SPTR<ICPConfig> m_pICPConfig;
		SPTR<ICPErrorEstimator> m_pErrorEstimator;
		const char* m_correctResultFilePath;
		ICPResult m_ICPResult;
		PostICPFunc m_postICPCallback;
		UpdateNewFrameFunc m_updateCallback;
	};
}