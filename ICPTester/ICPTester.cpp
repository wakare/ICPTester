#include "ICPTester.h"
#include "IICPImpl.h"
#include "DataLoader.h"
#include "ICPTestGlobalDef.h"
#include "Visualizator.h"
#include "Frame.h"

#include <iostream>
#include <windows.h>
#include <Eigen/Dense>

namespace ICPTest
{ 
	ICPTester::ICPTester(const char* pICPDataPath, EICPTestType ICPType)
	{
		// Init icp data
		m_pDataLoader = std::make_shared<DataLoader>(pICPDataPath, ICPType == EICPTT_PCL ? EDataLoaderType::EDLT_PCL : EDataLoaderType::EDLT_TRIMESH);
		
		switch (ICPType)
		{
		case ICPTest::EICPTT_PCL:
			m_pICPImpl = std::make_shared<ICPPCLImpl>();
			break;
		case ICPTest::EICPTT_TRIMESH:
			m_pICPImpl = std::make_shared<ICPTrimeshImpl>();
			break;
		default:
			throw "exception";
			break;
		}

		m_pVisualizator = std::make_shared<VisualizatorLeviathan>();
		m_pICPConfig = std::make_shared<ICPConfig>();

		m_updateCallback = [this](const IFrame& newFrame, const ICPMatrix4& transform)
		{
			m_pVisualizator->Update(newFrame, transform);
		};

		m_pICPConfig->m_maxCorrespondenceDistance = 6000.0f;
		m_pICPConfig->m_maxIterationCount = 200;
		m_pICPConfig->m_euclideanFitnessEpsilon = 1e-5;
		m_pICPConfig->m_transformationEpsilon = 1e-10;
		m_pICPConfig->m_useReciprocalCorrespondences = true;

		// config parameter error ?
		m_pICPImpl->ApplyICPConfig(*m_pICPConfig);
	}

	void ICPTester::DoICP()
	{
#ifdef DEBUG
		unsigned long long ullBeginTime = GetTickCount64();
		ICPTesterOutStream << "[DEBUG] ICP Begin." << std::endl;
#endif

		long long ICPTime = 0;

		for (unsigned currentFrameIndex = 0; currentFrameIndex < m_pDataLoader->FrameCount(); currentFrameIndex++)
		{ 
			auto& newFrame = m_pDataLoader->GetAFrame(currentFrameIndex);
			if (!newFrame.IsValid())
			{
				ICPTesterOutStream << "[INFO] Cannot get more frame, ICP exit";
				break;
			}

			ICPMatrix4 GlobalPose = ICPMatrix4::Identity();
			auto costtime = m_pICPImpl->DoICP(newFrame, GlobalPose);
			if (0 == costtime)
			{
				break;
			}

			m_ICPResult.AddFrameResult(std::make_pair(&newFrame, GlobalPose));

			ICPTime += costtime;

			if (m_updateCallback)
			{
				m_updateCallback(newFrame, GlobalPose);
			}

			if (m_postICPCallback)
			{
				m_postICPCallback(m_pICPImpl->GetICPResult());
			}
		}

		m_ICPResult = m_pICPImpl->GetICPResult();

		ICPTesterOutStream << "[PERFORMANCE] ICP total cost time: " << ICPTime << std::endl;
#ifdef DEBUG
		ICPTesterOutStream << "[DEBUG] ICP End, total cost time: " << GetTickCount64() - ullBeginTime << std::endl;
#endif
	}
}
