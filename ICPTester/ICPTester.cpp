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
	ICPTester::ICPTester(const char* pICPDataPath, EICPTestType ICPType, int startIndex /*= -1*/, int endIndex /*= -1*/):
		m_correctResultFilePath(/*nullptr*/ "Transform_pcl_200.dat")
	{
		// Init icp data
		m_pDataLoader = std::make_shared<DataLoader>(pICPDataPath, ICPType == EICPTT_PCL ? EDataLoaderType::EDLT_PCL : EDataLoaderType::EDLT_TRIMESH, startIndex, endIndex);
		
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
		m_pErrorEstimator = std::make_shared<ICPErrorEstimator>();

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
		//m_pICPImpl->ApplyICPConfig(*m_pICPConfig);
	}

	void ICPTester::DoICP(int startIndex /*= -1*/, int endIndex /*= -1*/)
	{
#ifdef DEBUG
		unsigned long long ullBeginTime = GetTickCount64();
		ICPTesterOutStream << "[DEBUG] DoICP Begin." << std::endl;
#endif

		long long ICPTime = 0;
		startIndex = (startIndex > 0) ? startIndex : 0;
		endIndex = (endIndex > 0) ? endIndex : m_pDataLoader->FrameCount();

		for (unsigned currentFrameIndex = startIndex; currentFrameIndex < endIndex; currentFrameIndex++)
		{ 
			auto& newFrame = m_pDataLoader->GetAFrame(currentFrameIndex);
			if (!newFrame.IsValid())
			{
				ICPTesterOutStream << "[INFO] Cannot get more frame, ICP exit";
				break;
			}

			ICPMatrix4 GlobalPose = ICPMatrix4::Identity();
			auto costtime = m_pICPImpl->DoICP(newFrame, GlobalPose);
			if (0 == costtime)  break; 

			m_ICPResult.AddFrameResult(std::make_pair(&newFrame, GlobalPose));
			ICPTime += costtime;

			if (m_updateCallback) m_updateCallback(newFrame, GlobalPose); 
			if (m_postICPCallback) m_postICPCallback(m_pICPImpl->GetICPResult()); 
		}

		if (m_correctResultFilePath)
		{
			if (!m_pErrorEstimator->Estimate(m_ICPResult, m_correctResultFilePath))
			{
				throw "exception";
			}
		}

		ICPTesterOutStream << "[PERFORMANCE] ICP total cost time: " << ICPTime << ", startIndex = " << startIndex << " , endIndex" << endIndex<<  std::endl;
#ifdef DEBUG
		ICPTesterOutStream << "[DEBUG] DoICP End, total cost time: " << GetTickCount64() - ullBeginTime << std::endl;
#endif
	}

	bool ICPTester::ExportICPResultToFile(const char* outputFilePath)
	{
		std::ofstream outFile(outputFilePath, std::ios::out);
		EXIT_GET_FALSE(outFile.is_open());
		EXIT_GET_FALSE(_exportPLYFile(outFile));
		outFile.close();
		ICPTesterOutStream << "[INFO] Export ICP result ply finished." << std::endl;

		std::ofstream outTransfromFile("Transform.dat");
		EXIT_GET_FALSE(outTransfromFile.is_open());
		EXIT_GET_FALSE(_exportTransformFile(outTransfromFile));
		outTransfromFile.close();
		ICPTesterOutStream << "[INFO] Export ICP result transform finished." << std::endl;

		return true;
	}

	bool ICPTester::_exportPLYFile(std::ofstream& outFile)
	{
		// Use trimesh export ply
		float fScale = 0.001f;

		outFile << "ply\nformat ascii 1.0\n";
		outFile << "element vertex " << m_ICPResult.m_totalVertexCount << std::endl;
		outFile << "property int	camera_id\n";
		outFile << "property float x\n";
		outFile << "property float y\n";
		outFile << "property float z\n";
		outFile << "property float nx\n";
		outFile << "property float ny\n";
		outFile << "property float nz\n";
		outFile << "end_header\n";

		const auto& frames = m_ICPResult.GetICPResultData();

		for (auto& frameResult : frames)
		{
			auto& frame = *frameResult.first;

			for (unsigned i = 0; i < frame.VertexCount(); i++)
			{
				float coord[3];
				float normalCoord[3];

				EXIT_GET_FALSE(frame.GetFrameData(i, coord, normalCoord));

				ICPScalar localCoord[4] = { coord[0], coord[1], coord[2], 1.0f };
				ICPScalar localNormal[4] = { normalCoord[0], normalCoord[1], normalCoord[2], 1.0f };

				ICPVector4 coordVec(localCoord);
				ICPVector4 normalVec(localNormal);

				auto globalCoord = frameResult.second * coordVec;
				// Get rotation
				ICPMatrix4 rotation(frameResult.second);
				rotation(0, 3) = 0.0f;
				rotation(3, 0) = 0.0f;
				rotation(1, 3) = 0.0f;
				rotation(3, 1) = 0.0f;
				rotation(2, 3) = 0.0f;
				rotation(3, 2) = 0.0f;

				auto globalNormal = rotation.inverse() * normalVec;

				// Camera index
				outFile << "0";
				outFile << " " << globalCoord[0] * fScale << " " << globalCoord[1] * fScale << " " << globalCoord[2] * fScale;
				outFile << " " << globalNormal[0] << " " << globalNormal[1] << " " << globalNormal[2] << std::endl;
			}
		}

		return true;
	}

	bool ICPTester::_exportTransformFile(std::ofstream& outFile)
	{
		const auto& frames = m_ICPResult.GetICPResultData();
		for (auto frame : frames)
		{
			outFile << frame.second;
		}

		return true;
	}

}
