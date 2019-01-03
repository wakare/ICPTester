#include "IICPImpl.h"
#include "ICPTestGlobalDef.h"
#include "ICP.h"
#include <chrono>

namespace ICPTest
{
	typedef std::chrono::high_resolution_clock ICPClock;

	ICPPCLImpl::ICPPCLImpl()
	{
		 m_pPCLICP.reset(new pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal>);
		// Init ICP config
	}

	unsigned ICPPCLImpl::DoICP(const IFrame& newFrame, ICPMatrix4& outPose)
	{
		static pcl::PointCloud<pcl::PointNormal>::Ptr lastFrame = nullptr;
		static ICPMatrix4 lastPose = ICPMatrix4::Identity();
		long long costtime = 1;

		if (lastFrame)
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr pcd_aligned(new pcl::PointCloud<pcl::PointNormal>);

			// Do ICP
			_setFrameData(newFrame, ICPSDT_SOURCE);
			_setFrameData(lastFrame, ICPSDT_TARGET);

			// Profile
			auto now = ICPClock::now();
			m_pPCLICP->align(*pcd_aligned, ICPMatrix4::Identity());
			costtime = std::chrono::duration_cast<std::chrono::microseconds>(ICPClock::now() - now).count();

			outPose = lastPose * m_pPCLICP->getFinalTransformation();
			lastFrame = pcd_aligned;
			lastPose = outPose;
		}
		
		else
		{
			outPose = ICPMatrix4::Identity();
			lastFrame = dynamic_cast<const PCLFrame&>(newFrame).GetPoints();
		}

		return costtime;
	}

	bool ICPPCLImpl::_setFrameData(const IFrame& frame, ICPSetDataType type)
	{
		const PCLFrame& pclFrame = dynamic_cast<const PCLFrame&>(frame);
		EXIT_GET_FALSE((&pclFrame));

		(type == ICPSDT_SOURCE) ?  m_pPCLICP->setInputSource(pclFrame.GetPoints()) : m_pPCLICP->setInputTarget(pclFrame.GetPoints());

		return true;
	}

	bool ICPPCLImpl::_setFrameData(pcl::PointCloud<pcl::PointNormal>::Ptr pFrame, ICPSetDataType type)
	{
		(type == ICPSDT_SOURCE) ? m_pPCLICP->setInputSource(pFrame) : m_pPCLICP->setInputTarget(pFrame);

		return true;
	}

	void ICPPCLImpl::ApplyICPConfig(ICPConfig& config)
	{
		m_pPCLICP->setMaxCorrespondenceDistance(config.m_maxCorrespondenceDistance);
		m_pPCLICP->setMaximumIterations(config.m_maxIterationCount);
		m_pPCLICP->setTransformationEpsilon(config.m_transformationEpsilon);
		m_pPCLICP->setEuclideanFitnessEpsilon(config.m_euclideanFitnessEpsilon);
		m_pPCLICP->setUseReciprocalCorrespondences(config.m_useReciprocalCorrespondences);
	}

	const ICPResult& ICPTest::ICPPCLImpl::GetICPResult()
	{
		return m_result;
	}

	unsigned ICPTrimeshImpl::DoICP(const IFrame& newFrame, ICPMatrix4& outPose)
	{
		static const TriMeshFrame* lastFrame = nullptr;
		static ICPMatrix4 lastPose = ICPMatrix4::Identity();
		long long costtime = 1;

		const TriMeshFrame& newTrimeshFrame = dynamic_cast<const TriMeshFrame&>(newFrame);

		if (lastFrame)
		{
			trimesh::xform outTransform;

			auto now = ICPClock::now();
			if (trimesh::ICP(lastFrame->GetTriMesh(), newTrimeshFrame.GetTriMesh(), trimesh::xform(), outTransform, 0) < 0.0f)
			{
				ICPTesterOutStream << "[ERROR] ICP failed." << std::endl;
				return 0;
			}
			costtime = std::chrono::duration_cast<std::chrono::microseconds>(ICPClock::now() - now).count();

			outPose = lastPose * _ConvertXFormToICPMatrix(outTransform);
			lastPose = outPose;
		}

		else
		{
			outPose = ICPMatrix4::Identity();
		}

		lastFrame = &newTrimeshFrame;
		return costtime;
	}

	void ICPTrimeshImpl::ApplyICPConfig(ICPConfig& config)
	{
		// Do nothing
	}

	const ICPResult& ICPTrimeshImpl::GetICPResult()
	{
		return m_result;
	}

	trimesh::xform ICPTrimeshImpl::_ConvertICPMatrixToXForm(const ICPMatrix4& input) const
	{
		trimesh::xform result;

		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				result(i, j) = input(i, j);
			}
		}

		return result;
	}

	ICPTest::ICPMatrix4 ICPTrimeshImpl::_ConvertXFormToICPMatrix(const trimesh::xform& input) const
	{
		ICPTest::ICPMatrix4 result;

		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				result(i, j) = input(i, j);
			}
		}

		return result;
	}

}