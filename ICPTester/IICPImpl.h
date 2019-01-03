#pragma once
#include "IICP.h"
#include "Frame.h"
#include "ICPResult.h"

#define BOOST_TYPEOF_EMULATION
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>

namespace ICPTest
{
	class ICPPCLImpl : public IICP
	{
	public:
		ICPPCLImpl();

		unsigned DoICP(const IFrame& newFrame, ICPMatrix4& outPose);
		
		void ApplyICPConfig(ICPConfig& config);
		const ICPResult& GetICPResult();

	private:
		bool _setFrameData(const IFrame& frame, ICPSetDataType type);
		bool _setFrameData(pcl::PointCloud<pcl::PointNormal>::Ptr pFrame, ICPSetDataType type);

		pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal>::Ptr m_pPCLICP;
		ICPResult m_result;
	};

	class ICPTrimeshImpl : public IICP
	{
	public:
		unsigned DoICP(const IFrame& newFrame, ICPMatrix4& outPose);
		void ApplyICPConfig(ICPConfig& config);
		const ICPResult& GetICPResult();

	private:
		trimesh::xform _ConvertICPMatrixToXForm(const ICPMatrix4& input) const;
		ICPMatrix4 _ConvertXFormToICPMatrix(const trimesh::xform& input) const;

		ICPResult m_result;
	};
}