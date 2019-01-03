#pragma once
#include "Frame.h"
#include "PointCloud.h"
#include "ICPTestGlobalDef.h"
#include "LPtr.h"

namespace Leviathan
{
	class RenderService;
}

using namespace Leviathan;

namespace ICPTest
{
	class IVisualizator
	{
	public:
		virtual void Update(const IFrame&, const ICPMatrix4&) = 0;
	};

	class VisualizatorLeviathan : public IVisualizator
	{
	public:
		VisualizatorLeviathan();
		void Update(const IFrame&, const ICPMatrix4&);

		void Init();
		void RunRenderThread();

	private:
		bool _addPointCloud(RenderService& service, PointCloudf& refPointCloud);
		bool _addFrameToPointCloudVec(const IFrame& frame, const ICPMatrix4& globalPose);

		std::vector<PointCloudf> m_pointClouds;
	};
}
