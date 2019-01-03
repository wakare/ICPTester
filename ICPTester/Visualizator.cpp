#include "Visualizator.h"
#include "RenderService.h"
#include "CCommonScene.h"
#include "DynamicArray.h"
#include <windows.h>

using namespace Leviathan;

namespace ICPTest
{
	VisualizatorLeviathan::VisualizatorLeviathan()
	{
		// Max size is 10000 frames.
		m_pointClouds.reserve(10000);
	}

	void VisualizatorLeviathan::Update(const IFrame& newFrame, const ICPMatrix4& transform)
	{
		static unsigned uLoopCount = 0;
		ICPTesterOutStream << "[DEBUG] Update frame, size = " << newFrame.VertexCount() << ", frame index = " << uLoopCount++ << std::endl;  
		
		// Convert to Leviathan point cloud.
		_addFrameToPointCloudVec(newFrame, transform);
	}

	void VisualizatorLeviathan::Init()
	{
		RenderService::Instance()->GetScene();
	}

	void VisualizatorLeviathan::RunRenderThread()
	{
		RenderService::Instance()->Run();
	}

	bool VisualizatorLeviathan::_addPointCloud(RenderService& service, PointCloudf& refPointCloud)
	{
		auto _addPointCloudRequest = [&service, &refPointCloud]()
		{
			service.GetScene()->UpdatePointCloud(refPointCloud);
		};

		EXIT_GET_FALSE(service.GetScene()->PushDataUpdateRequest(_addPointCloudRequest));

		return true;
	}

	bool VisualizatorLeviathan::_addFrameToPointCloudVec(const IFrame& frame, const ICPMatrix4& globalPose)
	{
		DynamicArray<float> tempVertexCoordArray(frame.VertexCount() * 3 * sizeof(float));
		DynamicArray<float> tempVertexNormalArray(frame.VertexCount() * 3 * sizeof(float));
		/*const auto& points = *frame.GetPoints();*/

		for (unsigned i = 0; i < frame.VertexCount(); i++)
		{
			float coord[3];
			float normalCoord[3];

			EXIT_GET_FALSE(frame.GetFrameData(i, coord, normalCoord));

			float localCoord[4]  = { coord[0], coord[1], coord[2], 1.0f};
			float localNormal[4] = { normalCoord[0], normalCoord[1], normalCoord[2], 1.0f };

			ICPVector4 coordVec(localCoord);
			ICPVector4 normalVec(localNormal);

			auto globalCoord = globalPose * coordVec;
			auto globalNormal = globalPose.inverse() * normalVec;

			float worldCoord[3] = { globalCoord.x(), globalCoord.y(), globalCoord.z() };
			float worldNormal[3] = { globalNormal.x(), globalNormal.y(), globalNormal.z() };

			memcpy(tempVertexCoordArray.m_pData + 3 * i, worldCoord, 3 * sizeof(float));
			memcpy(tempVertexNormalArray.m_pData + 3 * i, worldNormal, 3 * sizeof(float));
		}

		m_pointClouds.push_back(std::move(PointCloudf(frame.VertexCount(), tempVertexCoordArray.m_pData , tempVertexNormalArray.m_pData )));
		EXIT_GET_FALSE(_addPointCloud(*RenderService::Instance(), m_pointClouds.back()));

		return true;
	}

}