#pragma once
#include "RenderService.h"
#include "PointCloud.h"
#include "CCommonScene.h"
#include <windows.h>

using namespace Leviathan;

void _addRandomPointCloud(RenderService& service)
{
	auto _addPointCloudRequest = [&service]()
	{
		constexpr int uCount = 300;
		float testPointCoord[3 * uCount];
		float testPointNormal[3 * uCount];

		constexpr float fMaxRange = 100.0f;

		for (unsigned i = 0; i < uCount; i++)
		{
			testPointCoord[3 * i] = RANDOM_0To1 * fMaxRange;
			testPointCoord[3 * i + 1] = RANDOM_0To1 * fMaxRange;
			testPointCoord[3 * i + 2] = RANDOM_0To1 * fMaxRange;

			testPointNormal[3 * i] = 1.0f;
			testPointNormal[3 * i + 1] = 0.0f;
			testPointNormal[3 * i + 2] = 0.0f;
		}

		PointCloudf points(uCount, testPointCoord, testPointNormal);
		service.GetScene()->UpdatePointCloud(points);
	};

	service.GetScene()->PushDataUpdateRequest(_addPointCloudRequest);
}

void ThreadAddPointCloud()
{
	unsigned uLoopCount = 1;
	unsigned uCurrentLoopCount = 0;
	while (true)
	{
		Sleep(2000);

		_addRandomPointCloud(*RenderService::Instance());
		if (uCurrentLoopCount++ > uLoopCount)
		{
			//break;
		}
	}
}

void ThreadRender()
{
	RenderService::Instance()->Run();
}

bool Run()
{
	RenderService::Instance()->GetScene();

	std::thread renderThread(ThreadAddPointCloud);
	renderThread.detach();

	ThreadRender();

	return EXIT_SUCCESS;
}