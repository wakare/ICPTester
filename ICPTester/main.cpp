#include "ICPTester.h"
#include "Visualizator.h"
#include <windows.h>
#include <thread>

using namespace ICPTest;
using namespace Leviathan;

void ThreadDoICP(const char* DataFolder, const char* icpType)
{
	EICPTestType type;

	if (0 == strcmp(icpType, "pcl"))
	{
		type = EICPTT_PCL;
	}

	if (0 == strcmp(icpType, "trimesh"))
	{
		type = EICPTT_TRIMESH;
	}

	ICPTester tester(DataFolder, type);
	tester.DoICP();
	
	// Remain resources for a while 
	Sleep(100000);
}

int main(int argc, char** argv)
{
	if (argc < 3)
	{
		return EXIT_FAILURE;
	}

	VisualizatorLeviathan renderThread;
	renderThread.Init();

	std::thread icpthread(ThreadDoICP, argv[1], argv[2]);
	icpthread.detach();

	renderThread.RunRenderThread();

	return EXIT_SUCCESS;
}