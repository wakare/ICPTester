#include "ICPTester.h"
#include "Visualizator.h"
#include <windows.h>
#include <thread>

using namespace ICPTest;
using namespace Leviathan;

void ThreadDoICP(const char* DataFolder, const char* icpType, const char* outputICPResult, int startIndex , int endIndex)
{
	EICPTestType type;

	if (0 == strcmp(icpType, "pcl"))			type = EICPTT_PCL; 
	if (0 == strcmp(icpType, "trimesh"))		type = EICPTT_TRIMESH; 

	ICPTester tester(DataFolder, type, startIndex, endIndex);
	tester.DoICP();

	if (outputICPResult) tester.ExportICPResultToFile(outputICPResult);

	// Remain resources for a while 
	Sleep(10000);
}

int main(int argc, char** argv)
{
	if (argc < 3)
	{
		return EXIT_FAILURE;
	}

	VisualizatorLeviathan renderThread;
	renderThread.Init();

	int startIndex = (argc > 5) ? std::atoi(argv[4]) : -1;
	int endIndex = (argc > 5) ? std::atoi(argv[5]) : -1;

	std::thread icpthread(ThreadDoICP, argv[1], argv[2], (argc > 3) ? argv[3] : nullptr, startIndex, endIndex);
	icpthread.detach();

	renderThread.RunRenderThread();

	return EXIT_SUCCESS;
}