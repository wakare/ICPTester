#pragma once
#include "LPtr.h"
#include <vector>

using namespace Leviathan;

namespace ICPTest
{ 
	class IFrame;

	enum EDataLoaderType
	{
		EDLT_PCL,
		EDLT_TRIMESH
	};

	class DataLoader
	{
	public:
		DataLoader(const char* dataFolder, EDataLoaderType type, int startIndex = -1, int endIndex = -1);
		bool Load(const char* dataFolder, int startIndex, int endIndex);
		const IFrame& GetAFrame(unsigned index);
		unsigned FrameCount() const;

	private:
		bool _loadFile(const char* dataFilePath);

		EDataLoaderType m_eDataLoadType;
		std::vector<LPtr<IFrame>> m_frames;
	};
}