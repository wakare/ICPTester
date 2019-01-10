#include "DataLoader.h"
#include "Frame.h"
#include "ICPTestGlobalDef.h"
#include <fstream>
#include "TriMesh.h"
#include "DynamicArray.h"

using namespace trimesh;

namespace ICPTest
{
	DataLoader::DataLoader(const char* dataFolder, EDataLoaderType type, int startIndex/* = -1*/, int endIndex/* = -1*/):
		m_eDataLoadType(type)
	{
		if (!Load(dataFolder, startIndex, endIndex))
		{
			throw "exception";
		}
	}

	bool DataLoader::Load(const char* dataFolder, int startIndex, int endIndex)
	{
		char buf[512];
		unsigned uCurrentFrameIndex = 0;

		startIndex = (startIndex > 0) ? startIndex : 0;
		
		while (endIndex < 0 || (startIndex++) <= endIndex)
		{
			snprintf(buf, sizeof(buf), "%s//frame_%04d.ply", dataFolder, uCurrentFrameIndex++);

			// Init frame
			if (!_loadFile(buf))
			{
				ICPTesterOutStream << "[ERROR] Load file failed, path: " << buf << std::endl;
				break;
			}
		}

		return true;
	}

	const ICPTest::IFrame& DataLoader::GetAFrame(unsigned index)
	{
#ifdef DEBUG
		if (index > m_frames.size() - 1)
		{
			throw "exception";
		}
#endif 
		return (*m_frames[index]);
	}

	unsigned DataLoader::FrameCount() const
	{
		return m_frames.size();
	}

	bool DataLoader::_loadFile(const char * dataFilePath)
	{
		std::fstream frameFile(dataFilePath, std::ios::in);
		if (!frameFile.is_open())
		{
			return false;
		}

		if (m_eDataLoadType == EDLT_TRIMESH)
		{
			LPtr<TriMeshFrame> pTriMeshFrame = new TriMeshFrame(dataFilePath);
			m_frames.push_back(TryCast<TriMeshFrame, IFrame>(pTriMeshFrame));

			return true;
		}

		char line[300];
		unsigned vertexCount = 0;
		while (true)
		{
			frameFile.getline(line, 300);
			std::string _line = line;
			if (_line.find("element vertex") != std::string::npos)
			{
				std::string vertexCountStr = _line.substr(15, _line.length() - 15);
				vertexCount = std::atoi(vertexCountStr.c_str());
			}

			if (_line == "end_header")
			{
				break;
			}
		}

		// Skip first two line
		frameFile.getline(line, sizeof(line));
		frameFile.getline(line, sizeof(line));

		Leviathan::DynamicArray<float> databuffer(vertexCount * 6 * sizeof(float));
		float* pData = databuffer.m_pData;

		// Load XYZ & NXYZ
		for (unsigned i = 0; i < vertexCount * 7; i++)
		{
			float tempValue;
			frameFile >> tempValue;
			if (i % 7 == 0)
			{
				continue;
			}

			pData[i - 1 - (i / 7)] = tempValue;
		}

		LPtr<PCLFrame> _frame = new PCLFrame();
		if (!(*_frame).InitFrame(pData, vertexCount, EFrameMask::EFM_XYZ | EFrameMask::EFM_NXYZ))
		{
			throw "exception";
		}

		m_frames.push_back(TryCast<PCLFrame, IFrame>(_frame));

		return true;
	}
}