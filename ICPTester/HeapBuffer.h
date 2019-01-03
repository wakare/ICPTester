#pragma once
#include "ICPTestGlobalDef.h"
#include <stdlib.h>

namespace ICPTest
{
	template<typename T>
	class HeapBuffer
	{
	public:
		HeapBuffer(size_t size) : m_data(nullptr)
		{
			m_data = (T*)malloc(size);
#ifdef DEBUG
			if (!m_data)
			{
				ICPTesterOutStream << "[ERROR] Malloc failed." << std::endl;
				throw "exception";
			}
#endif
		}

		~HeapBuffer()
		{
			if (m_data)
			{
				free(m_data);
				m_data = nullptr;
			}
		}

		T* GetData()
		{
			return m_data;
		}

	private:
		T* m_data;
	};
}