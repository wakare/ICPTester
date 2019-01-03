#include "Frame.h"

namespace ICPTest
{
	PCLFrame::PCLFrame()
	{
		m_points.reset(new pcl::PointCloud<pcl::PointNormal>);
	}

	bool PCLFrame::InitFrame(float * data, unsigned pointCount, unsigned mask)
	{
		// Only support XYZ && NXYZ
		if (mask & EFM_XYZ == 0 || mask & EFM_NXYZ == 0)
		{
			return false;
		}

		m_verticesCount = pointCount;
		for (unsigned i = 0; i < pointCount; i++)
		{
			/*m_points.push_back(data + i * 6);*/
			pcl::PointNormal point;
			memcpy(point.data, data + i * 6, sizeof(float) * 3);
			memcpy(point.data_n, data + i * 6 + 3, sizeof(float) * 3);

			point.data[3] = 1.0f;
			point.data_n[3] = 1.0f;

			m_points->push_back(std::move(point));
		}

		return true;
	}

	bool PCLFrame::IsValid() const
	{
		return m_points->size() != 0;
	}
	
	unsigned PCLFrame::VertexCount() const
	{
		return m_verticesCount;
	}

	const pcl::PointCloud<pcl::PointNormal>::Ptr PCLFrame::GetPoints() const
{
		return m_points;
	}

	bool PCLFrame::GetFrameData(unsigned uIndex, float* outCoord, float* outNormal) const
	{
		auto& point = m_points->points[uIndex];

		memcpy(outCoord, point.data, sizeof(float) * 3);
		memcpy(outNormal, point.data_n, sizeof(float) * 3);

		return true;
	}

	TriMeshFrame::TriMeshFrame(const char* czFileName) : 
		m_pFrame(nullptr)
	{
		m_pFrame = trimesh::TriMesh::read(czFileName);
	}

	TriMeshFrame::~TriMeshFrame()
	{
		if (m_pFrame)
		{ 
			delete m_pFrame;
			m_pFrame = nullptr;
		}
	}

	bool TriMeshFrame::IsValid() const
	{
		return m_pFrame != nullptr;
	}

	unsigned TriMeshFrame::VertexCount() const
	{
		return m_pFrame->vertices.size();
	}

	bool TriMeshFrame::GetFrameData(unsigned uIndex, float* outCoord, float* outNormal) const
	{
		auto& point = m_pFrame->vertices[uIndex];
		auto& normal = m_pFrame->normals[uIndex];

		memcpy(outCoord, &point.x, sizeof(float) * 3);
		memcpy(outNormal, &normal.x, sizeof(float) * 3);

		return true;
	}

	trimesh::TriMesh* TriMeshFrame::GetTriMesh() const
	{
		return m_pFrame;
	}

}