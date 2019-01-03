#pragma once

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "ICP.h"

namespace ICPTest
{ 
	enum EFrameMask
	{
		EFM_XYZ = 0x1,
		EFM_NXYZ = 0x2,
	};

	struct Point
	{
		union PointType
		{
			float coord[6];
			struct
			{
				float x;
				float y;
				float z;
				float nx;
				float ny;
				float nz;
			};
		};

		PointType point;

		Point()
		{
			memset(point.coord, 0, sizeof(point.coord));
		}

		Point(float* data)
		{
			memcpy(point.coord, data, sizeof(PointType));
		}
	};

	class IFrame
	{
		enum ESetFrameDataType
		{
			ESFDT_PCL,
			ESFDT_TRIMESH
		};

	public:
		IFrame() {}
		virtual bool IsValid() const = 0;
		virtual unsigned VertexCount() const = 0;
		virtual bool GetFrameData(unsigned uIndex, float* outCoord, float* outNormal) const = 0;
	};

	class PCLFrame : public IFrame
	{
	public:
		PCLFrame();
		bool InitFrame(float* data, unsigned pointCount, unsigned mask);
		bool IsValid() const;
		unsigned VertexCount() const;
		const pcl::PointCloud<pcl::PointNormal>::Ptr GetPoints() const;
		bool GetFrameData(unsigned uIndex, float* outCoord, float* outNormal) const;

	private:
		unsigned m_verticesCount;
		pcl::PointCloud<pcl::PointNormal>::Ptr m_points;
	};

	class TriMeshFrame : public IFrame
	{
	public:
		TriMeshFrame(const char* czFileName);
		~TriMeshFrame();
		bool IsValid() const;
		unsigned VertexCount() const;
		bool GetFrameData(unsigned uIndex, float* outCoord, float* outNormal) const;
		trimesh::TriMesh* GetTriMesh() const;

	private:
		trimesh::TriMesh* m_pFrame;
	};
}