#pragma once

struct ICPConfig
{
	ICPConfig()
	{

	}

	float m_maxCorrespondenceDistance;
	unsigned m_maxIterationCount;
	double m_transformationEpsilon;
	double m_euclideanFitnessEpsilon;
	bool m_useReciprocalCorrespondences;
};