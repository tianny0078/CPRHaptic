#pragma once

#include "PointCloudMap.h"

class NormalsMap
{
public:
	Vector3d * m_normals;
	int m_width;
	int m_height;
	
public:
	NormalsMap();
	NormalsMap(int width, int height);
	~NormalsMap();
	void Resize(int width, int height);
	void Create(PointCloudMap& point_cloud, float max_distance);
	void FlipNormalsToVector(Vector3d main_vector);

private:
	inline bool IsNeighbor(Vector3d& dst, Vector3d& ori,float max_square_distance);
	Vector3d EstimateNormal(vector<Vector3d>& points);
};