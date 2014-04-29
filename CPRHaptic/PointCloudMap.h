#pragma once
#include "windows.h"
#include <opencv2\core\core.hpp>
#include <Eigen\Geometry>
using namespace Eigen;

using namespace cv;

class PointCloudMap
{
public:
	Vector3d* m_points;
	int m_width;
	int m_height;

public:
	PointCloudMap();
	PointCloudMap(int width,int height);
	~PointCloudMap();
	void Resize(int width,int height);
	void Create(Mat& depth_image,USHORT max_depth = 4000,float scale = 1);
};