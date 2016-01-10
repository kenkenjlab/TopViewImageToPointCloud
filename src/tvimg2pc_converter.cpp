#include "tvimg2pc_converter.hpp"

int TopViewImageToPointCloudConverter::compute(pcl::PointCloud<pcl::PointXYZRGBA> &cloud) {
	typedef pcl::PointXYZRGBA PointT;
	typedef pcl::PointCloud<PointT> CloudT;
	typedef CloudT::Ptr CloudPtrT;
	typedef CloudT::ConstPtr  CloudConstPtrT;

	int ret = generatePointCloudFromImage_(cloud);

	for (int i = 0; i < cloud.size(); i++) {
		PointT &p = cloud[i];
		p.r = p.g = p.b = p.a = 255;
	}

	return ret;
}

int TopViewImageToPointCloudConverter::generateLargeRandomInteger_(int max_num) {
	int seed = std::rand();
	while (seed < max_num) {
		seed = seed * (RAND_MAX + 1) + std::rand();
	}
	return seed % max_num;
}