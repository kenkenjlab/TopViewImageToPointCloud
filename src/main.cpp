#include "tvimg2pc_converter.hpp"
#include <time.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <opencv2/highgui.hpp>

int main(int argc, char** argv) {
	std::srand((unsigned int)time(NULL));

	// (1) Parse command line arguments
	//pcl::console::parse_argument(argc, argv, "", img_path);

	// (2) Load image
	cv::Mat image = cv::imread("C:/kenken/jLab/projects/prototypes/TopViewImageToPointCloud/img/sample1.png");

	// (3) Generate 3D point cloud
	TopViewImageToPointCloudConverter converter;
	pcl::PointCloud<pcl::PointXYZRGBA> cloud;
	converter.setInputImage(image);
	converter.setScale(0.1, 1.0);
	converter.setViewPoint(235, 344, 1.0);
	converter.addRecipe(Recipe(CV_RGB(0x22, 0xb1, 0x4c), 100, true, 0.08, 0.0));
	converter.addRecipe(Recipe(CV_RGB(0x00, 0xa2, 0x28), 100, true, 0.00, 0.0));
	converter.compute(cloud);

	// (4) Save
	pcl::io::savePLYFile("C:/kenken/jLab/projects/prototypes/TopViewImageToPointCloud/cloud.ply", cloud);

	return 0;
}