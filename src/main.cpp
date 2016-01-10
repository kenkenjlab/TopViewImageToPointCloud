#include "tvimg2pc_converter.hpp"
#include "recipe_reader.hpp"
#include <time.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <opencv2/highgui.hpp>

int main(int argc, char** argv) {
	std::srand((unsigned int)time(NULL));

	// (0) Prepare default parameters
	std::string input_img_path, input_recipe_path, output_cloud_path;
	cv::Point2i viewpoint(0, 0);
	double viewpoint_height = 1.0;
	double meter_per_pixel = 0.01;

	// (1) Parse command line arguments
	pcl::console::parse_argument(argc, argv, "-i", input_img_path);
	pcl::console::parse_argument(argc, argv, "-r", input_recipe_path);
	pcl::console::parse_argument(argc, argv, "-o", output_cloud_path);
	pcl::console::parse_2x_arguments(argc, argv, "-vp", viewpoint.x, viewpoint.y);
	pcl::console::parse_argument(argc, argv, "-h", viewpoint_height);
	pcl::console::parse_argument(argc, argv, "-mmp", meter_per_pixel);

	// (2) Load image
	cv::Mat image = cv::imread(input_img_path);

	// (3) Load recipes
	std::vector<Recipe> recipes;
	RecipeReader::read(input_recipe_path, recipes);

	// (4) Generate 3D point cloud
	TopViewImageToPointCloudConverter converter;
	pcl::PointCloud<pcl::PointXYZRGBA> cloud;
	converter.setInputImage(image);
	converter.setScale(meter_per_pixel);
	converter.setViewPoint(viewpoint.x, viewpoint.y, viewpoint_height);
	for (int i = 0; i < recipes.size(); i++) {
		converter.addRecipe(recipes[i]);
	}
	converter.compute(cloud);

	// (5) Save
	pcl::io::savePLYFile(output_cloud_path, cloud);

	return 0;
}