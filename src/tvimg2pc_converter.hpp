#ifndef _TVIMG2PC_CONVERTER_HPP_
#define _TVIMG2PC_CONVERTER_HPP_ 2016011001

// STL
#include <vector>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// OpenCV
#include <opencv2/core.hpp>

// Hand-made
#include "recipe.hpp"

class TopViewImageToPointCloudConverter {
private:
	// Parameters
	cv::Mat input_image_;
	double mpp_;	// Meter per Pixel
	double viewpoint_height_;
	std::vector<Recipe> recipes_;
	cv::Point2d viewpoint_;

	template <class PointType> int generatePointCloudFromImage_(pcl::PointCloud<PointType> &cloud);
	static double generateRandomProbability_() { return static_cast<double>(std::rand()) / RAND_MAX; }
	static int generateLargeRandomInteger_(int max_num);

public:
	TopViewImageToPointCloudConverter()
		: mpp_(100)
		, viewpoint_height_(0.0)
		, viewpoint_(0.0, 0.0)
	{}

	// Setter (mandatory)
	inline void setInputImage(const cv::Mat &image) { image.copyTo(input_image_); }
	inline void setScale(double meter_per_pixel) { mpp_ = meter_per_pixel; }
	inline void setScale(double meter, double pixel) { setScale(meter / pixel); }
	inline void addRecipe(const Recipe &recipe) { recipes_.push_back(recipe); }
	inline void clearRecipes() { recipes_.clear(); }
	inline void setViewPoint(int x, int y, double height) { viewpoint_.x = (double)x; viewpoint_.y = (double)y; viewpoint_height_ = height; }

	// Processor
	int compute(pcl::PointCloud<pcl::PointXYZRGBA> &cloud);
	
	

};

template <class PointType>
int TopViewImageToPointCloudConverter::generatePointCloudFromImage_(pcl::PointCloud<PointType> &cloud) {
	// (1) Pick up target pixels for all recipes
	std::vector<std::vector<cv::Point2d>> tgt_pixel_vec_(recipes_.size());
	for (int y = 0; y < input_image_.rows; y++) {
		for (int x = 0; x < input_image_.cols; x++) {
			const cv::Vec3b &p = input_image_.at<cv::Vec3b>(y, x);
			for (int i = 0; i < recipes_.size(); i++) {
				const Recipe &recipe = recipes_[i];
				bool is_same_color = true;

				// (a) Find the same color as user specified
				for (int k = 0; k < 3; k++) {
					if (p.val[k] == recipe.color[k]) { continue; }
					is_same_color = false;
					break;
				}
				if (!is_same_color) { continue; }

				// (b) Store
				tgt_pixel_vec_[i].push_back(cv::Point2d(x, y));
			}
		}
	}

	// (2) Generate 3D points as many as user specified
	cloud.clear();
	for (int i = 0; i < recipes_.size(); i++) {
		const Recipe &recipe = recipes_[i];
		const std::vector<cv::Point2d> &pixels = tgt_pixel_vec_[i];
		if (pixels.empty()) { continue; }

		// (2) Generate point cloud
		for (int j = 0; j < recipe.point_nr; j++) {
			// (a) Randomly sample pixels as many as user specified
			const int index = generateLargeRandomInteger_(pixels.size());
			const cv::Point2d &p_2d = pixels[index];
			

			// (b) Compute area where 3D point will be generated on XY plane
			cv::Point2d p_2d_relative = p_2d - viewpoint_;
			cv::Point2d p_3d_topleft;
			p_3d_topleft.x = (p_2d_relative.x - (p_2d_relative.x < 0 ? 1 : 0)) * mpp_;
			p_3d_topleft.y = (p_2d_relative.y - (p_2d_relative.y < 0 ? 1 : 0)) * mpp_;

			// (c) Generate 3D point
			PointType p_3d;
			const double min_height_relative = recipe.min_height - viewpoint_height_;
			const double max_height_relative = recipe.max_height - recipe.min_height;
			p_3d.x = p_3d_topleft.x + generateRandomProbability_() * mpp_;
			p_3d.z = p_3d_topleft.y + generateRandomProbability_() * mpp_;
			p_3d.y = min_height_relative + generateRandomProbability_() * max_height_relative;
			double noise = generateRandomProbability_() * recipe.horizontal_noise_half_width * 2 - recipe.horizontal_noise_half_width;

			// (d) Randomly decide the direction and then Add noise
			double angle = M_PI * 2 * generateRandomProbability_();
			p_3d.x += noise * std::cos(angle);
			p_3d.z += noise * std::sin(angle);
			cloud.push_back(p_3d);
		}
	}
	
	return 0;
}

#endif