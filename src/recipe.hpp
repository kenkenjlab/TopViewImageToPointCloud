#ifndef _RECIPE_HPP_
#define _RECIPE_HPP_ 2015011001

#include <opencv2/core.hpp>

struct Recipe {
	cv::Scalar color;
	int point_nr;
	double min_height, max_height;
	double horizontal_noise_half_width;

	Recipe(const cv::Scalar &_color = cv::Scalar(255, 255, 255), int _point_nr = 100, double _min_height = 0.0, double _max_height = 0.0, double _horizontal_noise_half_width = 0.0)
		: color(_color)
		, point_nr(_point_nr)
		, min_height(_min_height)
		, max_height(_max_height)
		, horizontal_noise_half_width(_horizontal_noise_half_width)
	{}
};

#endif