#ifndef _RECIPE_HPP_
#define _RECIPE_HPP_ 2015010301

#include <opencv2/core.hpp>

struct Recipe {
	cv::Scalar color;
	int point_nr;
	bool horizontal;
	double height;
	double noise_half_width;


	Recipe(const cv::Scalar &_color = cv::Scalar(255, 255, 255), int _point_nr = 100, bool _horizontal = true, double _height = 0.0, double _noise_half_width = 0.0)
		: color(_color)
		, point_nr(_point_nr)
		, horizontal(_horizontal)
		, height(_height)
		, noise_half_width(_noise_half_width)
	{}
};

#endif