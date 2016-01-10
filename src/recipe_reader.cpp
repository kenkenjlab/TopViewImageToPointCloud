#include "recipe_reader.hpp"
#include "string_utility.hpp"
#include <fstream>

int RecipeReader::read(const std::string &filepath, std::vector<Recipe> &recipes) {
	recipes.clear();
	std::ifstream ifs(filepath);
	if (!ifs) { return 1; }

	std::string line;
	while (!ifs.eof()) {
		Recipe recipe;
		std::getline(ifs, line);
		parse_(line, recipe);
		recipes.push_back(recipe);
	}
	return 0;
}

int RecipeReader::parse_(const std::string &line, Recipe &recipe) {
	// [Format] R, G, B, num_of_pt, min_height, max_height, horizontal_noise_half_width
	// * color must be specified from 0 to 255.

	// (1) Split one line
	std::vector<std::string> elems = StringUtility::split(line, ',');

	// (2) Check 
	if (elems.size() < 7) { return 1; }
	const std::string &first_elem = elems[0];
	if (!first_elem.empty()) {
		if (first_elem[0] == '#') { return -1; }	// Comment-outed line
	}

	// (3) Convert
	char *temp_endptr;
	for (int i = 0; i < elems.size(); i++) {
		recipe.color.val[2] = std::strtol(elems[0].c_str(), &temp_endptr, 16);	// B
		recipe.color.val[1] = std::strtol(elems[1].c_str(), &temp_endptr, 16);	// G
		recipe.color.val[0] = std::strtol(elems[2].c_str(), &temp_endptr, 16);	// R
		recipe.point_nr = std::atoi(elems[3].c_str());
		recipe.min_height = std::atof(elems[4].c_str());
		recipe.max_height = std::atof(elems[5].c_str());
		recipe.horizontal_noise_half_width = std::atof(elems[6].c_str());
	}

	return 0;
}

