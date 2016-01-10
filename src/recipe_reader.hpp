#ifndef _RECIPE_READER_HPP_
#define _RECIPE_READER_HPP_ 2016011001

#include <string>
#include <vector>
#include "recipe.hpp"

class RecipeReader {
private:
	static std::vector<std::string> RecipeReader::split_(const std::string& input, char delimiter);
	static int parse_(const std::string &line, Recipe &recipe);

public:
	static int read(const std::string &filepath, std::vector<Recipe> &recipes);
};

#endif