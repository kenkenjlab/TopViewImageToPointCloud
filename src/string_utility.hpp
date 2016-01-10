#ifndef _STRING_UTILITY_HPP_
#define _STRING_UTILITY_HPP_

#include <string>
#include <vector>

class StringUtility {
public:
	static std::vector<std::string> split(const std::string& input, char delimiter);
};

#endif