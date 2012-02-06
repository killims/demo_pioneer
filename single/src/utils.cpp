/*
 * utils.cpp
 *
 *  Created on: 14-may-2009
 *      Author: pablo
 */

#include "utils.h"
#include <sstream>

std::string utils::name(const std::string& base, int index,
        const std::string& ext, int width, char fill) {
    std::ostringstream oss;
    oss << base << '-';
    oss.width(width);
    oss.fill(fill);
    oss.flags(std::ios::right);
    oss << index << ext;

    return oss.str();
}
