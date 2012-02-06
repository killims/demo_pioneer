/*
 * config.cpp
 *
 *  Created on: 11-may-2009
 *      Author: pablo
 */
////////////////////////////////////////////////////////////////////////////////
#include "config.h"
#include <fstream>
#include <algorithm>
////////////////////////////////////////////////////////////////////////////////
config::config(const std::string& file) {
    parse(file);
}
////////////////////////////////////////////////////////////////////////////////
struct is_blank {
        bool operator()(char c) {
            return (c == ' ' or c == '\t');
        }
};
////////////////////////////////////////////////////////////////////////////////
void config::parse(const std::string& file) {
    std::ifstream fs(file.c_str());
    std::string line;
    while (std::getline(fs, line)) {

        //remove comments
        unsigned int sharp = line.find_first_of("#");
        if (sharp != std::string::npos)
            line.erase(sharp);
        if (line.empty())
            continue;

        //remove spaces and tabs
        line.erase(std::remove_if(line.begin(), line.end(), is_blank()),
                line.end());
        if (line.empty())
            continue;

        //check for include directives
        unsigned int include = line.find("%include");
        if (include != std::string::npos) {
            std::string f(line, include + 8);
            parse(f);
            continue;
        }

        //find the equal sign
        unsigned int equal = line.find_first_of("=");
        if (equal == std::string::npos)
            continue;

        //get the key
        std::string key(line.begin(), line.begin() + equal);

        //get the value
        std::string value(line.begin() + equal + 1, line.end());

        //insert in the map
        m_map.insert(std::make_pair(key, value));
    }
}
////////////////////////////////////////////////////////////////////////////////
