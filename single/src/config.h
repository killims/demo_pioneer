/*
 * config.h
 *
 *  Created on: 11-may-2009
 *      Author: pablo
 */

#ifndef CONFIG_H_
#define CONFIG_H_
////////////////////////////////////////////////////////////////////////////////
#include <string>
#include <map>
#include <sstream>
#include <stdexcept>
////////////////////////////////////////////////////////////////////////////////
class config {
    public:
        config(const std::string& file);
        bool find(const std::string& key);

        template<typename T>
        T get(const std::string& key);

    protected:
        std::map<std::string, std::string> m_map;
        void parse(const std::string& file);
};
////////////////////////////////////////////////////////////////////////////////
class key_not_defined: public std::exception {
    protected:
        std::string m_message;
    public:
        key_not_defined(const std::string& key);
        virtual ~key_not_defined() throw();
        virtual const char* what() const throw();
};
////////////////////////////////////////////////////////////////////////////////
inline bool config::find(const std::string& key) {
    return (m_map.find(key) != m_map.end());
}
////////////////////////////////////////////////////////////////////////////////
template<typename T>
inline T config::get(const std::string& key) {

    if (not find(key))
        throw key_not_defined(key);

    T ret;
    std::istringstream iss(m_map[key]);
    iss >> ret;

    return ret;
}
////////////////////////////////////////////////////////////////////////////////
inline key_not_defined::key_not_defined(const std::string& key) :
    std::exception(), m_message(key) {
    m_message.append(": key not defined");
}
////////////////////////////////////////////////////////////////////////////////
inline key_not_defined::~key_not_defined() throw() {
}
////////////////////////////////////////////////////////////////////////////////
inline const char* key_not_defined::what() const throw() {
    return m_message.c_str();
}
////////////////////////////////////////////////////////////////////////////////
#endif /* CONFIG_H_ */
