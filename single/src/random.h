/*
 * random.h
 *
 *  Created on: 13-may-2009
 *      Author: pablo
 */

#ifndef RANDOM_H_
#define RANDOM_H_
////////////////////////////////////////////////////////////////////////////////
#include <cstdlib>
#include <ctime>

////////////////////////////////////////////////////////////////////////////////
namespace my {
    class random {
        public:
            random();
            int get_integer();
            int get_integer(int max);
            int get_integer(int min, int max);

            double get_double();
            double get_double(double max);
            double get_double(double min, double max);
    };
}
////////////////////////////////////////////////////////////////////////////////
inline my::random::random() {
    srandom(time(NULL));
}
////////////////////////////////////////////////////////////////////////////////
inline int my::random::get_integer() {
    return rand();
}
////////////////////////////////////////////////////////////////////////////////
inline int my::random::get_integer(int max) {
    return (int) (max * get_double());
}
////////////////////////////////////////////////////////////////////////////////
inline int my::random::get_integer(int min, int max) {
    return get_integer(max - min) + min;
}
////////////////////////////////////////////////////////////////////////////////
inline double my::random::get_double() {
    return get_integer() / (RAND_MAX * 1.0);
}
////////////////////////////////////////////////////////////////////////////////
inline double my::random::get_double(double max) {
    return max * get_double();
}
////////////////////////////////////////////////////////////////////////////////
inline double my::random::get_double(double min, double max) {
    return get_double(max - min) + min;
}
////////////////////////////////////////////////////////////////////////////////

#endif /* RANDOM_H_ */
