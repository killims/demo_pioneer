/*
 * timer.h
 *
 *  Created on: 06-may-2009
 *      Author: pablo
 */

#ifndef TIMER_H_
#define TIMER_H_
#include <sys/time.h>
//////////////////////////////////////////////////////////////////////////
class timer {
    public:
        timer();
        void start();
        void stop();
        double ms_elapsed() const;
        double s_elapsed() const;
        double us_elapsed() const;
    protected:
        timeval begin, end;
};
//////////////////////////////////////////////////////////////////////////
inline timer::timer() {
    gettimeofday(&begin, NULL);
    end = begin;
}
//////////////////////////////////////////////////////////////////////////
inline void timer::start() {
    gettimeofday(&begin, NULL);
}
//////////////////////////////////////////////////////////////////////////
inline void timer::stop() {
    gettimeofday(&end, NULL);
}
//////////////////////////////////////////////////////////////////////////
inline double timer::s_elapsed() const {
    return us_elapsed() / 1e6;
}
//////////////////////////////////////////////////////////////////////////
inline double timer::ms_elapsed() const {
    return us_elapsed() / 1e3;
}
//////////////////////////////////////////////////////////////////////////
inline double timer::us_elapsed() const {
    return (end.tv_sec - begin.tv_sec) * 1e6 + (end.tv_usec - begin.tv_usec);
}
//////////////////////////////////////////////////////////////////////////
#endif /* TIMER_H_ */
