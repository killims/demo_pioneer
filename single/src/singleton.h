/*
 * singleton.h
 *
 *  Created on: 06-may-2009
 *      Author: pablo
 */

#ifndef SINGLETON_H_
#define SINGLETON_H_

#include <memory>

template<class T>
class singleton {
    public:
        static T* instance();
    protected:
        singleton();
        singleton(const singleton& s);
        singleton& operator=(const singleton& s);
};

template <class T>
T* singleton<T>::instance() {
    static std::auto_ptr<T> m_instance;

    if (not m_instance.get())
        m_instance.reset(new T);

    return m_instance.get();
}

#endif /* SINGLETON_H_ */
