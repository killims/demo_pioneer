#ifndef PQUEUE_H
#define PQUEUE_H

#include <map>
#include <unordered_map>
#include <boost/tuple/tuple.hpp>

/**
  * @class pqueue
  *
  * Priority queue with priority update operation
  */

template <  class T,
class Hash,
class Priority,
class Priority_Order = std::less<Priority> > class pqueue {

    typedef std::multimap<Priority, T> queue;
    typedef typename std::unordered_map<T, typename queue::iterator, Hash> table;
public:
    //stl container types
    typedef typename queue::value_type value_type;
    typedef typename queue::reference reference;
    typedef typename queue::const_reference const_reference;
    typedef typename queue::pointer pointer;

    typedef typename queue::iterator iterator;
    typedef typename queue::const_iterator const_iterator;

    typedef typename queue::difference_type difference_type;
    typedef typename queue::size_type size_type;

    //stl container operations
    inline iterator begin() {
        return m_queue.begin();
    }

    inline iterator end() {
        return m_queue.end();
    }

    inline const_iterator begin() const {
        return m_queue.begin();
    }

    inline const_iterator end() const {
        return m_queue.begin();
    }

    inline size_type size() const {
        return m_queue.size();
    }

    inline size_type max_size() const {
        return m_queue.max_size();
    }

    inline bool empty() const {
        return m_queue.empty();
    }

    inline void swap(const pqueue<T, Priority, Priority_Order>& q) {
        return m_queue.swap(q);
    }

    //stl assignable operations
    inline pqueue(const pqueue<T, Priority, Priority_Order>& q): m_queue(q.m_queue) {}

    friend void swap(pqueue<T, Priority, Priority_Order>& a, pqueue<T, Priority, Priority_Order>& b) {
        a.swap(b);
    }

    //stl equality comparable operations
    inline bool operator==(const pqueue<T, Priority, Priority_Order>& q) const {
        return m_queue == q.m_queue;
    }

    inline bool operator!=(const pqueue<T, Priority, Priority_Order>& q) const {
        return m_queue != q.m_queue;
    }

    //stl less than comparable operations
    inline bool operator<(const pqueue<T, Priority, Priority_Order>& q) const {
        return m_queue < q.m_queue;
    }

    inline bool operator<=(const pqueue<T, Priority, Priority_Order>& q) const {
        return m_queue <= q.m_queue;
    }

    inline bool operator>(const pqueue<T, Priority, Priority_Order>& q) const {
        return m_queue > q.m_queue;
    }

    inline bool operator>=(const pqueue<T, Priority, Priority_Order>& q) const {
        return m_queue >= q.m_queue;
    }

    //priority queue operations
    inline pqueue() {}

    inline const_reference top() const {
        return *begin();
    }

    inline void pop() {
        m_queue.erase(m_queue.begin());
    }

    inline void push(const T& e, const Priority& priority) {
        typename table::iterator it;
        bool inserted;
        boost::tie(it, inserted) = m_table.insert(typename table::value_type(e, m_queue.end()));

        //if exists, remove previous priority (update)
        if (not inserted)
            m_queue.erase(it->second);

        it->second = m_queue.insert(value_type(priority, e));
    }

    inline void erase(const T& e) {
        typename table::iterator it = m_table.find(e);

        if (it != m_table.end()) {
            m_queue.erase(it->second);
            m_table.erase(it);
        }
    }

    inline void clear() {
        m_table.clear();
        m_queue.clear();
    }

private:
    queue m_queue;
    table m_table;
};

#endif // PQUEUE_H
