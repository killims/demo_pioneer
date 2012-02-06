/*
 * state.h
 *
 *  Created on: 03/01/2011
 *      Author: pablo
 */

#ifndef STATE_H_
#define STATE_H_

#include <cmath>

class state_iterator;

class state {
public:
    int x, y;

    state(int _x, int _y);

    bool operator<(const state& s) const;
    bool operator==(const state& s) const;
    bool operator!=(const state& s) const;

    typedef state_iterator iterator;
    typedef state_iterator const_iterator;

    const_iterator begin_predecessors() const;
    iterator begin_predecessors();

    const_iterator end_predecessors() const;
    iterator end_predecessors();

    const_iterator begin_successors() const;
    iterator begin_successors();

    const_iterator end_successors() const;
    iterator end_successors();

};

double heuristic(const state& a, const state& b);

struct state_hash {

    inline unsigned long operator()(const state & s) const {
        return 503 * s.x + s.y;
    }
};

class state_iterator {
public:
    //iterator required types
    typedef std::forward_iterator_tag iterator_category;
    typedef state value_type;
    typedef unsigned int difference_type;
    typedef const state* pointer;
    typedef const state& reference;

    //assignable (by default)

    //equally comparable
    bool operator==(const state_iterator& i) const;
    bool operator!=(const state_iterator& i) const;

    //default constructible
    state_iterator();

    //trivial iterator
    reference operator*() const;
    pointer operator->() const;

    //input iterator
    state_iterator & operator++(); //pre-increment
    state_iterator operator++(int); //post-increment

    //constructor
    state_iterator(const state& s);

private:
    const state m_base;
    state m_actual;
    bool m_end;
};

inline state::state(int _x, int _y) :
x(_x), y(_y) {
}

inline bool state::operator<(const state& s) const {
    return x < s.x || (x == s.x && y < s.y);
}

inline bool state::operator ==(const state& s) const {
    return x == s.x && y == s.y;
}

inline bool state::operator!=(const state& s) const {
    return x != s.x || y != s.y;
}

inline state::const_iterator state::begin_predecessors() const {

    return state_iterator(*this);
}

inline state::iterator state::begin_predecessors() {

    return state_iterator(*this);
}

inline state::const_iterator state::end_predecessors() const {

    return state_iterator();
}

inline state::iterator state::end_predecessors() {

    return state_iterator();
}

inline state::const_iterator state::begin_successors() const {

    return state_iterator(*this);
}

inline state::iterator state::begin_successors() {

    return state_iterator(*this);
}

inline state::const_iterator state::end_successors() const {

    return state_iterator();
}

inline state::iterator state::end_successors() {

    return state_iterator();
}

inline bool state_iterator::operator==(const state_iterator& i) const {
    return (m_end && i.m_end) || (m_base == i.m_base && m_actual
            == i.m_actual);
}

inline bool state_iterator::operator!=(const state_iterator& i) const {

    return !this->operator ==(i);
}

inline state_iterator::state_iterator() :
m_base(0, 0), m_actual(0, 0), m_end(true) {
}

inline state_iterator::reference state_iterator::operator*() const {

    return m_actual;
}

inline state_iterator::pointer state_iterator::operator->() const {

    return &m_actual;
}

inline state_iterator& state_iterator::operator++() {
    ++m_actual.x;
    if (m_actual.x - m_base.x > 1) {
        m_actual.x = m_base.x - 1;
        ++m_actual.y;

        if (m_actual.y - m_base.y > 1)
            m_end = true;
    } else if (m_actual == m_base)
        ++m_actual.x;

    return *this;
}

inline state_iterator state_iterator::operator++(int) {
    state_iterator tmp = *this;
    this->operator++();

    return tmp;
}

//constructor

inline state_iterator::state_iterator(const state& s) :
m_base(s), m_actual(s.x -1, s.y - 1), m_end(false) {
}

inline double heuristic(const state& a, const state& b) {
    int dx = b.x - a.x;
    int dy = b.y - a.y;
   
    int d = std::min(abs(dx), abs(dy));
    int l = std::max(abs(dx), abs(dy));

    return (M_SQRT2 - 1) * d + l; //8-connect distance
}

#endif /* STATE_H_ */
