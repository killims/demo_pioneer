/*
 * line.h
 *
 *  Created on: 01-abr-2009
 *      Author: pablo
 */

#ifndef LINE_H_
#define LINE_H_

#include "pointi.h"
#include <iterator>
#include <cstdlib>

////////////////////////////////////////////////////////////////////////////////
namespace my {
class line {
public:
	class iterator: public std::iterator<std::forward_iterator_tag, pointi> {
	public:
		inline iterator() :
			m_current(0, 0), index(-1) {
		}

		iterator(const pointi& from, const pointi& to) :
			m_current(from), index(0) {
			int deltax, deltay;

			deltax = abs(to.x - from.x);
			deltay = abs(to.y - from.y);

			/*  Initialize all vars based on which is the independent variable  */

			if (deltax >= deltay) {
				/* x is independent variable  */
				numpixels = deltax + 1;
				d = (deltay << 1) - deltax;
				dinc1 = deltay << 1;
				dinc2 = (deltay - deltax) << 1;
				xinc1 = 1;
				xinc2 = 1;
				yinc1 = 0;
				yinc2 = 1;
			} else {
				/* y is independent variable */
				numpixels = deltay + 1;
				d = (deltax << 1) - deltay;
				dinc1 = deltax << 1;
				dinc2 = (deltax - deltay) << 1;
				xinc1 = 0;
				xinc2 = 1;
				yinc1 = 1;
				yinc2 = 1;
			}

			/* Make sure x and y move in the right directions  */
			if (from.x > to.x) {
				xinc1 = -xinc1;
				xinc2 = -xinc2;
			}

			if (from.y > to.y) {
				yinc1 = -yinc1;
				yinc2 = -yinc2;
			}

		}

		bool operator==(const iterator& other) {
			return (index == other.index);
		}

		bool operator!=(const iterator& other) {
			return (index != other.index);
		}

		// Update my state such that I refer to the next element
		iterator& operator++() {
			if (index >= 0 and index < numpixels) {
				++index;
				if (d < 0) {
					d += dinc1;
					m_current.x += xinc1;
					m_current.y += yinc1;
				} else {
					d += dinc2;
					m_current.x += xinc2;
					m_current.y += yinc2;
				}
			}
			if (index == numpixels)
				index = -1;
			return (*this);
		}

		inline pointi& operator*() {
			return m_current;
		}

		inline pointi* operator->() {
			return &m_current;
		}

	private:
		pointi m_current;
		int numpixels, index;
		int d, dinc1, dinc2, xinc1, xinc2, yinc1, yinc2;
	};

	inline line(const pointi& from, const pointi& to) :
		m_from(from), m_to(to) {

	}

	inline iterator begin() {
		return iterator(m_from, m_to);
	}

	inline iterator end() {
		return iterator();
	}

private:
	const pointi m_from;
	const pointi m_to;
};
}
#endif /* LINE_H_ */
