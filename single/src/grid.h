/*
 * grid.h
 *
 *  Created on: 18-Feb-2009
 *      Author: pablo
 */

#ifndef GRID_H_
#define GRID_H_

////////////////////////////////////////////////////////////////////////////////
#include "matrix.h"
#include "cell.h"
#include "line.h"
#include "timer.h"
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "image.h"
////////////////////////////////////////////////////////////////////////////////

template<int Size>
class grid: public matrix<cell::type, Size> {
public:
	grid(cell::type c = cell::UNKNOWN);

	unsigned int size() const;

	template<int S>
	void mask(const grid<S>& m, grid<Size>& out) const;

	void clear(cell::type c = cell::UNKNOWN);
	void patch(const grid<Size>& g);
	void fuse(const grid<Size>& g);
	void move(int deltax, int deltay);
	bool is_free(const pointi& start, const pointi& end) const;
	void drawline(const pointi& start, const pointi& end, cell::type value =
			cell::FREE);

	void save(IplImage* image) const;
protected:
	void move_cell(int fromx, int fromy, int tox, int toy);
};
////////////////////////////////////////////////////////////////////////////////
template<int Size>
inline grid<Size>::grid(cell::type c) {
	this->m_cells.assign(c);
}
////////////////////////////////////////////////////////////////////////////////
template<int Size>
inline unsigned int grid<Size>::size() const {
	return Size;
}
////////////////////////////////////////////////////////////////////////////////
template<int Size>
inline void grid<Size>::clear(cell::type c) {
	this->m_cells.assign(c);
}
////////////////////////////////////////////////////////////////////////////////
template<int Size>
void grid<Size>::patch(const grid<Size>& g) {
	this->apply(g, cell::patch);
	//for (unsigned int i = 0; i < this->m_cells.size(); i++)
	//this->m_cells[i] = cell::patch(this->m_cells[i], g.m_cells[i]);
}
////////////////////////////////////////////////////////////////////////////////
template<int Size>
template<int S>
void grid<Size>::mask(const grid<S>& m, grid<Size>& out) const {
	assert(m.size() <= size());
	out = *this;
	int radius = m.size() / 2;
	for (int i = 0; i < int(size()); i++)
		for (int j = 0; j < int(size()); j++)
			if (this->operator ()(i, j) == cell::NONFREE) {
				int ibase = i - radius;
				int jbase = j - radius;
				for (int mi = 0; mi < int(m.size()); mi++)
					for (int mj = 0; mj < int(m.size()); mj++) {
						int indexi = ibase + mi;
						int indexj = jbase + mj;
						if (this->valid(indexi, indexj) and m(mi, mj)
								== cell::NONFREE)
							out(indexi, indexj) = cell::NONFREE;
					}
			}
}
////////////////////////////////////////////////////////////////////////////////
template<int Size>
void grid<Size>::move(int deltax, int deltay) {
	if (deltax > 0) {
		if (deltay > 0) {
			for (int x = size() - 1; x >= 0; x--)
				for (int y = size() - 1; y >= 0; y--)
					move_cell(x - deltax, y - deltay, x, y);
		} else {
			for (int x = size() - 1; x >= 0; x--)
				for (unsigned int y = 0; y < size(); y++)
					move_cell(x - deltax, y - deltay, x, y);
		}
	} else {
		if (deltay > 0) {
			for (unsigned int x = 0; x < size(); x++)
				for (int y = size() - 1; y >= 0; y--)
					move_cell(x - deltax, y - deltay, x, y);
		} else {
			for (unsigned int x = 0; x < size(); x++)
				for (unsigned int y = 0; y < size(); y++)
					move_cell(x - deltax, y - deltay, x, y);
		}
	}
}
////////////////////////////////////////////////////////////////////////////////
template<int Size>
void grid<Size>::move_cell(int fromx, int fromy, int tox, int toy) {
	this->operator()(tox, toy) = this->valid(fromx, fromy) ? this->operator ()(
			fromx, fromy) : cell::UNKNOWN;
}
////////////////////////////////////////////////////////////////////////////////
template<int Size>
void grid<Size>::fuse(const grid<Size>& g) {
	this->apply(g, cell::fuse);
	//for (unsigned int i = 0; i < this->m_cells.size(); i++)
	//this->m_cells[i] = cell::fuse(this->m_cells[i], g.m_cells[i]);
}
////////////////////////////////////////////////////////////////////////////////
template<int Size>
bool grid<Size>::is_free(const pointi& start, const pointi& end) const {
	my::line theline(start, end);

	for (my::line::iterator i = theline.begin(); i != theline.end(); ++i) {
		if (this->valid(i->x, i->y) and this->operator ()(i->x, i->y)
				== cell::NONFREE)
			return false;
	}

	return true;
}
////////////////////////////////////////////////////////////////////////////////
template<int Size>
void grid<Size>::drawline(const pointi& start, const pointi& end,
		cell::type value) {
	my::line theline(start, end);
	for (my::line::iterator i = theline.begin(); i != theline.end(); ++i) {
		if (this->valid(i->x, i->y))
			this->operator ()(i->x, i->y) = value;
	}
}
////////////////////////////////////////////////////////////////////////////////
template<int Size>
void grid<Size>::save(IplImage* im) const {
	image pic(im);

	for (int i = 0; i < Size; i++) {
		for (int j = 0; j < Size; j++) {
			pixel p;
			switch (this->operator()(i, j)) {
			case cell::UNKNOWN:
				p.r = p.g = p.b = 128;
				break;
			case cell::FREE:
				p.r = p.g = p.b = 255;
				break;
			case cell::NONFREE:
				p.r = p.g = p.b = 0;
				break;
			}
			pic(i, j) = p;
		}
	}
}
//*/
#endif /* GRID_H_ */

