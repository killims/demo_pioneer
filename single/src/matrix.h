/*
 * matrix.h
 *
 *  Created on: 11-mar-2009
 *      Author: pablo
 */

#ifndef MATRIX_H_
#define MATRIX_H_
////////////////////////////////////////////////////////////////////////////////
#include <vector>
#include <boost/array.hpp>
////////////////////////////////////////////////////////////////////////////////
template<typename T, int Width, int Height = Width>
class matrix {
public:
	//operators
	T operator()(int x, int y) const;
	T& operator()(int x, int y);

	bool valid(int x, int y) const;
	unsigned int width() const;
	unsigned int height() const;

	template<class Function>
	void apply(const matrix<T, Width, Height>& a, Function f);

	template<class TT, int W, int H, class Function>
	friend void apply(const matrix<TT, W, H>& a, const matrix<TT, W, H>& b,
			matrix<TT, W, H>& c, Function f);

protected:
	boost::array<T, Width * Height> m_cells;
};
////////////////////////////////////////////////////////////////////////////////
template<typename T, int Width, int Height>
inline T matrix<T, Width, Height>::operator()(int x, int y) const {
	return m_cells[y * Width + x];
}
////////////////////////////////////////////////////////////////////////////////
template<typename T, int Width, int Height>
inline T& matrix<T, Width, Height>::operator()(int x, int y) {
	return m_cells[y * Width + x];
}
////////////////////////////////////////////////////////////////////////////////
template<typename T, int Width, int Height>
inline bool matrix<T, Width, Height>::valid(int x, int y) const {
	return (x >= 0 and x < Width and y >= 0 and y < Height);
}
////////////////////////////////////////////////////////////////////////////////
template<typename T, int Width, int Height>
inline unsigned int matrix<T, Width, Height>::width() const {
	return Width;
}
////////////////////////////////////////////////////////////////////////////////
template<typename T, int Width, int Height>
inline unsigned int matrix<T, Width, Height>::height() const {
	return Height;
}
////////////////////////////////////////////////////////////////////////////////
template<typename T, int Width, int Height>
template<class Function>
void matrix<T, Width, Height>::apply(const matrix<T, Width, Height>& a,
		Function f) {
	for (unsigned int i = 0; i < m_cells.size(); i++) {
		m_cells[i] = f(m_cells[i], a.m_cells[i]);
	}
}
////////////////////////////////////////////////////////////////////////////////
template<class T, int W, int H, class Function>
void apply(const matrix<T, W, H>& a, const matrix<T, W, H>& b,
		matrix<T, W, H>& c, Function f) {
	for (unsigned int i = 0; i < c.m_cells.size(); i++)
		c.m_cells[i] = f(a.m_cells[i], b.m_cells[i]);
}
////////////////////////////////////////////////////////////////////////////////
/*
 template<typename T>
 class matrix<T, -1, -1> {
 public:
 //contructors
 matrix();
 matrix(const unsigned int size);
 matrix(const unsigned int width, const unsigned int height);

 //operators
 T operator()(int x, int y) const;
 T& operator()(int x, int y);

 bool valid(int x, int y) const;
 unsigned int width() const;
 unsigned int height() const;

 protected:
 unsigned int m_width, m_height;
 std::vector<T> m_cells;
 };
 ////////////////////////////////////////////////////////////////////////////////
 template<typename T>
 inline matrix<T>::matrix() :
 m_width(0), m_height(0) {
 }
 ////////////////////////////////////////////////////////////////////////////////
 template<typename T>
 inline matrix<T>::matrix(const unsigned int size) :
 m_width(size), m_height(size), m_cells(m_width * m_height) {
 }
 ////////////////////////////////////////////////////////////////////////////////
 template<typename T>
 inline matrix<T>::matrix(const unsigned int width, const unsigned int height) :
 m_width(width), m_height(height), m_cells(m_width * m_height) {
 }
 ////////////////////////////////////////////////////////////////////////////////
 template<typename T>
 inline T matrix<T>::operator()(int x, int y) const {
 assert(valid(x,y));
 return m_cells[y * m_width + x];
 }
 ////////////////////////////////////////////////////////////////////////////////
 template<typename T>
 inline T& matrix<T>::operator()(int x, int y) {
 assert(valid(x,y));
 return m_cells[y * m_width + x];
 }
 ////////////////////////////////////////////////////////////////////////////////
 template<typename T>
 inline bool matrix<T>::valid(int x, int y) const {
 return (x >= 0 and x < int(m_width) and y >= 0 and y < int(m_height));
 }
 ////////////////////////////////////////////////////////////////////////////////
 template<typename T>
 inline unsigned int matrix<T>::width() const {
 return m_width;
 }
 ////////////////////////////////////////////////////////////////////////////////
 template<typename T>
 inline unsigned int matrix<T>::height() const {
 return m_height;
 }
 ////////////////////////////////////////////////////////////////////////////////
 // */
#endif /* MATRIX_H */
