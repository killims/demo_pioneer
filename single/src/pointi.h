/*
 * pointi.h
 *
 *  Created on: 08-abr-2009
 *      Author: pablo
 */

#ifndef POINTI_H_
#define POINTI_H_

#include <opencv/cv.h>

/*
 *
 */
class pointi {
public:

	pointi(int x, int y);

	int x, y;
	bool operator==(const pointi& p) const;
	bool operator<(const pointi& p) const;

	operator CvPoint() {
		return cvPoint(y, x);
	}
};

inline pointi::pointi(int px, int py) :
	x(px), y(py) {
}

inline
bool pointi::operator==(const pointi& p) const {
	return x == p.x and y == p.y;
}

inline
bool pointi::operator<(const pointi& p) const {
	return x < p.x or (x == p.x and y < p.y);
}

#endif /* POINTI_H_ */
