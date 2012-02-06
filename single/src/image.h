/*
 * image.h
 *
 *  Created on: 05/04/2010
 *      Author: pablo
 */

#ifndef IMAGE_H_
#define IMAGE_H_

#include <opencv/cv.h>

struct pixel {
	unsigned char b, g, r;
};

class image {
public:
	inline image(const IplImage* cvimage) :
		m_image(cvimage) {
	}

	inline const pixel& operator()(int i, int j) const {
		return *((pixel *)(m_image->imageData + m_image->widthStep * i) + j);
	}

	pixel& operator()(int i, int j) {
		return *((pixel *)(m_image->imageData + m_image->widthStep * i) + j);
	}

private:
	const IplImage* m_image;
};

#endif /* IMAGE_H_ */
