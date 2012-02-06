/*
 * cell.h
 *
 *  Created on: 30-mar-2009
 *      Author: pablo
 */

#ifndef CELL_H_
#define CELL_H_

namespace cell {

typedef char type;

const char UNKNOWN = 0;
const char FREE = 1;
const char NONFREE = 2;
const char NOCHANGE = 3;
const char TONONFREE = 4;
const char TOFREE = 5;

type diff(type a, type b);

type patch(type a, type b);

type mask(type a, type b);

type fuse(type a, type b);

}

#endif /* CELL_H_ */
