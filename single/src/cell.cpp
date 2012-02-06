/*
 * cell.cpp
 *
 *  Created on: 30/03/2010
 *      Author: pablo
 */

#include <algorithm>
#include "cell.h"

////////////////////////////////////////////////////////////////////////////////
cell::type cell::diff(cell::type a, cell::type b) {
	if (b == cell::NONFREE and a != cell::NONFREE)
		return TONONFREE;

	if (b == cell::FREE and a == cell::NONFREE)
		return cell::TOFREE;

	return cell::NOCHANGE;
}
////////////////////////////////////////////////////////////////////////////////
cell::type cell::patch(cell::type a, cell::type b) {
	if (b == cell::UNKNOWN)
		return a;
	else
		return b;
}
////////////////////////////////////////////////////////////////////////////////
cell::type cell::mask(cell::type a, cell::type b) {
	return std::max(a, b);
}
////////////////////////////////////////////////////////////////////////////////
cell::type cell::fuse(cell::type a, cell::type b) {
	if (a == cell::UNKNOWN)
		return b;

	if (b == cell::UNKNOWN)
		return a;

	if (a == b)
		return a;

	return cell::UNKNOWN;
}
////////////////////////////////////////////////////////////////////////////////
