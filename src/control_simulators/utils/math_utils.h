#ifndef _MATH_UTILS_H_
#define _MATH_UTILS_H_

// From http://stackoverflow.com/a/4609795
template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

#endif
