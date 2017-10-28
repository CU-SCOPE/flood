/**************************************************************************
**
**  svd3
**
** Quick singular value decomposition as described by: 
** A. McAdams, A. Selle, R. Tamstorf, J. Teran and E. Sifakis, 
** "Computing the Singular Value Decomposition of 3x3 matrices 
** with minimal branching and elementary floating point operations",
**  University of Wisconsin - Madison technical report TR1690, May 2011
**  
**  OPTIMIZED CPU VERSION
**  Implementation by: Eric Jang
**   
**  13 Apr 2014
**
**************************************************************************/


#ifndef SVD_H
#define SVD_H

#define _gamma 5.828427124 // FOUR_GAMMA_SQUARED = sqrt(8)+3;
#define _cstar 0.923879532 // cos(pi/8)
#define _sstar 0.3826834323 // sin(p/8)
#define EPSILON 1e-6

#include <math.h>

/* This is a novel and fast routine for the reciprocal square root of an
IEEE float (single precision).
http://www.lomont.org/Math/Papers/2003/InvSqrt.pdf
http://playstation2-linux.com/download/p2lsd/fastrsqrt.pdf
http://www.beyond3d.com/content/articles/8/
*/

#ifdef __cplusplus
extern "C" {
#endif

	void runSVD(float W[3][3], float U[3][3], float V[3][3]);

#ifdef __cplusplus
} //end extern "C"
#endif


#endif