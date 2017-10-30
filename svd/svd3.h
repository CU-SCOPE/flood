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
**	OPTIMIZED CPU VERSION
** 	Implementation by: Eric Jang
**	 
**  13 Apr 2014
**
**************************************************************************/


#ifndef SVD3_H
#define SVD3_H

#define _gamma 5.828427124 // FOUR_GAMMA_SQUARED = sqrt(8)+3;
#define _cstar 0.923879532 // cos(pi/8)
#define _sstar 0.3826834323 // sin(p/8)
#define EPSILON 1e-6

#ifdef __cplusplus
extern "C" {
#endif

	void runSVD(float a[3][3], float u[3][3], float v[3][3]);
	void matMul3D(float a[3][3], float b[3][3], float result[3][3]);

#ifdef __cplusplus
}
#endif

#endif