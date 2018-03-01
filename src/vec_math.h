#ifndef VEC_MATH_H
#define VEC_MATH_H

#include <math.h>

static inline float dot3D(float *x, float *y) {
	return x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
}

static inline void sub3D(float *x, float *y, float *result) {
	result[0] = x[0] - y[0];
	result[1] = x[1] - y[1];
	result[2] = x[2] - y[2];
}

static inline void add3D(float *x, float *y, float *result) {
	result[0] = x[0] + y[0];
	result[1] = x[1] + y[1];
	result[2] = x[2] + y[2];
}

static inline void elemMul3D(float *x, float *y, float *result) {
	result[0] = x[0] * y[0];
	result[1] = x[1] * y[1];
	result[2] = x[2] * y[2];
}

static inline void scalarMul3D(float scal, float *vec) {
	vec[0] *= scal;
	vec[1] *= scal;
	vec[2] *= scal;
}

static inline void matMulVec4D(float mat[4][4], float *vec, float *result) {
	result[0] = (mat[0][0]*vec[0]) + (mat[0][1]*vec[1]) + (mat[0][2]*vec[2]) + (mat[0][3]*vec[3]);
	result[1] = (mat[1][0]*vec[0]) + (mat[1][1]*vec[1]) + (mat[1][2]*vec[2]) + (mat[1][3]*vec[3]);
	result[2] = (mat[2][0]*vec[0]) + (mat[2][1]*vec[1]) + (mat[2][2]*vec[2]) + (mat[2][3]*vec[3]);
	result[3] = (mat[3][0]*vec[0]) + (mat[3][1]*vec[1]) + (mat[3][2]*vec[2]) + (mat[3][3]*vec[3]);
}

static inline void matMulVec3D(float mat[3][3], float *vec, float *result) {
	result[0] = (mat[0][0]*vec[0]) + (mat[0][1]*vec[1]) + (mat[0][2]*vec[2]);
	result[1] = (mat[1][0]*vec[0]) + (mat[1][1]*vec[1]) + (mat[1][2]*vec[2]);
	result[2] = (mat[2][0]*vec[0]) + (mat[2][1]*vec[1]) + (mat[2][2]*vec[2]);
}

static inline void eye4D(float mat[4][4]) {
	mat[0][0] = 1;
	mat[1][1] = 1;
	mat[2][2] = 1;
	mat[3][3] = 1;
}

static inline void eye3D(float mat[3][3]) {
	mat[0][0] = 1;
	mat[1][1] = 1;
	mat[2][2] = 1;
}

static inline void meanVec(float *vec, float *mean, unsigned int numPts) {
	unsigned int i;
	float sum;
	for(i=0; i<numPts; i++) {
		sum += vec[i];
	}
	(*mean) = sum/numPts;
}

static inline void stdev(float *vec, float *s, float mean, unsigned int numPts) {
	unsigned int i;
	float sum = 0, tmp;
	for(i=0; i<numPts; i++) {
		tmp = vec[i] - mean;
		sum += tmp * tmp;
	}
	*s = sqrt(sum/(numPts-1));
}

static inline unsigned int remOutliers(float *vec, float thresh, unsigned int numPts, point4D *modelSrc, point4D *scanSrc, point4D *modelDst, point4D *scanDst) {
	unsigned int i;
	unsigned int counter = 0;
	for(i=0; i<numPts; i++) {
		if(vec[i] < thresh) {
			modelDst[counter] = modelSrc[i];
			scanDst[counter] = scanSrc[i];
			counter++;
		}
	}
	return counter;
}

static inline void transpose(float a[3][3], float aT[3][3]) {
	aT[0][0] = a[0][0];
	aT[0][1] = a[1][0];
	aT[0][2] = a[2][0];
	aT[1][0] = a[0][1];
	aT[1][1] = a[1][1];
	aT[1][2] = a[2][1];
	aT[2][0] = a[0][2];
	aT[2][1] = a[1][2];
	aT[2][2] = a[2][2];
}

static inline float determinant(float R[3][3]) {
	return R[0][0]*(R[1][1]*R[2][2] - R[1][2]*R[2][1]) - R[0][1]*(R[1][0]*R[2][2] - R[1][2]*R[2][0]) + R[0][2]*(R[0][1]*R[2][1] - R[1][1]*R[2][0]);
}

static inline void getMat(point4D *scan, point4D *model, float W[3][3], unsigned int numPts) {
	unsigned int i;
	for(i=0; i<numPts; i++) {
		W[0][0] += scan[i].point[0] * model[i].point[0];
		W[0][1] += scan[i].point[0] * model[i].point[1];
		W[0][2] += scan[i].point[0] * model[i].point[2];
		W[1][0] += scan[i].point[1] * model[i].point[0];
		W[1][1] += scan[i].point[1] * model[i].point[1];
		W[1][2] += scan[i].point[1] * model[i].point[2];
		W[2][0] += scan[i].point[2] * model[i].point[0];
		W[2][1] += scan[i].point[2] * model[i].point[1];
		W[2][2] += scan[i].point[2] * model[i].point[2];
	}
}

static inline void matMul3D(float a[3][3], float b[3][3], float c[3][3]) {

   float m[24]; // not off by one, just wanted to match the index from the paper

   m[1 ]= (a[0][0]+a[0][1]+a[0][2]-a[1][0]-a[1][1]-a[2][1]-a[2][2])*b[1][1];
   m[2 ]= (a[0][0]-a[1][0])*(-b[0][1]+b[1][1]);
   m[3 ]= a[1][1]*(-b[0][0]+b[0][1]+b[1][0]-b[1][1]-b[1][2]-b[2][0]+b[2][2]);
   m[4 ]= (-a[0][0]+a[1][0]+a[1][1])*(b[0][0]-b[0][1]+b[1][1]);
   m[5 ]= (a[1][0]+a[1][1])*(-b[0][0]+b[0][1]);
   m[6 ]= a[0][0]*b[0][0];
   m[7 ]= (-a[0][0]+a[2][0]+a[2][1])*(b[0][0]-b[0][2]+b[1][2]);
   m[8 ]= (-a[0][0]+a[2][0])*(b[0][2]-b[1][2]);
   m[9 ]= (a[2][0]+a[2][1])*(-b[0][0]+b[0][2]);
   m[10]= (a[0][0]+a[0][1]+a[0][2]-a[1][1]-a[1][2]-a[2][0]-a[2][1])*b[1][2];
   m[11]= a[2][1]*(-b[0][0]+b[0][2]+b[1][0]-b[1][1]-b[1][2]-b[2][0]+b[2][1]);
   m[12]= (-a[0][2]+a[2][1]+a[2][2])*(b[1][1]+b[2][0]-b[2][1]);
   m[13]= (a[0][2]-a[2][2])*(b[1][1]-b[2][1]);
   m[14]= a[0][2]*b[2][0];
   m[15]= (a[2][1]+a[2][2])*(-b[2][0]+b[2][1]);
   m[16]= (-a[0][2]+a[1][1]+a[1][2])*(b[1][2]+b[2][0]-b[2][2]);
   m[17]= (a[0][2]-a[1][2])*(b[1][2]-b[2][2]);
   m[18]= (a[1][1]+a[1][2])*(-b[2][0]+b[2][2]);
   m[19]= a[0][1]*b[1][0];
   m[20]= a[1][2]*b[2][1];
   m[21]= a[1][0]*b[0][2];
   m[22]= a[2][0]*b[0][1];
   m[23]= a[2][2]*b[2][2];

  c[0][0] = m[6]+m[14]+m[19];
  c[0][1] = m[1]+m[4]+m[5]+m[6]+m[12]+m[14]+m[15];
  c[0][2] = m[6]+m[7]+m[9]+m[10]+m[14]+m[16]+m[18];
  c[1][0] = m[2]+m[3]+m[4]+m[6]+m[14]+m[16]+m[17];
  c[1][1] = m[2]+m[4]+m[5]+m[6]+m[20];
  c[1][2] = m[14]+m[16]+m[17]+m[18]+m[21];
  c[2][0] = m[6]+m[7]+m[8]+m[11]+m[12]+m[13]+m[14];
  c[2][1] = m[12]+m[13]+m[14]+m[15]+m[22];
  c[2][2] = m[6]+m[7]+m[8]+m[9]+m[23];    
}

static inline void triangleDist(face tri, float *point, float *dist, float *closestPt) {
	float B[3] = {tri.v1.x, tri.v1.y, tri.v1.z};
	float E0[3] = {tri.v2.x - B[0], tri.v2.y - B[1], tri.v2.z - B[2]};
	float E1[3] = {tri.v3.x - B[0], tri.v3.y - B[1], tri.v3.z - B[2]};
	float D[3];
	sub3D(B, point, D);
	float a = dot3D(E0, E0);
	float b = dot3D(E0, E1);
	float c = dot3D(E1, E1);
	float d = dot3D(E0, D);
	float e = dot3D(E1, D);
	float f = dot3D(D, D);
	float det = a*c - b*b;
	float s = b*e - c*d;
	float t = b*d - a*e;
	// printf("%f   %f   %f   %f   %f\n", B[0],B[1],B[2],E1[0],E1[1]);
	unsigned int region;
	float tmp1, tmp0, numer, denom, invDet;

	if((s+t) <= det) {
		if(s < 0) {
			if(t < 0) {
				region = 4;
			} else {
				region = 3;
			}
		} else if(t < 0) {
			region = 5;
		} else {
			region = 0;
		}
	} else {
		if(s < 0) {
			region = 2;
		} else if(t < 0) {
			region = 6;
		} else {
			region = 1;
		}
	}
	switch(region) {
		case 0:
			invDet = 1/det;
			s *= invDet;
			t *= invDet;
			*(dist) = s * (a * s + b * t + 2.0 * d) + t * (b * s + c * t + 2.0 * e) + f;
		case 1:
			numer = (c + e) - (b + d);
			if(numer <= 0) {
				s = 0;
				t = 1;
				*(dist) = c + 2*e + f;
			} else {
				denom = a - 2*b + c;
				if(numer >= denom) {
					s = 1;
					t = 0;
					*(dist) = a + 2 * d + f;
				} else {
					s = numer/denom;
					t = 1-s;
					*(dist) = s * (a * s + b * t + 2.0 * d) + t * (b * s + c * t + 2.0 * e) + f;
				}
			}
		case 2:
			tmp0 = b + d;
			tmp1 = c + e;
			if(tmp1 > tmp0){
				numer = tmp1 - tmp0;
				denom = a - 2.0 * b + c;
				if(numer >= denom){
					s = 1.0;
					t = 0.0;
					*(dist) = a + 2.0 * d + f;
				} else {
					s = numer / denom;
					t = 1 - s;
					*(dist) = s * (a * s + b * t + 2 * d) + t * (b * s + c * t + 2 * e) + f;
				}
			} else {
				s = 0.0;
				if(tmp1 <= 0.0) {
					t = 1;
					*(dist) = c + 2.0 * e + f;
				} else {
					if(e >= 0.0){
						t = 0.0;
						*(dist) = f;
					} else {
						t = -e / c;
						*(dist) = e * t + f;
					}
				}
			}
		case 3:
			s = 0;
			if(e >= 0) {
				t = 0;
				*(dist) = f;
			} else {
				if(-e >= c) {
					t = 1;
					*(dist) = c + 2.0 * e + f;
				} else {
					t = -e / c;
					*(dist) = e * t + f;
				}
			}
		case 4:
			if(d < 0) {
				t = 0.0;
				if(-d >= a) {
					s = 1.0;
					*(dist) = a + 2.0 * d + f;
				} else {
					s = -d / a;
					*(dist) = d * s + f;
				}
			} else {
				s = 0.0;
				if(e >= 0.0) {
					t = 0.0;
					*(dist) = f;
				} else {
					if(-e >= c) {
						t = 1.0;
						*(dist) = c + 2.0 * e + f;
					} else {
						t = -e / c;
						*(dist) = e * t + f;
					}
				}
			}
		case 5:
			t = 0;
			if(d >= 0) {
				s = 0;
				*(dist) = f;
			} else {
				if(-d >= a) {
					s = 1;
					*(dist) = a + 2.0 * d + f;
				} else {
					s = -d / a;
					*(dist) = d * s + f;
				}
			}
		case 6:
			tmp0 = b + e;
			tmp1 = a + d;
			if(tmp1 > tmp0) {
				numer = tmp1 - tmp0;
				denom = a - 2.0 * b + c;
				if(numer >= denom) {
					t = 1.0;
					s = 0;
					*(dist) = c + 2.0 * e + f;
				} else {
					t = numer / denom;
					s = 1 - t;
					*(dist) = s * (a * s + b * t + 2.0 * d) + t * (b * s + c * t + 2.0 * e) + f;
				}
			} else {
				t = 0.0;
				if(tmp1 <= 0.0) {
					s = 1;
					*(dist) = a + 2.0 * d + f;
				} else {
					if(d >= 0.0) {
						s = 0.0;
						*(dist) = f;
					} else {
						s = -d / a;
						*(dist) = d * s + f;	
					}
				}
			}
	}
	float temp[3];
	scalarMul3D(s, E0);
	scalarMul3D(t, E1);
	add3D(B, E0, temp);
	add3D(temp, E1, closestPt);
	if(*(dist) < 0) {
		*(dist) = 0.0;
	}
}


#endif