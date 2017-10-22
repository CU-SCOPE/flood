#ifndef VEC_MATH_H
#define VEC_MATH_H

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



static inline void triangleDist(face tri, float *point, float *dist, float *closestPt) {
	float B[3] = {tri.v1.x, tri.v1.y, tri.v1.z};
	float E0[3] = {tri.v2.x - B[0], tri.v2.y - B[0], tri.v2.z - B[0]};
	float E1[3] = {tri.v3.x - B[0], tri.v3.y - B[0], tri.v3.z - B[0]};
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
	uint8_t region;
	float tmp1, tmp0, numer, denom, invDet;

	if(s+t <= det) {
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
			*dist = s * (a*s + b*t + 2*d) + t * (b*s + c*t + 2*e) + f;
		case 1:
			numer = c + e - b - d;
			if(numer <= 0) {
				s = 0;
				t = 1;
				*dist = c + 2*e + f;
			} else {
				denom = a - 2*b + c;
				if(numer >= denom) {
					s = 1;
					t = 0;
					*dist = a + 2 * d + f;
				} else {
					s = numer/denom;
					t = 1-s;
					*dist = s * (a*s + b*t + 2*d) + t * (b*s + c*t + 2*e) + f;
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
					*dist = a + 2.0 * d + f;
				} else {
					s = numer / denom;
					t = 1 - s;
					*dist = s * (a * s + b * t + 2 * d) + t * (b * s + c * t + 2 * e) + f;
				}
			} else {
				s = 0.0;
				if(tmp1 <= 0.0) {
					t = 1;
					*dist = c + 2.0 * e + f;
				} else {
					if(e >= 0.0){
						t = 0.0;
						*dist = f;
					} else {
						t = -e / c;
						*dist = e * t + f;
					}
				}
			}
		case 3:
			s = 0;
			if(e >= 0) {
				t = 0;
				*dist = f;
			} else {
				if(-e >= c) {
					t = 1;
					*dist = c + 2.0 * e + f;
				} else {
					t = -e / c;
					*dist = e * t + f;
				}
			}
		case 4:
			if(d < 0) {
				t = 0.0;
				if(-d >= a) {
					s = 1.0;
					*dist = a + 2.0 * d + f;
				} else {
					s = -d / a;
					*dist = d * s + f;
				}
			} else {
				s = 0.0;
				if(e >= 0.0) {
					t = 0.0;
					*dist = f;
				} else {
					if(-e >= c) {
						t = 1.0;
						*dist = c + 2.0 * e + f;
					} else {
						t = -e / c;
						*dist = e * t + f;
					}
				}
			}
		case 5:
			t = 0;
			if(d >= 0) {
				s = 0;
				*dist = f;
			} else {
				if(-d >= a) {
					s = 1;
					*dist = a + 2.0 * d + f;
				} else {
					s = -d / a;
					*dist = d * s + f;
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
					*dist = c + 2.0 * e + f;
				} else {
					t = numer / denom;
					s = 1 - t;
					*dist = s * (a * s + b * t + 2.0 * d) + t * (b * s + c * t + 2.0 * e) + f;
				}
			} else {
				t = 0.0;
				if(tmp1 <= 0.0) {
					s = 1;
					*dist = a + 2.0 * d + f;
				} else {
					if(d >= 0.0) {
						s = 0.0;
						*dist = f;
					} else {
						s = -d / a;
						*dist = d * s + f;	
					}
				}
			}
	}
	float temp[3];
	scalarMul3D(s, E0);
	scalarMul3D(t, E1);
	add3D(B, E0, temp);
	add3D(temp, E1, closestPt);
}


#endif