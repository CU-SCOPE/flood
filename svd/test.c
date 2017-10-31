#include <stdio.h>
#include "svd3.h"

// some printing utilities
inline void printMat3(float a11, float a12, float a13,
               float a21, float a22, float a23,
               float a31, float a32, float a33)
{
    printf("%f %f %f \n", a11, a12, a13);
    printf("%f %f %f \n", a21, a22, a23);
    printf("%f %f %f \n", a31, a32, a33);
}

inline void printQuat(float * q)
{
    // print w,x,y,z
    printf("%f %f %f %f\n",q[3],q[0],q[1],q[2]);
}


int main(void)
{
    float a[3][3];

    a[0][0]= -0.558253; a[0][1] = -0.0461681; a[0][2] = -0.505735;
    a[1][0] = -0.411397; a[1][1] = 0.0365854; a[1][2] = 0.199707;
    a[2][0] = 0.285389; a[2][1] =-0.313789; a[2][2] = 0.200189;

    // printf("Original Matrix:\n");
    // printMat3(a11, a12, a13, a21, a22, a23, a31, a32, a33);

    float u[3][3], v[3][3];

    for (int i=0; i<10; i++)
    {
        runSVD(a,u,v);
    }        

    return 0;

}
