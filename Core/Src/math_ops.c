
#include "math_ops.h"
#include "lookup.h"


float fast_fmaxf(float x, float y){
    /// Returns maximum of x, y ///
    return (((x)>(y))?(x):(y));
    }

float fast_fminf(float x, float y){
    /// Returns minimum of x, y ///
    return (((x)<(y))?(x):(y));
    }

float fmaxf3(float x, float y, float z){
    /// Returns maximum of x, y, z ///
    return (x > y ? (x > z ? x : z) : (y > z ? y : z));
    }

float fminf3(float x, float y, float z){
    /// Returns minimum of x, y, z ///
    return (x < y ? (x < z ? x : z) : (y < z ? y : z));
    }
/*
float roundf(float x){
    /// Returns nearest integer ///
    return x < 0.0f ? ceilf(x - 0.5f) : floorf(x + 0.5f);
    }
  */
void limit_norm(float *x, float *y, float limit){
    /// Scales the lenght of vector (x, y) to be <= limit ///
    float norm = sqrtf(*x * *x + *y * *y);
    if(norm > limit){
        *x = *x * limit/norm;
        *y = *y * limit/norm;
        }
    }
    
void limit(float *x, float min, float max){
    *x = fast_fmaxf(fast_fminf(*x, max), min);
    }

int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }
    
    
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }

void sincos_lut(float theta, float *s, float *c){
	/* Table-based sine and cosine with linear interpolation.
	 * One range reduction serves both outputs; cosine comes from the same
	 * table a quarter turn ahead.  Max error ~2e-5, ~30 cycles for the pair.
	 *
	 * Range reduction is done by multiplication instead of fmodf, which is
	 * exact to a small fraction of a table step for |theta| up to a few
	 * thousand radians - far beyond the largest angle used here (one
	 * mechanical revolution of electrical angle during calibration) */
	float k = theta*INV_TWO_PI_F;			// angle in revolutions
	k = k - (float)(int)k;					// fractional revolution, (-1, 1)
	if(k < 0.0f){k += 1.0f;}				// [0, 1)...
	if(k >= 1.0f){k = 0.0f;}				// ...except -epsilon + 1.0f can round to exactly 1.0f

	float idx = k*512.0f;
	int i = (int)idx;						// 0-511
	float frac = idx - (float)i;

	float s0 = sin_tab[i];					// sin_tab has 513 entries, so i+1 is always valid
	*s = s0 + frac*(sin_tab[i+1] - s0);
	int j = (i + 128) & 511;				// cos(theta) = sin(theta + pi/2) = 128 steps ahead
	float c0 = sin_tab[j];
	*c = c0 + frac*(sin_tab[j+1] - c0);
}

float sin_lut(float theta){
	float s, c;
	sincos_lut(theta, &s, &c);
	return s;
}

float cos_lut(float theta){
	float s, c;
	sincos_lut(theta, &s, &c);
	return c;
}
