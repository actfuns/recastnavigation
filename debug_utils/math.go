package debug_utils

import "math"

// sqrt returns the square root of x.
func sqrt(x float32) float32 {
	return float32(math.Sqrt(float64(x)))
}

// cos returns the cosine of x.
func cos(x float32) float32 {
	return float32(math.Cos(float64(x)))
}

// sin returns the sine of x.
func sin(x float32) float32 {
	return float32(math.Sin(float64(x)))
}

// fabs returns the absolute value of x.
func fabs(x float32) float32 {
	if x < 0 {
		return -x
	}
	return x
}
