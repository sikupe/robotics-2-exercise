#ifndef _SIMPLEMATH_H
#define _SIMPLEMATH_H

#include "SimpleMathFixed.h"
#include "SimpleMathDynamic.h"
#include "SimpleMathMixed.h"

typedef SimpleMath::Fixed::Matrix<int, 3, 1> Vector3i;

typedef SimpleMath::Fixed::Matrix<double, 3, 1> Vector3d;
typedef SimpleMath::Fixed::Matrix<double, 3, 3> Matrix33d;

typedef SimpleMath::Fixed::Matrix<double, 4, 1> Vector4d;

typedef SimpleMath::Fixed::Matrix<float, 3, 1> Vector3f;
typedef SimpleMath::Fixed::Matrix<float, 4, 1> Vector4f;
typedef SimpleMath::Fixed::Matrix<float, 3, 3> Matrix33f;
typedef SimpleMath::Fixed::Matrix<float, 4, 4> Matrix44f;

typedef SimpleMath::Dynamic::Matrix<double> MatrixNd;
typedef SimpleMath::Dynamic::Matrix<double> VectorNd;
typedef SimpleMath::Dynamic::Matrix<long double> VectorNdd;

typedef SimpleMath::Dynamic::Matrix<float> MatrixNf;
typedef SimpleMath::Dynamic::Matrix<float> VectorNf;

#endif /* _SIMPLEMATH_H */
