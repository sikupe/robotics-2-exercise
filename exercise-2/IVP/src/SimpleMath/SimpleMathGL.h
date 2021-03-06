#ifndef _SIMPLEMATHGL_H_
#define _SIMPLEMATHGL_H_

#include "SimpleMath.h"
#include <cmath>

inline Matrix44f smRotate (float rot_deg, float x, float y, float z) {
	float c = cosf (rot_deg * M_PI / 180.f);
	float s = sinf (rot_deg * M_PI / 180.f);
	return Matrix44f (
			x * x * (1.0f - c) + c,
			y * x * (1.0f - c) + z * s,
			x * z * (1.0f - c) - y * s,
			0.f, 

			x * y * (1.0f - c) - z * s,
			y * y * (1.0f - c) + c,
			y * z * (1.0f - c) + x * s,
			0.f,

			x * z * (1.0f - c) + y * s,
			y * z * (1.0f - c) - x * s,
			z * z * (1.0f - c) + c,
			0.f,

			0.f, 0.f, 0.f, 1.f
			);
}

inline Matrix44f smTranslate (float x, float y, float z) {
	return Matrix44f (
			1.f, 0.f, 0.f, 0.f,
			0.f, 1.f, 0.f, 0.f,
			0.f, 0.f, 1.f, 0.f,
			  x,   y,   z, 1.f
			);
}

inline Matrix44f smScale (float x, float y, float z) {
	return Matrix44f (
			  x, 0.f, 0.f, 0.f,
			0.f,   y, 0.f, 0.f,
			0.f, 0.f,   z, 0.f,
			0.f, 0.f, 0.f, 1.f
			);
}

/** Quaternion 
 *
 * order: x,y,z,w
 */
class smQuaternion : public Vector4f {
	public:
		smQuaternion () :
			Vector4f (0.f, 0.f, 0.f, 1.f)
		{}
		smQuaternion (const Vector4f vec4) :
			Vector4f (vec4)
		{}
		smQuaternion (float x, float y, float z, float w):
			Vector4f (x, y, z, w)
		{}
		/** This function is equivalent to multiplicate their corresponding rotation matrices */
		smQuaternion operator* (const smQuaternion &q) const {
			return smQuaternion (
					q[3] * (*this)[0] + q[0] * (*this)[3] + q[1] * (*this)[2] - q[2] * (*this)[1],
					q[3] * (*this)[1] + q[1] * (*this)[3] + q[2] * (*this)[0] - q[0] * (*this)[2],
					q[3] * (*this)[2] + q[2] * (*this)[3] + q[0] * (*this)[1] - q[1] * (*this)[0],
					q[3] * (*this)[3] - q[0] * (*this)[0] - q[1] * (*this)[1] - q[2] * (*this)[2]
					);
		}
		smQuaternion& operator*=(const smQuaternion &q) {
			set (
					q[3] * (*this)[0] + q[0] * (*this)[3] + q[1] * (*this)[2] - q[2] * (*this)[1],
					q[3] * (*this)[1] + q[1] * (*this)[3] + q[2] * (*this)[0] - q[0] * (*this)[2],
					q[3] * (*this)[2] + q[2] * (*this)[3] + q[0] * (*this)[1] - q[1] * (*this)[0],
					q[3] * (*this)[3] - q[0] * (*this)[0] - q[1] * (*this)[1] - q[2] * (*this)[2]
					);
			return *this;
		}

		static smQuaternion fromGLRotate (float angle, float x, float y, float z) {
			float st = sinf (angle * M_PI / 360.f);
			return smQuaternion (
						st * x,
						st * y,
						st * z,
						cosf (angle * M_PI / 360.f)
						);
		}

		smQuaternion normalize() {
			return Vector4f::normalize();
		}

		smQuaternion slerp (float alpha, const smQuaternion &quat) const {
			// check whether one of the two has 0 length
			float s = sqrt (squaredNorm() * quat.squaredNorm());

			// division by 0.f is unhealthy!
			assert (s != 0.f);

			float angle = acos (dot(quat) / s);
			if (angle == 0.f || isnan(angle)) {
				return *this;
			}
			assert(!isnan(angle));

			float d = 1.f / sinf (angle);
			float p0 = sinf ((1.f - alpha) * angle);
			float p1 = sinf (alpha * angle);

			if (dot (quat) < 0.f) {
				return smQuaternion( ((*this) * p0 - quat * p1) * d);
			}
			return smQuaternion( ((*this) * p0 + quat * p1) * d);
		}

		Matrix44f toGLMatrix() const {
			float x = (*this)[0];
			float y = (*this)[1];
			float z = (*this)[2];
			float w = (*this)[3];
			return Matrix44f (
					1 - 2*y*y - 2*z*z,
					2*x*y + 2*w*z,
					2*x*z - 2*w*y,
					0.f,

					2*x*y - 2*w*z,
					1 - 2*x*x - 2*z*z,
					2*y*z + 2*w*x,
					0.f,

					2*x*z + 2*w*y,
					2*y*z - 2*w*x,
					1 - 2*x*x - 2*y*y,
					0.f,
					
					0.f,
					0.f,
					0.f,
					1.f);
		}

		smQuaternion conjugate() const {
			return smQuaternion (
					-(*this)[0],
					-(*this)[1],
					-(*this)[2],
					(*this)[3]);
		}

		Vector3f rotate (const Vector3f &vec) const {
			Vector3f vn (vec);
			vn.normalize();

			smQuaternion vec_quat (vn[0], vn[1], vn[2], 0.f), res_quat;

			res_quat = vec_quat * (*this);
			res_quat = conjugate() * res_quat;

			return Vector3f (res_quat[0], res_quat[1], res_quat[2]);
		}
};

/* _SIMPLEMATHGL_H_ */
#endif
