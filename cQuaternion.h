#ifndef CQUATERNION_H
#define CQUATERNION_H

class cQuaternion
{
	public:
	cQuaternion();

	cQuaternion operator*(const cQuaternion &other);
	cQuaternion operator*(const float scalar);
	cQuaternion operator+(const cQuaternion &other);
	cQuaternion operator-(const cQuaternion &other);
	cQuaternion conjugate();
	void normalize();

	float a,b,c,d;
};

#endif // CQUATERNION_H
