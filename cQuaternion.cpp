#include "cquaternion.h"

#include <math.h>

cQuaternion::cQuaternion()
{
	a = 1;
	b = c = d = 0;
}

cQuaternion cQuaternion::operator*(const cQuaternion &other)
{
	cQuaternion result;

	float a1,b1,c1,d1;
	float a2,b2,c2,d2;

	a1 = a;
	b1 = b;
	c1 = c;
	d1 = d;

	a2 = other.a;
	b2 = other.b;
	c2 = other.c;
	d2 = other.d;

	result.a = a1*a2 - b1*b2 - c1*c2 - d1*d2;
	result.b = a1*b2 + b1*a2 + c1*d2 - d1*c2;
	result.c = a1*c2 - b1*d2 + c1*a2 + d1*b2;
	result.d = a1*d2 + b1*c2 - c1*b2 + d1*a2;

	return result;
}

cQuaternion cQuaternion::operator*(const float scalar)
{
	cQuaternion result;

	result.a = a * scalar;
	result.b = b * scalar;
	result.c = c * scalar;
	result.d = d * scalar;

	return result;
}


cQuaternion cQuaternion::operator+(const cQuaternion &other)
{
	cQuaternion result;
	result.a = a + other.a;
	result.b = b + other.b;
	result.c = c + other.c;
	result.d = d + other.d;
	return result;
}

cQuaternion cQuaternion::operator-(const cQuaternion &other)
{
	cQuaternion result;
	result.a = a - other.a;
	result.b = b - other.b;
	result.c = c - other.c;
	result.d = d - other.d;
	return result;
}

cQuaternion cQuaternion::conjugate()
{
	cQuaternion result;
	result.a = a;
	result.b = -b;
	result.c = -c;
	result.d = -d;
	return result;
}

void cQuaternion::normalize()
{
	float norm = sqrt(a*a + b*b + c*c + d*d);

	a = a/norm;
	b = b/norm;
	c = c/norm;
	d = d/norm;
}

