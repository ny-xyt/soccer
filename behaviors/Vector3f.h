#ifndef _VECTOR3F_H_
#define _VECTOR3F_H_

#include <math.h>
#include <iostream>
#include <string>
#include <sstream>

using namespace std;

class Vector3f
{
public:
    float x, y, z;

    Vector3f();
    Vector3f(float x, float y, float z);
    Vector3f(const Vector3f &v);

    void set(float x, float y, float z);
    void set(const Vector3f &v);

    float length() const;
    float lengthSquared() const;
    float distance(const Vector3f &v) const;
    float distanceSquared(const Vector3f &v) const;
    float angle() const;
    float angle(const Vector3f &v) const;
    float dot(const Vector3f &v) const;
    Vector3f cross(const Vector3f &v) const;

    void normalize();
    void rotateX(float radians);
    void rotateY(float radians);
    void rotateZ(float radians);

    bool equals(const Vector3f &v, float errorMargin = 0.0001) const;

    string toString() const;

    static const Vector3f ZERO;
    static const Vector3f X_AXIS;
    static const Vector3f Y_AXIS;
    static const Vector3f Z_AXIS;

    // Operators
    inline bool operator==(const Vector3f &v) const
    {
        return x == v.x && y == v.y && z == v.z;
    }

    inline bool operator!=(const Vector3f &v) const
    {
        return x != v.x || y != v.y || z != v.z;
    }

    inline bool operator<(const Vector3f &v) const
    {
        return x < v.x && y < v.y && z < v.z;
    }

    inline bool operator>(const Vector3f &v) const
    {
        return x > v.x && y > v.y && z > v.z;
    }

    inline bool operator<=(const Vector3f &v) const
    {
        return x <= v.x && y <= v.y && z <= v.z;
    }

    inline bool operator>=(const Vector3f &v) const
    {
        return x >= v.x && y >= v.y && z >= v.z;
    }

    inline float& operator[](int i)
    {
        if (i == 0)
            return x;
        else if (i == 1)
            return y;
        else
            return z;
    }

    inline const float& operator[](int i) const
    {
        if (i == 0)
            return x;
        else if (i == 1)
            return y;
        else
            return z;
    }

#define VECTOR_OP(op)\
inline Vector3f& operator op (const Vector3f &v)\
{\
x op v.x;\
y op v.y;\
z op v.z;\
return *this;\
}\
inline Vector3f& operator op (float f)\
{\
x op f;\
y op f;\
z op f;\
return *this;\
}

#define VECTOR_ROP(op)\
inline friend Vector3f operator op (const Vector3f &a, const Vector3f &b)\
{\
return Vector3f(a.x op b.x, a.y op b.y, a.z op b.z);\
}\
inline friend Vector3f operator op (float f, const Vector3f &v)\
{\
return Vector3f(f op v.x, f op v.y, f op v.z);\
}\
inline friend Vector3f operator op (const Vector3f &v, float f)\
{\
return Vector3f(v.x op f, v.y op f, v.z op f);\
}

#define VECTOR_MOP(op)\
inline friend ostream& operator op (ostream& os, const Vector3f &v)\
{\
os << v.toString();\
return os;\
}

    VECTOR_OP(+=)
    VECTOR_OP(-=)
    VECTOR_OP(*=)
    VECTOR_OP(/=)

    VECTOR_ROP(+)
    VECTOR_ROP(-)
    VECTOR_ROP(*)
    VECTOR_ROP(/)

    VECTOR_MOP(<<)

#undef VECTOR_OP
#undef VECTOR_ROP
#undef VECTOR_MOP
};

#endif
