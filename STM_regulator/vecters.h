#pragma once
#include <cmath>


typedef struct Vecter
{
    double x_proj, y_proj, z_proj;
    Vecter() {
        x_proj = 0.0;
        y_proj = 0.0;
        z_proj = 0.0;
    }
    Vecter(double x, double y, double z) {
        x_proj = x;
        y_proj = y;
        z_proj = z;
    }
    Vecter(double x) {
        x_proj = x;
        y_proj = 0.0;
        z_proj = 0.0;
    }
    inline double len()
    {
        return sqrt(x_proj*x_proj + y_proj*y_proj + z_proj*z_proj);
    }
    void set(double x, double y, double z)
    {
        x_proj = x;
        y_proj = y;
        z_proj = z;
    }
    inline Vecter Rotate_X(double angle)
    {
        Vecter tmp;
        tmp.x_proj = x_proj;
        tmp.y_proj = y_proj*cos(angle) - z_proj*sin(angle);
        tmp.z_proj = y_proj*sin(angle) + z_proj*cos(angle);
        return tmp;
    }
    inline Vecter Rotate_Y(double angle)
    {
        Vecter tmp;

        tmp.y_proj = y_proj;
        tmp.x_proj = x_proj*cos(angle) + z_proj*sin(angle);
        tmp.z_proj = -x_proj*sin(angle) + z_proj*cos(angle);
        return tmp;
    }
    inline Vecter Rotate_Z(double angle)
    {
        Vecter tmp;

        tmp.z_proj = z_proj;
        tmp.y_proj = x_proj*sin(angle) + y_proj*cos(angle);
        tmp.x_proj = x_proj*cos(angle) - y_proj*sin(angle);
        return tmp;
    }
    Vecter Rotate(double angle_x, double angle_y, double angle_z)
    {
        return this->Rotate_X(angle_x).Rotate_Y(angle_y).Rotate_Z(angle_z);
    }
    inline Vecter Normalize()
    {
        Vecter tmp;

        tmp.z_proj = z_proj / this->len();
        tmp.y_proj = x_proj / this->len();
        tmp.x_proj = x_proj / this->len();
        return tmp;
    }
} Vecter;


inline Vecter operator * (Vecter a, Vecter b)
{
    Vecter tmp;
    tmp.x_proj = (a.y_proj * b.z_proj) - (a.z_proj * b.y_proj);
    tmp.y_proj = (a.z_proj * b.x_proj) - (a.x_proj * b.z_proj);
    tmp.z_proj = (a.x_proj * b.y_proj) - (a.y_proj * b.x_proj);
    return tmp;
}
inline Vecter operator + (Vecter a, Vecter b)
{
    Vecter tmp;
    tmp.x_proj = a.x_proj + b.x_proj;
    tmp.y_proj = a.y_proj + b.y_proj;
    tmp.z_proj = a.z_proj + b.z_proj;
    return tmp;
}
inline Vecter operator - (Vecter a, Vecter b)
{
    Vecter tmp;
    tmp.x_proj = a.x_proj - b.x_proj;
    tmp.y_proj = a.y_proj - b.y_proj;
    tmp.z_proj = a.z_proj - b.z_proj;
    return tmp;
}
inline Vecter operator *(Vecter a, double b)
{
    Vecter tmp;
    tmp.x_proj = a.x_proj * b;
    tmp.y_proj = a.y_proj * b;
    tmp.z_proj = a.z_proj * b;
    return tmp;
}
inline Vecter operator /(Vecter a, double b)
{
    Vecter tmp;
    tmp.x_proj = a.x_proj / b;
    tmp.y_proj = a.y_proj / b;
    tmp.z_proj = a.z_proj / b;
    return tmp;
}
inline Vecter operator *(double b, Vecter a)
{
    Vecter tmp;
    tmp.x_proj = a.x_proj * b;
    tmp.y_proj = a.y_proj * b;
    tmp.z_proj = a.z_proj * b;
    return tmp;
}
inline Vecter&  operator *=(Vecter &a, const double b)
{

    Vecter tmp;
    tmp.x_proj = a.x_proj * b;
    tmp.y_proj = a.y_proj * b;
    tmp.z_proj = a.z_proj * b;
    a = tmp;
    return a;

}
inline Vecter&  operator /=(Vecter &a, const double b)
{

    Vecter tmp;
    tmp.x_proj = a.x_proj / b;
    tmp.y_proj = a.y_proj / b;
    tmp.z_proj = a.z_proj / b;
    a = tmp;
    return a;

}
inline double operator & (Vecter a, Vecter b)
{
    double tmp;
    tmp = a.x_proj * b.x_proj + a.y_proj * b.y_proj + a.z_proj * b.z_proj;
    return tmp;
}
inline Vecter&  operator += (Vecter &a, Vecter b)
{
    Vecter tmp;
    tmp.x_proj = a.x_proj + b.x_proj;
    tmp.y_proj = a.y_proj + b.y_proj;
    tmp.z_proj = a.z_proj + b.z_proj;
    a = tmp;
    return a;
}

inline Vecter&  operator *= (Vecter &a, Vecter b)
{
    Vecter tmp;
    tmp = a;
    tmp.x_proj = (a.y_proj * b.z_proj) - (a.z_proj * b.y_proj);
    tmp.y_proj = (a.z_proj * b.x_proj) - (a.x_proj * b.z_proj);
    tmp.z_proj = (a.x_proj * b.y_proj) - (a.y_proj * b.x_proj);
    a = tmp;
    return a;
}

inline  Vecter& operator -= (Vecter &a, const Vecter b)
{
    Vecter tmp;
    tmp.x_proj = a.x_proj - b.x_proj;
    tmp.y_proj = a.y_proj - b.y_proj;
    tmp.z_proj = a.z_proj - b.z_proj;
    a = tmp;
    return a;
}
inline Vecter Vec_Rotate_X(Vecter vec, double angle)
{

    Vecter tmp;
    tmp.x_proj = vec.x_proj;
    tmp.y_proj = vec.y_proj*cos(angle) - vec.z_proj*sin(angle);
    tmp.z_proj = vec.y_proj*sin(angle) + vec.z_proj*cos(angle);
    return tmp;
}
inline Vecter operator -(Vecter vec)
{
    Vecter tmp;
    tmp.x_proj = -vec.x_proj;
    tmp.y_proj = -vec.y_proj;
    tmp.z_proj = -vec.z_proj;
    return tmp;
}

inline int sign(double x)
{
    if (x > 0) { return 1; }
    else if (x < 0) { return -1; }
    else return 0;
}
