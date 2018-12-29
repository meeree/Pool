#ifndef  __QUATERNION_H__
#define  __QUATERNION_H__

#include "core.h"

struct Quaternion 
{
    double s; 
    arma::vec3 v;

    Quaternion () = default;
    Quaternion (double const& s_, arma::vec3 const& v_) : s{s_}, v{v_} {}
    Quaternion (double const& s_, double const& x, double const& y, double const& z) : s{s_}, v{x,y,z} {}
    Quaternion (arma::vec3 const& v_) : s{0.0f}, v{v_} {}

    inline static Quaternion UnitRotation (double const& t, arma::vec3 const& u) {return Quaternion(std::cos(t/2.0f), std::sin(t/2.0f)*u);}
    inline arma::vec4 ToVector () const {return {v[0], v[1], v[2], s};}

    inline double Norm () {return arma::norm(ToVector());}
    inline void Normalize () {double nm{Norm()}; s /= nm; v /= nm;}

    inline Quaternion operator+ (Quaternion const& rhs) {return {s+rhs.s, v+rhs.v};}
    inline Quaternion operator* (Quaternion const& rhs) {return {s*rhs.s-arma::dot(v,rhs.v), s*rhs.v+rhs.s*v+arma::cross(v,rhs.v)};}

    inline Quaternion& operator+= (Quaternion const& rhs) {return (*this = *this + rhs);}
    inline Quaternion& operator*= (Quaternion const& rhs) {return (*this = *this * rhs);}

    static arma::mat33 QuatToMat (Quaternion const& q);
};

inline Quaternion operator+ (Quaternion const& lhs, arma::vec3 const& rhs) {return {lhs.s, lhs.v+rhs};}
inline Quaternion operator+ (arma::vec3 const& lhs, Quaternion const& rhs) {return {rhs.s, lhs+rhs.v};}

inline Quaternion operator* (Quaternion const& lhs, arma::vec3 const& rhs) {return {-arma::dot(lhs.v,rhs), lhs.s*rhs+arma::cross(lhs.v,rhs)};}
inline Quaternion operator* (arma::vec3 const& lhs, Quaternion const& rhs) {return {-arma::dot(lhs,rhs.v), lhs*rhs.s+arma::cross(lhs,rhs.v)};}

inline Quaternion operator* (double const& lhs, Quaternion const& rhs) {return {lhs * rhs.s, lhs * rhs.v};}
inline Quaternion operator* (Quaternion const& lhs, double const& rhs) {return {lhs.s * rhs, lhs.v * rhs};}

#endif //__QUATERNION_H__
