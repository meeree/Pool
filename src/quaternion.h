#ifndef  __QUATERNION_H__
#define  __QUATERNION_H__

#include "core.h"

struct Quaternion 
{
    float s; 
    arma::fvec3 v;

    Quaternion () = default;
    Quaternion (float const& s_, arma::fvec3 const& v_) : s{s_}, v{v_} {}
    Quaternion (float const& s_, float const& x, float const& y, float const& z) : s{s_}, v{x,y,z} {}
    Quaternion (arma::fvec3 const& v_) : s{0.0f}, v{v_} {}

    FORCE_INLINE static Quaternion UnitRotation (float const& t, arma::fvec3 const& u) {return Quaternion(std::cos(t/2.0f), std::sin(t/2.0f)*u);}
    FORCE_INLINE arma::fvec4 ToVector () const {return {v[0], v[1], v[2], s};}

    FORCE_INLINE float Norm () {return arma::norm(ToVector());}
    FORCE_INLINE void Normalize () {float nm{Norm()}; s /= nm; v /= nm;}

    FORCE_INLINE Quaternion operator+ (Quaternion const& rhs) {return {s+rhs.s, v+rhs.v};}
    inline Quaternion operator* (Quaternion const& rhs) {return {s*rhs.s-arma::dot(v,rhs.v), s*rhs.v+rhs.s*v+arma::cross(v,rhs.v)};}

    FORCE_INLINE Quaternion& operator+= (Quaternion const& rhs) {return (*this = *this + rhs);}
    FORCE_INLINE Quaternion& operator*= (Quaternion const& rhs) {return (*this = *this * rhs);}

    static arma::fmat33 QuatToMat (Quaternion const& q);
};

inline Quaternion operator+ (Quaternion const& lhs, arma::fvec3 const& rhs) {return {lhs.s, lhs.v+rhs};}
inline Quaternion operator+ (arma::fvec3 const& lhs, Quaternion const& rhs) {return {rhs.s, lhs+rhs.v};}

inline Quaternion operator* (Quaternion const& lhs, arma::fvec3 const& rhs) {return {-arma::dot(lhs.v,rhs), lhs.s*rhs+arma::cross(lhs.v,rhs)};}
inline Quaternion operator* (arma::fvec3 const& lhs, Quaternion const& rhs) {return {-arma::dot(lhs,rhs.v), lhs*rhs.s+arma::cross(lhs,rhs.v)};}

inline Quaternion operator* (float const& lhs, Quaternion const& rhs) {return {lhs * rhs.s, lhs * rhs.v};}
inline Quaternion operator* (Quaternion const& lhs, float const& rhs) {return {lhs.s * rhs, lhs.v * rhs};}

#endif //__QUATERNION_H__
