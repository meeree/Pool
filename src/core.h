#ifndef  __CORE_H__
#define  __CORE_H__

#include <armadillo>
#include <cmath>

#if defined(_MSC_VER)
#define FORCE_INLINE __forceinline 
#elif defined(__GNUC__)
#define FORCE_INLINE __attribute__((always_inline))
#else 
#define FORCE_INLINE inline
#endif 

inline float clamp (float const& f, float const& min, float const& max) 
{
    return f < min ? min : (f > max ? max : f);
}

#endif //__CORE_H__
