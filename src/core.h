#ifndef  __CORE_H__
#define  __CORE_H__

#define ARMA_USE_SUPERLU 1
#include <armadillo>
#include <cmath>

inline float clamp (float const& f, float const& min, float const& max) 
{
    return f < min ? min : (f > max ? max : f);
}

#endif //__CORE_H__
