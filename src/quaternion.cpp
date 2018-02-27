#include "quaternion.h"

arma::fmat33 Quaternion::QuatToMat (Quaternion const& q)
{
    return {1 - 2*q.v[1]*q.v[1] - 2*q.v[2]*q.v[2], 2*q.v[0]*q.v[1] + 2*q.v[2]*q.s, 2*q.v[0]*q.v[2] - 2*q.v[1]*q.s, 
            2*q.v[0]*q.v[1] - 2*q.v[2]*q.s, 1 - 2*q.v[0]*q.v[0] - 2*q.v[2]*q.v[2], 2*q.v[1]*q.v[2] + 2*q.v[0]*q.s,
            2*q.v[0]*q.v[2] + 2*q.v[1]*q.s, 2*q.v[1]*q.v[2] - 2*q.v[0]*q.s, 1 - 2*q.v[0]*q.v[0] - 2*q.v[1]*q.v[1]};
}
