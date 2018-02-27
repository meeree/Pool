#ifndef  __BOOK_KEEPING_H__
#define  __BOOK_KEEPING_H__

#include "../../SGV3D/src/base.h"
#include "core.h"

namespace BookKeeping 
{
    glm::vec3 atg3 (arma::fvec3 const& v);
    glm::vec4 atg4 (arma::fvec4 const& v);
    glm::mat3x3 atg33 (arma::fmat33 const& v);
    glm::mat4x4 atg44 (arma::fmat44 const& v);
    arma::fvec3 gta3 (glm::vec3 const& v);
    arma::fvec4 gta4 (glm::vec4 const& v);
    arma::fmat33 gta33 (glm::mat3x3 const& v);
    arma::fmat44 gta44 (glm::mat4x4 const& v);
}

#endif //__BOOK_KEEPING_H__
