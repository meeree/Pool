#ifndef  __BOOK_KEEPING_H__
#define  __BOOK_KEEPING_H__

#include "../SGV3D/src/base.h"
#include "core.h"

namespace BookKeeping 
{
    glm::vec3 atg3 (arma::vec3 const& v);
    glm::vec4 atg4 (arma::vec4 const& v);
    glm::mat3x3 atg33 (arma::mat33 const& v);
    glm::mat4x4 atg44 (arma::mat44 const& v);
    arma::vec3 gta3 (glm::vec3 const& v);
    arma::vec4 gta4 (glm::vec4 const& v);
    arma::mat33 gta33 (glm::mat3x3 const& v);
    arma::mat44 gta44 (glm::mat4x4 const& v);
}

#endif //__BOOK_KEEPING_H__
