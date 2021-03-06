#include "bookKeeping.h"

namespace BookKeeping 
{
    glm::vec3 atg3 (arma::vec3 const& v)
    {
        return {v[0], v[1], v[2]};
    }


    glm::vec4 atg4 (arma::vec4 const& v)
    {
        return {v[0], v[1], v[2], v[3]};
    }


    glm::mat3x3 atg33 (arma::mat33 const& v)
    {
        return {v(0,0), v(1,0), v(2,0), 
                v(0,1), v(1,1), v(2,1),
                v(0,2), v(1,2), v(2,2)};
    }


    glm::mat4x4 atg44 (arma::mat44 const& v)
    {
        return {v(0,0), v(1,0), v(2,0), v(3,0),
                v(0,1), v(1,1), v(2,1), v(3,1),
                v(0,2), v(1,2), v(2,2), v(3,2),
                v(0,3), v(1,3), v(2,3), v(3,3)};
    }

    arma::vec3 gta3 (glm::vec3 const& v)
    {
        return {v[0], v[1], v[2]};
    }


    arma::vec4 gta4 (glm::vec4 const& v)
    {
        return {v[0], v[1], v[2], v[3]};
    }


    arma::mat33 gta33 (glm::mat3x3 const& v)
    {
        return {v[0][0], v[1][0], v[2][0], 
                v[0][1], v[1][1], v[2][1],
                v[0][2], v[1][2], v[2][2]};
    }


    arma::mat44 gta44 (glm::mat4x4 const& v)
    {
        return {v[0][0], v[1][0], v[2][0], v[3][0],
                v[0][1], v[1][1], v[2][1], v[3][1],
                v[0][2], v[1][2], v[2][2], v[3][2],
                v[0][3], v[1][3], v[2][3], v[3][3]};
    }
}
