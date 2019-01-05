#include "camera.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/ext.hpp>

void BasicCamera::UpdateUniforms (GLint const& posIdx, GLint const& dirIdx, GLint const& projIdx, GLint const& viewIdx)
{
    if(posIdx > 0)
        glUniform3fv(posIdx, 1, glm::value_ptr(m_pos));
    if(dirIdx > 0)
        glUniform3fv(dirIdx, 1, glm::value_ptr(m_dir));
    if(projIdx > 0)
        glUniformMatrix4fv(projIdx, 1, GL_FALSE, glm::value_ptr(m_projection));
    if(viewIdx > 0)
        glUniformMatrix4fv(viewIdx, 1, GL_FALSE, glm::value_ptr(m_view));
}

void FreeRoamCamera::SetAngles (glm::vec2 const& theta)
{
    //theta.x is rotation around y axis starting at vector (1,0,0)
    //theta.y is rotation around z axis starting at vector (1,0,0)
    
    m_theta = theta;
    m_dir = {
        cos(m_theta.x) * cos(m_theta.y),
                         sin(m_theta.y),
        sin(m_theta.x) * cos(m_theta.y)
    };
}

//void FreeRoamCamera::SetDirection (glm::vec3 const& dir)
//{
//    glm::vec3 dirNm{glm::normalize(dir)};
//	m_theta = {acos(dot(glm::vec3(1.0f,0.0f,0.0f), dirNm)),
//               acos(dot(glm::vec3(0.0f,1.0f,0.0f), dirNm))};
//    UpdateView();
//}

FreeRoamCamera::FreeRoamCamera (float const& lookSpeed, float const& moveSpeed) 
    : m_right{0.0f}, m_up{0.0f}, m_theta{0.0f, M_PI}, m_mouse{0.0f, 0.0f}, m_keyMask{0}, BasicCamera(lookSpeed, moveSpeed)
{
    UpdateView();
}

void FreeRoamCamera::UpdateView (bool updateMatrix) 
{
    m_dir = {
        cos(m_theta.x) * cos(m_theta.y),
                         sin(m_theta.y),
        sin(m_theta.x) * cos(m_theta.y)
    };
    m_right = glm::vec3{
        cos(m_theta.x - M_PI_2),
        0.0f,
        sin(m_theta.x - M_PI_2)
    };
    m_up = glm::cross(m_right, m_dir);

    if(updateMatrix)
        m_view = glm::lookAt(m_pos, m_pos + m_dir, m_up);
}

void FreeRoamCamera::Update (glm::vec2 const& mouse, float const& t, float const& dt, bool const& slow) 
{
    GLfloat slowSclr{slow ? 0.5f : 1.0f};
    m_theta += m_lookSpeed * dt * (mouse - m_mouse);
    m_mouse = mouse;
    
    UpdateView(false); //don't update matrix yet

    glm::vec3 moveVec(0.0f);
    if(m_keyMask & 1)              //'q'/pg_up   key 
        moveVec -= m_up;
    if(m_keyMask & 2)              //'w'/up      key 
        moveVec += m_dir;
    if(m_keyMask & 4)              //'e'/pg_down key 
        moveVec += m_up;       
    if(m_keyMask & 8)              //'a'/left    key 
        moveVec -= m_right;       
    if(m_keyMask & 16)             //'s'/down    key 
        moveVec -= m_dir;       
    if(m_keyMask & 32)             //'d'/right   key 
        moveVec += m_right;       

    Move(slowSclr * moveVec);
    m_view = glm::lookAt(m_pos, m_pos + m_dir, m_up);
}
