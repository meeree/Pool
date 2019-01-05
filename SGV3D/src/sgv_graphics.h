#ifndef  __SGV_GRAPHICS_H__
#define  __SGV_GRAPHICS_H__

#include "graphics_internal.h"
#include "sceneGraph.h"
#include "camera.h"

//Include all GLM stuff here so user doesn't have to 
#define GLM_FORCE_RADIANS
#include <glm/ext.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#define SGV_MAJOR 1
#define SGV_MINOR 5

class Node;

class SGVGraphics final : public GLFWContext
{
private:
    Node* m_root;
    FreeRoamCamera m_camera;
    bool m_useCamera;

    virtual void SaveImportantUniforms () override;

    //Important shader uniform locations 
    GLint m_modelLoc; 

public:
    SGVGraphics () : m_useCamera{false}, GLFWContext() {DEBUG_MSG("Construct SGVGraphics");}
    bool Initailize (GLfloat const& width=640, GLfloat const& height=480,
                     bool const& initGlew=true, 
                     GLFWkeyfun const& keyCallback=nullptr, GLFWmousebuttonfun const& mouseButtonCallback=nullptr);

    inline void SetCamera (FreeRoamCamera const& camera) {m_useCamera = true; m_camera = camera;}
    inline void SetRoot (Node* root) {m_root = root;}
    virtual bool Render (StrippedGLProgram const& program, GLfloat const (&color)[4]={0.0f,0.0f,0.0f,1.0f}) override;
};


#endif //__SGV_GRAPHICS_H__
