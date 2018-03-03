#ifndef  __RENDERER_H__
#define  __RENDERER_H__

#include <vector>

#include "core.h"
#include "../../SGV3D/src/camera.h"
#include "../../SGV3D/src/graphics_internal.h"

class Entity;
class Engine;

class Renderer final : public GLFWContext
{
private:
    std::vector<GraphMesh> m_meshes;
    std::vector<glm::mat4x4> m_modelMats;
    FreeRoamCamera m_camera;

    Engine* m_engPtr;
    GLProgram* m_curProgPtr;

    virtual void SaveImportantUniforms () override;

    GLint m_modelLoc;

public:
    Renderer () = default;
    bool Initailize (GLfloat const& width=640, GLfloat const& height=480,
                     bool const& initGlew=true, 
                     GLFWkeyfun const& keyCallback=nullptr, GLFWmousebuttonfun const& mouseButtonCallback=nullptr);

    void SetModelUniforms (std::vector<Entity*> const& ents);
    virtual bool Render (StrippedGLProgram const& program, GLfloat const (&color)[4]={0.0f,0.0f,0.0f,1.0f}) override;

    inline void SetProgram (GLProgram* prog) {m_curProgPtr = prog;}
    inline GLProgram* GetCurrentProgram () const {return m_curProgPtr;}

    void SetCamera (FreeRoamCamera const& camera) {m_camera = camera;}
    FreeRoamCamera const& GetCamera () const {return m_camera;}
    FreeRoamCamera& GetCameraRef () {return m_camera;}
    void AddEntity (GLProgram& program, Entity* const& ent);
    void AddEntities (GLProgram& program, std::vector<Entity*> const& ents);

    void SetEnginePtr (Engine& engine) {m_engPtr = &engine;}
    Engine* GetEnginePtr () const {return m_engPtr;}
};

#endif //__RENDERER_H__
