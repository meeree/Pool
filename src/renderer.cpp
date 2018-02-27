#include "renderer.h"
#include "phys.h" 
#include <algorithm>
#include "bookKeeping.h"

#include <glm/gtx/transform.hpp>
#include <glm/gtc/type_ptr.hpp>

bool Renderer::Initailize (GLfloat const& width, GLfloat const& height,
                           bool const& initGlew, 
                           GLFWkeyfun const& keyCallback, GLFWmousebuttonfun const& mouseButtonCallback)
{
    bool good{GLFWContext::Initailize(
            {4, 5},                              //OpenGL version
            width, height,                       //Window dimension 
            initGlew,                            //Initialize GLEW? 
            "Physics Fun",                       //Window title
            keyCallback, mouseButtonCallback     //Callbacks 
    )};

    DisableCursor();
    return good;
}

void Renderer::SetModelUniforms (std::vector<Entity*> const& ents) 
{
    ASSERT(m_meshes.size() == m_modelMats.size());
    std::transform(ents.begin(), ents.end(), m_modelMats.begin(), 
            [](Entity* const& ent)
            {return glm::translate(BookKeeping::atg3(ent->GetRigidBody()->Position())) 
                   * glm::mat4x4(BookKeeping::atg33(ent->GetRigidBody()->Orientation()));});
}

#define WAS_PRESSED(key) (glfwGetKey(m_window, key) == GLFW_PRESS)

bool Renderer::Render (StrippedGLProgram const& program, GLfloat const (&color)[4])
{
    glBindVertexArray(program.Vao());
    glfwPollEvents();
    if(m_done)
        return false;

    double t{glfwGetTime()};

    glClearBufferfv(GL_COLOR, 0.0f, color); 
    glClear(GL_DEPTH_BUFFER_BIT);

    //Get camera position
    double xpos, ypos;
    glfwGetCursorPos(m_window, &xpos, &ypos);

    uint8_t keyMask{0};
    keyMask |= 1  * WAS_PRESSED(GLFW_KEY_Q);
    keyMask |= 2  * WAS_PRESSED(GLFW_KEY_W);
    keyMask |= 4  * WAS_PRESSED(GLFW_KEY_E);
    keyMask |= 8  * WAS_PRESSED(GLFW_KEY_A);
    keyMask |= 16 * WAS_PRESSED(GLFW_KEY_S);
    keyMask |= 32 * WAS_PRESSED(GLFW_KEY_D);

    bool slow{WAS_PRESSED(GLFW_KEY_LEFT_SHIFT)};

    m_camera.Update({xpos, ypos}, keyMask, glfwGetTime(), glfwGetTime() - t, slow);
    m_camera.UpdateUniforms(6, -1, 3, 4);

    for(unsigned i = 0; i < m_meshes.size(); ++i)
    {
        GraphMesh& gMesh{m_meshes[i]};

        glUniformMatrix4fv(m_modelLoc, 1, GL_FALSE, glm::value_ptr(m_modelMats[i]));
        Indexer indexer{gMesh.GetSigIndexer()};
        if (gMesh.UsesIndices()) //Handle errors with glGetError here??
            glDrawElements(gMesh.GetPrimType(), indexer.Count(), 
                    GL_UNSIGNED_INT, (void*)(size_t)indexer.First());
        else 
            glDrawArrays(gMesh.GetPrimType(), indexer.First(), indexer.Count());
    }

    glfwSwapBuffers(m_window);

    return true;
}

#undef WAS_PRESSED

void Renderer::SaveImportantUniforms () 
{
    m_modelLoc = m_info.LookupUniform("model");
    DEBUG_MSG("Stored model matrix shader location as %i", m_modelLoc);
}

void Renderer::AddEntity (GLProgram& program, Entity* const& ent)
{
    m_meshes.push_back(program.AddMesh(*ent->GetMesh(), GL_TRIANGLES));
    m_modelMats.push_back(glm::mat4x4());
}

void Renderer::AddEntities (GLProgram& program, std::vector<Entity*> const& ents)
{
	size_t curSz{m_meshes.size()};
	m_meshes.resize(m_meshes.size() + ents.size());
	std::transform(ents.begin(), ents.end(), m_meshes.begin()+curSz, 
			[&](Entity* const& ent){return program.AddMesh(*ent->GetMesh(), GL_TRIANGLES) ;});
    m_modelMats.resize(m_modelMats.size()+ents.size());
}
