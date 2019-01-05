#include "graphics_internal.h"

#include <glm/gtc/type_ptr.hpp>
#include <algorithm>

#include <iostream>
#include <iomanip>

void GLUniformCache::CacheUniforms (GLuint const& shader)
{
    DEBUG_MSG("Beginning to cache uniforms for current shader program");

    GLint numActiveUniforms = 0;
    glGetProgramInterfaceiv(shader, GL_UNIFORM, GL_ACTIVE_RESOURCES, &numActiveUniforms);

    std::vector<GLchar> nameBuff(256);
    std::vector<GLenum> properties;
    properties.push_back(GL_NAME_LENGTH);
    properties.push_back(GL_TYPE);
    properties.push_back(GL_ARRAY_SIZE);
    std::vector<GLint> values(properties.size());
    for(int i = 0; i < numActiveUniforms; ++i)
    {
      glGetProgramResourceiv(shader, GL_UNIFORM, i, properties.size(),
        &properties[0], values.size(), NULL, &values[0]);
    
      nameBuff.resize(values[0]); //The length of the name.
      glGetProgramResourceName(shader, GL_UNIFORM, i, nameBuff.size(), NULL, nameBuff.data());
      std::string name(nameBuff.data(), nameBuff.size() - 1);

      GLint loc{glGetUniformLocation(shader, name.c_str())};
      m_layout[name] = loc;
      DEBUG_MSG("Cached uniform \"%s\" to location %i", name.c_str(), loc);
    }
}

GLint GLUniformCache::Lookup (std::string const& uniform) const
{
    auto it{m_layout.find(uniform)};
    return it == m_layout.end() ? -1 : it->second;
}

void GLInfo::CacheProgram (StrippedGLProgram const& program) 
{
    GLUniformCache uc(program.Shader()); //Cache uniforms in current context
    m_programs.push_back({program, std::move(uc)});
}

bool GLInfo::SetProgram (StrippedGLProgram const& program)
{
    auto progIdx = std::find(m_programs.begin(), m_programs.end(), program);
    if(progIdx == m_programs.end())
        return false;

    m_curProgram = &(*progIdx);
    return true;
}

bool GLContext::GetNewProgram (GLProgram& program, const char* const& vertShader, const char* const& fragShader, uint8_t const& meshMask, bool const& isStatic)
{
    program = GLProgram(meshMask, vertShader, fragShader, isStatic);
    m_info.CacheProgram(program.Strip());
    m_info.SetProgram(program.Strip());

    return true;
}

bool GLContext::BindProgram (GLProgram const& program)
{
    return BindProgram(program.Strip());
}

bool GLContext::BindProgram (StrippedGLProgram const& program)
{
    if(m_info.SetProgram(program))
    {
        glUseProgram(program.Shader());
        glBindVertexArray(program.Vao()); 
        
        SaveImportantUniforms();

        return true;
    }

    return false;
}

GLFWContext::GLFWContext () 
    : m_keyCallback{nullptr}, m_mouseButtonCallback{nullptr}, m_window{nullptr} 
{
    DEBUG_MSG("Constructed GLFWContext");
}

bool GLFWContext::Initailize (GLint const (&version)[2], 
                 GLfloat const& width, GLfloat const& height, 
                 bool const& initGlew,
                 std::string const& title,
                 GLFWkeyfun const& keyCallback, GLFWmousebuttonfun const& mouseButtonCallback) 
{
    DEBUG_MSG("Initializing GLFWContext \"%s\"; window width %.1f, height %.1f; for OpenGL v.%i.%i", 
              title.c_str(), width, height, version[0], version[1]);

    m_info.SetDimension(width, height);
    m_info.SetTitle(title);

    if(!glfwInit())
    {
        ERROR("Failed to initialize GLFW");
        return false;
    }    
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, version[0]);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, version[1]);
    m_window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if(!m_window)
    {
        ERROR("Failed to initialize GLFW window");
        return false;
    }
    glfwMakeContextCurrent(m_window);

    if(initGlew)
    {
        glewExperimental = GL_TRUE;
        if(glewInit() != GLEW_OK) 
        {
            ERROR("Failed to initialize glew");
            return false;
        }
    }

    if(keyCallback)
        glfwSetKeyCallback(m_window, keyCallback);

    if(mouseButtonCallback)
        glfwSetMouseButtonCallback(m_window, mouseButtonCallback);

    //Mouse and key callbacks can retrieve information from and update GLFWContext
    //through glfw's "user pointer":
    //see http://www.glfw.org/docs/latest/window_guide.html#window_userptr
    glfwSetWindowUserPointer(m_window, this);

    glEnable(GL_BLEND); 
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST); 
    glDepthFunc(GL_LESS);

    DEBUG_MSG("Successfully initialized GLFWContext");

    return true;
}

void GLFWContext::DisableCursor () const
{
    glfwSetInputMode(m_window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glfwSetInputMode(m_window, GLFW_STICKY_KEYS, false);
}
