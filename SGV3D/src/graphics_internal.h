#ifndef  __GRAPHICS_INTERNAL_H__
#define  __GRAPHICS_INTERNAL_H__

#include "base.h"
#include <GLFW/glfw3.h>

#include <unordered_map>
#include <string>
#include <iostream>

class GLUniformCache 
{
private:
    std::unordered_map<std::string, GLint> m_layout;

public:
    ///\brief Initialize cache using LightGLContext
    GLUniformCache (GLuint const& shader) {CacheUniforms(shader);}

    ///\brief Cache all uniforms in provided shader program 
    void CacheUniforms (GLuint const& shader);

    ///\brief Lookup uniform index 
    GLint Lookup (std::string const& uniform) const;
};

class GLInfo 
{
private:
    struct ProgramUniPair
    {
        StrippedGLProgram program;
        GLUniformCache uniCache; 

        inline bool operator< (StrippedGLProgram const& rhs) {return program < rhs;}
        inline bool operator== (StrippedGLProgram const& rhs) {return program == rhs;}
    };
    std::vector<ProgramUniPair> m_programs;
    ProgramUniPair* m_curProgram;

    GLfloat m_width, m_height;
    std::string m_title;
    GLuint m_colorscheme;
    GLfloat m_scalar;

public:
    GLInfo () : m_curProgram{nullptr}, m_colorscheme{0}, m_scalar{1.0f} {}

    void CacheProgram (StrippedGLProgram const& program);
    bool SetProgram (StrippedGLProgram const& program);
    inline void SetDimension (GLfloat const& width, GLfloat const& height) {m_width = width; m_height = height;}
    inline void SetTitle (std::string const& title) {m_title = title;}                                          

    inline GLfloat Width () const {return m_width;}
    inline GLfloat Height () const {return m_height;}
    inline std::string Title () const {return m_title;}
    inline GLint LookupUniform (std::string const& uniform) const {return m_curProgram->uniCache.Lookup(uniform);}

    inline void ScaleUp (GLfloat const& scaleFactor) {m_scalar *= scaleFactor;}
    inline void ScaleDown (GLfloat const& scaleFactor) {m_scalar /= scaleFactor;}
    inline void SetScalar (GLfloat const& scalar) {m_scalar = scalar;}
    inline void UpdateScalarUniform (GLint const& scalarLoc) const {glUniform1f(scalarLoc, m_scalar);}
};

class GLContext
{
protected:
    GLContext () : m_done{false} {g_GLContextCreated = true;}
    virtual ~GLContext () {}

    virtual void SaveImportantUniforms () = 0;

    bool m_done; 
    GLInfo m_info;

public:
    virtual bool Render (StrippedGLProgram const& program, GLfloat const (&color)[4]) = 0;

    bool GetNewProgram (GLProgram& program, const char* const& vertShader, const char* const& fragShader, uint8_t const& meshMask, bool const& isStatic=false);
    bool BindProgram (GLProgram const& program);
    bool BindProgram (StrippedGLProgram const& program);

    inline GLInfo& Info () {return m_info;}
    inline GLuint LookupUniform (std::string const& uniform) const {return m_info.LookupUniform(uniform);}
    inline void Done () {m_done = true;} 
};

class GLGraphicsManager 
{
private:
    std::vector<GLContext*> m_contexts;

public:
    GLGraphicsManager () {DEBUG_MSG("Constructed GLGraphicsManager");}

    inline void AttachContext (GLContext* context) {m_contexts.push_back(context);}
};

class GLFWContext : public GLContext
{
protected:
    GLFWkeyfun m_keyCallback;
    GLFWmousebuttonfun m_mouseButtonCallback;
    GLFWwindow* m_window;

    GLFWContext ();
    virtual ~GLFWContext () override {glfwDestroyWindow(m_window);}

    bool Initailize (GLint const (&version)[2], 
                     GLfloat const& width=640, GLfloat const& height=480,
                     bool const& initGlew=true, 
                     std::string const& title="Untitled Window",
                     GLFWkeyfun const& keyCallback=nullptr, GLFWmousebuttonfun const& mouseButtonCallback=nullptr);

public:
    void DisableCursor () const;

    GLFWwindow* Window () const {return m_window;}
};

#endif //__GRAPHICS_INTERNAL_H__
