#ifndef __BASE_H__
#define __BASE_H__

#include <GL/glew.h>
#include <GL/gl.h>
#include <glm/glm.hpp>
#include <vector>
#include "logger.h"

#define UINT_ERR (GLuint)-1

#define SGV_POSITION 1
#define SGV_NORMAL   2
#define SGV_COLOR    4
#define SGV_INDEX    8

#define X_VEC glm::vec3(1.0f,0.0f,0.0f)
#define Y_VEC glm::vec3(0.0f,1.0f,0.0f)
#define Z_VEC glm::vec3(0.0f,0.0f,1.0f)

//THIS IS A VERY IMPORTANT GLOBAL VARIABLE: IT TELLS US IF AN OPENGL CONTEXT HAS BEEN CREATED!
extern bool g_GLContextCreated;

struct Mesh
{
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals  ;
    std::vector<glm::vec4> colors   ;
    std::vector<GLuint>    indices  ;

    void Concatenate (Mesh const& mesh);
    uint8_t GetMeshMask () const;
};

class Indexer 
{
private:
    GLuint m_first; 
    GLsizei m_count;
    
public:
    Indexer () : m_first{UINT_ERR}, m_count{-1} {WARNING("Constructed invalid Indexer");}
    Indexer (GLuint const& first, GLsizei const& count) : m_first{first}, m_count{count} {}

    inline GLuint First () const {return m_first;}
    inline GLuint Count () const {return m_count;}
    inline bool Valid () const {return UINT_ERR || m_count == -1;} 

    inline bool operator== (Indexer const& rhs) const {return m_first == rhs.m_first && m_count == rhs.m_count;}
};

class GraphMesh
{
private:
    Indexer m_eboIndexer, m_vboIndexer;
    GLenum m_primType;
    bool m_pureVertexDraw;

public:
    GraphMesh () : m_primType{UINT_ERR} {}
    GraphMesh (Indexer const& vboIndexer, GLenum const& primType=GL_TRIANGLES) : m_vboIndexer{vboIndexer}, m_eboIndexer{UINT_ERR, -1}, m_primType{primType}, m_pureVertexDraw{true} {}
    GraphMesh (Indexer const& vboIndexer, Indexer const& eboIndexer, GLenum const& primType=GL_TRIANGLES) : m_vboIndexer{vboIndexer}, m_eboIndexer{eboIndexer}, m_primType{primType}, m_pureVertexDraw{false} {}

    inline Indexer VboIndexer () const {return m_vboIndexer;} 
    inline Indexer EboIndexer () const {return m_eboIndexer;} 
    inline Indexer GetSigIndexer () const {return m_pureVertexDraw ? m_vboIndexer: m_eboIndexer;}
    inline void SetPrimType (GLenum const& primType) {m_primType = primType;}
    inline GLenum GetPrimType () const {return m_primType;} 
    bool UsesIndices () const {return !m_pureVertexDraw;}

    inline bool operator== (GraphMesh const& rhs) const {return m_primType == rhs.m_primType && m_vboIndexer == rhs.m_vboIndexer && m_eboIndexer == rhs.m_eboIndexer;}
};

//It is only necessary to store vao and shader for rendering
class StrippedGLProgram 
{
protected:
    GLuint m_vao, m_shader;

public:
    StrippedGLProgram () : m_vao{UINT_ERR}, m_shader{UINT_ERR} {}

    //Note ordering by significance below
    inline bool operator == (StrippedGLProgram const& rhs) const {return m_vao == rhs.m_vao && m_shader == rhs.m_shader;}
    inline bool operator <  (StrippedGLProgram const& rhs) const {return m_vao <  rhs.m_vao && m_shader <  rhs.m_shader;} 
    inline bool operator >  (StrippedGLProgram const& rhs) const {return m_vao >  rhs.m_vao && m_shader >  rhs.m_shader;} 
    inline bool operator <= (StrippedGLProgram const& rhs) const {return m_vao <= rhs.m_vao && m_shader <= rhs.m_shader;} 
    inline bool operator >= (StrippedGLProgram const& rhs) const {return m_vao >= rhs.m_vao && m_shader >= rhs.m_shader;} 

    inline GLuint Vao    () const {return m_vao   ;}
    inline GLuint Shader () const {return m_shader;}
};

#define BUFF_CHECK(cnd) 

struct GLProgram : public StrippedGLProgram
{
private:
    Mesh m_mesh;
    uint8_t m_meshMask;
    GLuint m_buffers[4];
    bool m_static;

public:
    GLProgram () : StrippedGLProgram(), m_meshMask{0} {std::fill(m_buffers, m_buffers+4, UINT_ERR);}
    GLProgram (uint8_t meshMask, const char* const& vertShader, const char* const& fragShader, bool const& isStatic=false);

    inline StrippedGLProgram Strip () const {return static_cast<StrippedGLProgram>(*this);}

    inline bool operator == (GLProgram const& rhs) const {return Strip() == rhs.Strip() && m_buffers[0] == rhs.m_buffers[0] && m_buffers[1] == rhs.m_buffers[1] && m_buffers[2] == rhs.m_buffers[2] && m_buffers[3] == rhs.m_buffers[3];}
    inline bool operator <  (GLProgram const& rhs) const {return Strip() <  rhs.Strip() && m_buffers[0] <  rhs.m_buffers[0] && m_buffers[1] <  rhs.m_buffers[1] && m_buffers[2] <  rhs.m_buffers[2] && m_buffers[3] <  rhs.m_buffers[3];}
    inline bool operator >  (GLProgram const& rhs) const {return Strip() >  rhs.Strip() && m_buffers[0] >  rhs.m_buffers[0] && m_buffers[1] >  rhs.m_buffers[1] && m_buffers[2] >  rhs.m_buffers[2] && m_buffers[3] >  rhs.m_buffers[3];}
    inline bool operator <= (GLProgram const& rhs) const {return Strip() <= rhs.Strip() && m_buffers[0] <= rhs.m_buffers[0] && m_buffers[1] <= rhs.m_buffers[1] && m_buffers[2] <= rhs.m_buffers[2] && m_buffers[3] <= rhs.m_buffers[3];}
    inline bool operator >= (GLProgram const& rhs) const {return Strip() >= rhs.Strip() && m_buffers[0] >= rhs.m_buffers[0] && m_buffers[1] >= rhs.m_buffers[1] && m_buffers[2] >= rhs.m_buffers[2] && m_buffers[3] >= rhs.m_buffers[3];}

    inline bool Static () const {return m_static;}
    inline Mesh const& MeshRORef () const {return m_mesh;}
    inline bool UsesIndices () const {return m_buffers[3] != UINT_ERR;}

    GraphMesh AddMesh (Mesh const& mesh, GLenum const& primType=GL_TRIANGLES); 
//    GraphMesh AddMesh (Mesh const& mesh, std::vector<GLuint> const& indices, GLenum const& primType=GL_TRIANGLES); 

    GraphMesh SetMesh (Mesh const& mesh, GLenum const& primType=GL_TRIANGLES); 
//    GraphMesh SetMesh (Mesh const& mesh, std::vector<GLuint> const& indices, GLenum const& primType=GL_TRIANGLES); 
};

#endif //__BASE_H__
