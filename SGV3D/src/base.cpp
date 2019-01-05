#include "base.h"

#include <iostream>

bool g_GLContextCreated{false};

void Mesh::Concatenate (Mesh const& mesh)
{
    positions.insert(positions.end(), mesh.positions.begin(), mesh.positions.end());
    normals.insert(normals.end(), mesh.normals.begin(), mesh.normals.end());
    colors.insert(colors.end(), mesh.colors.begin(), mesh.colors.end());
    indices.insert(indices.end(), mesh.indices.begin(), mesh.indices.end());
}

uint8_t Mesh::GetMeshMask () const
{
    uint8_t mask{0};
    mask |= SGV_POSITION * !positions.empty();
    mask |= SGV_NORMAL   * !normals.empty()  ;
    mask |= SGV_COLOR    * !colors.empty()   ;
    mask |= SGV_INDEX    * !indices.empty()  ;
    return mask;
}

GLuint CompileShader (const char* fname, GLenum const& shaderType)
{
    std::vector<char> buffer;
    std::ifstream in;
    in.open(fname, std::ios::binary);

    if(in.is_open()) 
	{
        in.seekg(0, std::ios::end);
        size_t const &length = in.tellg();

        in.seekg(0, std::ios::beg);

        buffer.resize(length + 1);
        in.read(&buffer[0], length);
        in.close();
        buffer[length] = '\0';
    } 
	else 
	{
        ERROR("Unable to open \"%s\"", fname);
        return (GLuint)-1;
    }

    GLchar const *src = &buffer[0];

    GLuint shader = glCreateShader(shaderType);
    glShaderSource(shader, 1, &src, NULL);
    glCompileShader(shader);
    GLint test;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &test);

    if(!test) 
	{
        char compilationLog[512];
        glGetShaderInfoLog(shader, 512, NULL, compilationLog);
        ERROR("Shader compilation failed with this message: %s", compilationLog);
        return (GLuint)-1;
    }

    return shader;
}

GLProgram::GLProgram (uint8_t meshMask, const char* const& vertShader, const char* const& fragShader, bool const& isStatic)
    : m_meshMask{meshMask}, m_static{isStatic}
{
    std::fill(m_buffers, m_buffers+4, UINT_ERR);
    if(!g_GLContextCreated)
    {
        ERROR("Attempt to create GL Program when no context is created!");
        return;
    }

    m_shader = glCreateProgram();
    GLuint compiledVertShader(CompileShader(vertShader, GL_VERTEX_SHADER));
    GLuint compiledFragShader(CompileShader(fragShader, GL_FRAGMENT_SHADER));

    if(compiledVertShader == UINT_ERR || compiledFragShader == UINT_ERR)
    {
        ERROR("Failed to compile shaders \"%s\", \"%s\"", vertShader, fragShader);
        m_shader = (GLuint)-1;
        return; 
    }

    glAttachShader(m_shader, compiledVertShader);
    glAttachShader(m_shader, compiledFragShader);

    glDeleteShader(compiledVertShader);
    glDeleteShader(compiledFragShader);

    glLinkProgram(m_shader);

    glCreateVertexArrays(1, &m_vao);

    if(meshMask == 0)
    {
        WARNING("Created GLProgram for meshes with no information! I.e., meshes will have no position, normal, etc.");
        return;
    }

    unsigned layout{0};
    if(m_meshMask & SGV_POSITION)
    {
        glCreateBuffers(1, &m_buffers[0]);

        glVertexArrayVertexBuffer(m_vao, layout, m_buffers[0], 0, sizeof(glm::vec3));
        glVertexArrayAttribFormat(m_vao, layout, 3, GL_FLOAT, GL_FALSE, 0);
        glVertexArrayAttribBinding(m_vao, layout, layout);
        glEnableVertexArrayAttrib(m_vao, layout);
        ++layout;
    }
    if(m_meshMask & SGV_NORMAL)
    {
        glCreateBuffers(1, &m_buffers[1]);

        glVertexArrayVertexBuffer(m_vao, layout, m_buffers[1], 0, sizeof(glm::vec3));
        glVertexArrayAttribFormat(m_vao, layout, 3, GL_FLOAT, GL_FALSE, 0);
        glVertexArrayAttribBinding(m_vao, layout, layout);
        glEnableVertexArrayAttrib(m_vao, layout);
        ++layout;
    }
    if(m_meshMask & SGV_COLOR)
    {
        glCreateBuffers(1, &m_buffers[2]);

        glVertexArrayVertexBuffer(m_vao, layout, m_buffers[2], 0, sizeof(glm::vec4));
        glVertexArrayAttribFormat(m_vao, layout, 4, GL_FLOAT, GL_FALSE, 0);
        glVertexArrayAttribBinding(m_vao, layout, layout);
        glEnableVertexArrayAttrib(m_vao, layout);
        ++layout;
    }
    if(m_meshMask & SGV_INDEX)
    {
        glCreateBuffers(1, &m_buffers[3]);
    }
}

GraphMesh GLProgram::AddMesh (Mesh const& mesh, GLenum const& primType)
{
    if(mesh.positions.size() == 0)
    {
        ERROR("Attempt to add mesh to GLProgram with no vertices!");
        return GraphMesh(); //Return invalid GraphMesh
    }

    size_t const& prevSz{m_mesh.positions.size()};
    if(m_static)
    {
        if(prevSz > 0)
        {
            ERROR("Attempt to add mesh more than once to static GLProgram: use a dynamic one instead");
            return GraphMesh();
        }

        m_mesh = mesh;
        if(m_meshMask & SGV_POSITION)
            glNamedBufferStorage(m_buffers[0], sizeof(glm::vec3)*m_mesh.positions.size(), m_mesh.positions.data(), 0);
        if(m_meshMask & SGV_NORMAL)
            glNamedBufferStorage(m_buffers[1], sizeof(glm::vec3)*m_mesh.normals.size(), m_mesh.normals.data(), 0);
        if(m_meshMask & SGV_COLOR)
            glNamedBufferStorage(m_buffers[2], sizeof(glm::vec4)*m_mesh.colors.size(), m_mesh.colors.data(), 0);
        //WHAT ABOUT INDICES???
    }
    else 
    {
        m_mesh.Concatenate(mesh);
        if(m_meshMask & SGV_POSITION)
            glNamedBufferData(m_buffers[0], sizeof(glm::vec3) * m_mesh.positions.size(), m_mesh.positions.data(), GL_DYNAMIC_DRAW);
        if(m_meshMask & SGV_NORMAL)
            glNamedBufferData(m_buffers[1], sizeof(glm::vec3) * m_mesh.normals.size(), m_mesh.normals.data(), GL_DYNAMIC_DRAW);
        if(m_meshMask & SGV_COLOR)
            glNamedBufferData(m_buffers[2], sizeof(glm::vec4) * m_mesh.colors.size(), m_mesh.colors.data(), GL_DYNAMIC_DRAW);
        //WHAT ABOUT INDICES???
    }
    
    return GraphMesh(Indexer(prevSz, m_mesh.positions.size() - prevSz), primType);
}

GraphMesh GLProgram::SetMesh (Mesh const& mesh, GLenum const& primType)
{
    m_mesh.positions.clear();    
    m_mesh.normals.clear();    
    m_mesh.colors.clear();    
    m_mesh.indices.clear();    

    return AddMesh(mesh, primType);
}
