#include "sceneGraph.h"
#include "runtimeOptions.h"

#include <algorithm>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
 
// GET RUNTIME OPTIONS INCORPORATED !!

//int StaticVars::uniLookupSafe (std::string const& key, GLint const& shdr) const
//{
//    GLint loc;
//    try 
//    {
//        loc = uniformMap.at(key);
//        if (loc == -1) {throw std::out_of_range("");} //Throw so that both of these cases are caught.
//    }
//    catch(std::out_of_range) 
//    {
//        if (shdr==-1)
//            ERROR("Lookup of variable \"%s\" in unspecified shader failed", key.c_str());
//        else 
//            ERROR("Lookup of variable \"%s\" in shader %i failed", key.c_str(), shdr);
//    }
//    return loc;
//}

GroupNode::GroupNode (std::vector<Node*> const& children, eGroupType type) 
    : m_children{children}, m_groupType(type), Node(eNodeType::GROUP) {}

GroupNode::GroupNode (std::vector<Node*> const& children) 
    : m_children{children}, m_groupType(eGroupType::GROUP), Node(eNodeType::GROUP) {}

TransformNode::TransformNode (glm::mat4x4 const& mat, std::vector<Node*> const& children)
    : m_mat{mat}, GroupNode(children, eGroupType::TRANSFORM) {}

GeometryNode::GeometryNode (GraphMesh graphMesh)
    : m_graphMesh{graphMesh}, LeafNode(eLeafType::GEOMETRY) {}

void GroupNode::render (RenderContext* rc)
{
    for (auto& child: m_children) 
    {
        child->render(rc);
    }
}

void GeometryNode::render (RenderContext* rc)
{
    glUniformMatrix4fv(rc->globals.modelLoc, 1, GL_FALSE, glm::value_ptr(rc->matStack.top()));
    Indexer indexer{m_graphMesh.GetSigIndexer()};
    if (m_graphMesh.UsesIndices()) //Handle errors with glGetError here??
        glDrawElements(m_graphMesh.GetPrimType(), indexer.Count(), 
                GL_UNSIGNED_INT, (void*)(size_t)indexer.First());
    else 
        glDrawArrays(m_graphMesh.GetPrimType(), indexer.First(), indexer.Count());
}   

void AnimationNode::render (RenderContext* rc)
{
    glm::mat4x4 mat = animate(rc->globals.t);
    rc->matStack.push(rc->matStack.top() * mat);
    GroupNode::render(rc);
    rc->matStack.pop();
}

void TransformNode::render (RenderContext* rc)
{
    rc->matStack.push(rc->matStack.top() * m_mat);
    GroupNode::render(rc);
    rc->matStack.pop();
}

void ContextNode::render (RenderContext* rc)
{
    rc->glContext = m_context;
}
