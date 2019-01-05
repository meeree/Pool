#ifndef  __SCENE_GRAPH_H__
#define  __SCENE_GRAPH_H__

#include <stack>
#include <unordered_map>
#include "base.h"

//TODO URGENT: add destructors

/***********************//**
 * StaticVars
 * Variables that remain constant throughout a traversal such as time and uniform locations. 
 **************************/
struct StaticVars 
{
    GLint modelLoc;
    double t;
}; 

/***********************//**
 * RenderContext 
 * Context passed down in render traversals. 
 **************************/
struct RenderContext
{
    StaticVars globals;
    std::stack<glm::mat4x4> matStack;
    StrippedGLProgram glContext;
};

/***********************//**
 * Node
 * Abstract base class for all scene graph nodes. Note that nodes contain no children.
 **************************/
class Node 
{
public:
    enum eNodeType 
    {
        GROUP=0,LEAF=1 //I use these two classes to avoid having to handle both cases in each deriving node.
                       //Note: group nodes can be explicitly created whereas leaf nodes are abstract.
    };

protected:
    Node (eNodeType type) : m_type{type} {}
    eNodeType const m_type;

public:
    ///\brief Abstract render method that passes through the graph, updates transforms, and performs rendering. 
    virtual void render (RenderContext*) = 0;

    inline bool isGroup () const {return m_type == GROUP;}
    inline bool isLeaf () const {return m_type == LEAF;}
};

/***********************//**
 * GroupNode
 * Non-abstract derived node with children. 
 **************************/
class GroupNode : public Node 
{
public:
    enum eGroupType 
    {
        GROUP=0,TRANSFORM=1,CONTEXT=2,ANIMATION=3
    };

protected:
    std::vector<Node*> m_children;
    eGroupType const m_groupType;
    GroupNode (std::vector<Node*> const&, eGroupType); 

public:
    GroupNode (std::vector<Node*> const& children={});
    virtual void render (RenderContext*) override;

    //Getter/setter
    inline void addChild (Node* const& child) {m_children.push_back(child);} 
    inline void addChildren (std::vector<Node*> const& children) {m_children.insert(m_children.end(), children.begin(), children.end());} 
    inline void removeChild (unsigned const& index) {m_children.erase(m_children.begin()+index);}
    inline std::vector<Node*> const& getChildren () const {return m_children;}
    inline Node* const& getChild (unsigned const& index) const {return m_children.at(index);} 

    inline bool isGroupType (eGroupType const& groupType) const {return m_groupType == groupType;}
    inline eGroupType const& getType () const {return m_groupType;}
};

/***********************//**
 * LeafNode
 * Abstract derived node. Nodes are divided into group and leaf nodes because 
 * certain nodes, such as geometry nodes, are required to be leaf nodes by caching
 * and performing leaf checks in each of these types of nodes would be redundent and unclean. 
 **************************/
class LeafNode : public Node 
{
public:
    enum eLeafType 
    {
        GEOMETRY=0 
    };

protected:
    eLeafType const m_leafType;
    LeafNode (eLeafType leafType) : m_leafType{leafType}, Node(eNodeType::LEAF) {}

public:
    virtual void render (RenderContext*) = 0;

    inline bool isLeafType (eLeafType const& leafType) const {return m_leafType == leafType;}
    inline eLeafType const& getType () const {return m_leafType;}
};

/***********************//**
 * TransformNode
 * GroupNode containing matrix transform. 
 **************************/
class TransformNode : public GroupNode
{
protected:
    glm::mat4x4 m_mat;

public:
    TransformNode (glm::mat4x4 const& mat=glm::mat4x4(1.0f), std::vector<Node*> const& children={});
    virtual void render (RenderContext*) override;

    //Getter/setter
    inline void setTransform (glm::mat4x4 const& mat) {m_mat = mat;}
    inline glm::mat4x4 const& getTransform () const {return m_mat;}
};

/***********************//**
 * GeometryNode
 * LeafNode containing a GraphMesh that it uses for rendering.
 **************************/
class GeometryNode : public LeafNode
{
protected:
    GraphMesh m_graphMesh;

public:
    GeometryNode () : LeafNode(eLeafType::GEOMETRY) {}
    GeometryNode (GraphMesh graphMesh);
    virtual void render (RenderContext*) override;

    //Getter/setter
    inline void setGraphMesh (GraphMesh const& graphMesh) {m_graphMesh = graphMesh;}
    inline GraphMesh const& getGraphMesh () const {return m_graphMesh;}
};

/***********************//**
 * AnimationNode
 * This abstract GroupNode calculates a transform at a given point in time using an abstract 
 * function to be overloaded by the user. Further, it also stores the previously calculated
 * matrix so more fluid animations can be performed.
 **************************/
class AnimationNode : public GroupNode
{
protected:
	glm::mat4x4 m_mat;
	virtual glm::mat4x4 animate (double const&) = 0; 

public:
    AnimationNode () : GroupNode({}, eGroupType::ANIMATION) {}
    virtual void render (RenderContext* rc) override final;

    //Getter/setter
	inline glm::mat4x4 getTransform () const {return m_mat;}
};

/***********************//**
 * ContextNode
 * Holds a StrippedGLProgram object which it pushes onto the context stack during rendering.
 **************************/
class ContextNode : public GroupNode 
{
protected:
    StrippedGLProgram m_context;

public:
    ContextNode () : GroupNode({}, eGroupType::CONTEXT) {}
    virtual void render (RenderContext* rc) override final;

    //Getter/setter
	inline StrippedGLProgram getContext () const {return m_context;}
};

#endif //__SCENE_GRAPH_H__
