#include "core.h"
#include "../SGV3D/src/logger.h"

#include <vector>
#include <float.h>
#include <memory>

class Endpoint;
class RigidBody;

class Collidable 
{
public:
    virtual void CalculateBoundingBox(Endpoint* (&min)[3], Endpoint* (&max)[3], RigidBody* rb) = 0;

    enum class Type : uint16_t
    {
        eSphere=0,
        eMesh,
        eCount
    };
    Type m_type;

    Type const& GetType () const {return m_type;}
    
protected:
    Collidable (Type const& type) : m_type{type} {ASSERT(type <= Type::eCount);}
};

class SphereCollidable : public Collidable
{
private:
    double m_rad;

public:
    SphereCollidable (double const& rad) : m_rad{rad}, Collidable(Type::eSphere) {ASSERT(m_rad >= -FLT_EPSILON);}

    virtual void CalculateBoundingBox(Endpoint* (&min)[3], Endpoint* (&max)[3], RigidBody* rb) final;

    double const& Radius () const {return m_rad;}
    void SetRadius (double const& rad) {m_rad = rad;}
};

#if 0 
class PlaneCollidable : public Collidable
{
private:
    arma::vec3 m_normal;
    arma::vec3 m_origin;

public:
    PlaneCollidable (arma::vec3 const& normal, arma::vec3 const& origin) : m_normal{normal}, m_origin{origin} {}

    virtual void CalculateBoundingBox(Endpoint* (&min)[3], Endpoint* (&max)[3], RigidBody* rb) final;

    arma::vec3 const& Normal () const {return m_normal;}
    void SetNormal (arma::vec3 const& normal) {m_normal = normal;}

    arma::vec3 const& Origin () const {return m_origin;}
    void SetOrigin (arma::vec3 const& origin) {m_origin = origin;}
};
#endif 

class HalfEdgeMesh 
{
public:
    struct HalfEdge;

    struct Vertex 
    {
        unsigned idx;
        HalfEdge* edge;  
    };
    
    struct Face 
    {
        arma::vec3 normal;
        HalfEdge* edge;
    };

    struct HalfEdge 
    {
        Vertex* end_vert;
        Face* face; 
        HalfEdge* next_edge; //Ordered ccw
        HalfEdge* opposite_edge;
    };

private:
    std::vector<arma::vec3> m_positions;
    std::vector<Vertex> m_vertices;
    std::vector<Face> m_faces;
    std::vector<HalfEdge> m_half_edges;

    //This allows us to find all vertices at the same position by lookup up the position index. 
    std::vector<std::vector<Vertex*>> m_vert_lookup; 
    
public:
    HalfEdgeMesh (std::vector<arma::vec3> const& positions, std::vector<Vertex> const& verts, std::vector<Face> const& faces, std::vector<HalfEdge> const& half_edges, std::vector<std::vector<Vertex*>> const& vert_lookup) 
        : m_positions{positions}, m_vertices{verts}, m_faces{faces}, m_half_edges{half_edges}, m_vert_lookup{vert_lookup} {}

    // Construct HalfEdgeMesh from list of faces. Vertices are assumed to be ccw. 
    HalfEdgeMesh (std::vector<arma::vec3> const& positions, std::vector<std::vector<unsigned>> const& faces);

    std::vector<arma::vec3> const& VertexPositions () const {return m_positions;}
    std::vector<Vertex> const& Vertices () const {return m_vertices;}
    std::vector<Face> const& Faces () const {return m_faces;}
    std::vector<HalfEdge> const& HalfEdges () const {return m_half_edges;}
    std::vector<Vertex*> const& GetVerticesAtPos (unsigned const& idx) const {return m_vert_lookup[idx];}
};

//MeshInstance currently stores the transformed positions instead
//of computing them on the fly. TODO: IS THERE SOMETHING SMARTER WE CAN DO HERE?
class MeshInstance 
{
private:
    std::shared_ptr<HalfEdgeMesh> m_mesh;
    std::vector<arma::vec3> m_transformed_positions;
    std::vector<arma::vec3> m_transformed_normals;

public:
    MeshInstance (HalfEdgeMesh* const mesh) : m_mesh(mesh), m_transformed_positions(mesh->VertexPositions().size()), m_transformed_normals(mesh->Faces().size())
    {
        Update({0.0,0.0,0.0}, arma::eye(3,3));
    } 

    void Update (arma::vec3 const& trans, arma::mat33 const& ortho_mat);

    std::shared_ptr<HalfEdgeMesh> const& MeshHandle () const {return m_mesh;}
    std::vector<arma::vec3> const& TransformedVertexPositions () const {return m_transformed_positions;}
    std::vector<arma::vec3> const& TransformedNormals () const {return m_transformed_normals;}
};

class MeshCollidable : public Collidable
{
private:
    std::unique_ptr<MeshInstance> m_mesh;    
public:
    MeshCollidable (std::unique_ptr<MeshInstance>&& mesh) : m_mesh{std::move(mesh)}, Collidable{Type::eMesh} {}

    virtual void CalculateBoundingBox(Endpoint* (&min)[3], Endpoint* (&max)[3], RigidBody* rb) final;

    void UpdateInstance (arma::vec3 const& trans, arma::mat33 const& ortho_mat) {m_mesh->Update(trans, ortho_mat);}

    std::vector<HalfEdgeMesh::Vertex> const& Vertices () const {return m_mesh->MeshHandle()->Vertices();}
    std::vector<HalfEdgeMesh::Face> const& Faces () const {return m_mesh->MeshHandle()->Faces();}
    std::vector<HalfEdgeMesh::HalfEdge> const& HalfEdges () const {return m_mesh->MeshHandle()->HalfEdges();}
    std::vector<HalfEdgeMesh::Vertex*> const& GetVerticesAtPos (unsigned const& idx) const {return m_mesh->MeshHandle()->GetVerticesAtPos(idx);}
    std::vector<arma::vec3> const& TransformedVertexPositions () const {return m_mesh->TransformedVertexPositions();}
    std::vector<arma::vec3> const& TransformedNormals () const {return m_mesh->TransformedNormals();}
};
