#include "phys.h"
#include <unordered_map>
#include <time.h>
#include "collision_detection.h"
#include "collidable.h"

//NO DAMPING !!!!

#define BIAS 0.0f

RigidBody::RigidBody (double const& mass, arma::mat33 const& inertia, arma::vec3 const& pos, Collidable* col, arma::vec3 const& vel, double const& damping, double const& restitution) 
    : m_invMass{1.0f/mass}, 
      m_invIntertiaBody{inertia.i()}, m_invIntertia{m_invIntertiaBody}, 
      m_orientation{arma::eye<arma::mat>(3,3)}, m_pos{pos}, m_quatOrientation{1.0f, 0.0f, 0.0f, 0.0f},
      m_vel{vel}, m_angVel{0.0f}, 
      m_force{0.0f}, m_torque{0.0f},  
      m_col{col},
      m_damping{damping}, m_restitution{restitution} 
{}

Engine::Engine (std::vector<Entity*> const& ents, unsigned const& solverIterations)
    : m_ents{ents}, m_t{0.0}, m_grav{0.0f}, m_solver{new ConstraintSolver(solverIterations)}
{}

Engine::Engine (std::vector<Entity*>&& ents, unsigned const& solverIterations) 
    : m_ents{ents}, m_t{0.0}, m_grav{0.0f}, m_solver{new ConstraintSolver(solverIterations)}
{}

std::vector<PairWiseConstraint*> SphereSphereCollision (unsigned const& rb_idx1, unsigned const& rb_idx2, RigidBody* rb1, RigidBody* rb2)
{
    double rad1{static_cast<SphereCollidable*>(rb1->GetCollidable())->Radius()};
    double rad2{static_cast<SphereCollidable*>(rb2->GetCollidable())->Radius()};

    arma::vec3 diff{rb1->Position() - rb2->Position()};
    double totalDist{arma::norm(diff)};
    double dist{totalDist - (rad1 + rad2)}; //Gap between the two objects 

    if(dist > DBL_EPSILON) //Are the objects not in contact?
        return {};

    arma::vec3 normOnSphere1{-1.0f, 0.0f, 0.0f};
    if(totalDist > 0.0f)
        normOnSphere1 = -diff / totalDist;

    arma::vec3 contactPt{(rb1->Position() + rb2->Position() + (rad2 - rad1) * normOnSphere1)/2.0f};

    PairWiseConstraint* constraint{new ContactConstraint{rb_idx1, rb_idx2, contactPt, normOnSphere1}};
    return {constraint};
}

std::vector<PairWiseConstraint*> SphereMeshCollision (unsigned const& rb_idx1, unsigned const& rb_idx2, RigidBody* rb1, RigidBody* rb2)
{
    (void)rb_idx1; 
    (void)rb_idx2;
    (void)rb1;
    (void)rb2;
    return {};
}

arma::vec3 const* GetSupport (MeshCollidable const& mesh, arma::vec3 const& dir, unsigned& vert_idx)
{
    arma::vec3 const* rex{nullptr};
    double max_proj{-DBL_MAX};

    unsigned idx;
    for(arma::vec3 const& v: mesh.TransformedVertexPositions())
    {
        double proj{arma::dot(v, dir)};

        if(proj > max_proj)
        {
            max_proj = proj;
            rex = &v;
            vert_idx = idx;
        }

        ++idx;
    }

    return rex;
}

double QueryFaceDirections (MeshCollidable const& mesh1, MeshCollidable const& mesh2, HalfEdgeMesh::Face const*& ref_face, arma::vec3& ref_normal, unsigned& ref_vert_idx)
{
    double max_dist{-DBL_MAX};

    for(size_t i = 0; i < mesh1.Faces().size(); ++i)
    {
        HalfEdgeMesh::Face const& face{mesh1.Faces()[i]};
        arma::vec3 const& normal{mesh1.TransformedNormals()[i]};

        HalfEdgeMesh::HalfEdge* const& edge{face.edge};
        if(!edge)
            throw std::logic_error("Edge is null.");

        HalfEdgeMesh::Vertex* const face_vert{edge->end_vert};
        if(!face_vert)
            throw std::logic_error("Face_vert is null.");

        arma::vec3 const& face_vert_pos{mesh1.TransformedVertexPositions()[face_vert->idx]};

        unsigned vert_idx;
        arma::vec3 const* vert{GetSupport(mesh2, -normal, vert_idx)};
        
        //Distance from face plane to vertex.
        double dist{arma::dot(normal, *vert - face_vert_pos)};

        if(dist > max_dist)
        {
            max_dist = dist;
            ref_face = &face;
            ref_vert_idx = vert_idx;
            ref_normal = normal;

        }
    } 

    return max_dist;
}

std::pair<double, double> Project (MeshCollidable const& mesh, arma::vec3 const& dir)
{
    std::pair<double, double> min_max_pair;
    min_max_pair.first = DBL_MAX;
    min_max_pair.second = -DBL_MAX;

    for(arma::vec3 const& vert: mesh.TransformedVertexPositions())
    {
        double proj{arma::dot(dir, vert)};
        if(proj < min_max_pair.first)
            min_max_pair.first = proj;
        if(proj > min_max_pair.second)
            min_max_pair.second = proj;
    }

    return min_max_pair;
}

double QueryEdgeDirections (MeshCollidable const& mesh1, MeshCollidable const& mesh2, HalfEdgeMesh::HalfEdge const*& ref_edge1, HalfEdgeMesh::HalfEdge const*& ref_edge2, arma::vec3& ref_normal)
{
    double max_dist{-DBL_MAX};

    //TODO: DONT REPEAT OPPOSITE EDGES!
    for(size_t i = 0; i < mesh1.HalfEdges().size(); ++i)
    {
        for(size_t j = 0; j < mesh2.HalfEdges().size(); ++j)
        {
            HalfEdgeMesh::HalfEdge const& half_edge1{mesh1.HalfEdges()[i]};
            HalfEdgeMesh::HalfEdge const& half_edge2{mesh1.HalfEdges()[i]};

            if(!half_edge1.opposite_edge || !half_edge2.opposite_edge)
                throw std::logic_error("An edge is null.");

            if(!half_edge1.opposite_edge->end_vert || !half_edge2.opposite_edge->end_vert)
                throw std::logic_error("A vert is null.");

            if(!half_edge1.end_vert || !half_edge2.end_vert)
                throw std::logic_error("A vert is null.");

            arma::vec3 const& a1{mesh1.TransformedVertexPositions()[half_edge1.opposite_edge->end_vert->idx]};
            arma::vec3 const& b1{mesh1.TransformedVertexPositions()[half_edge1.end_vert->idx]};
            arma::vec3 const& a2{mesh2.TransformedVertexPositions()[half_edge2.opposite_edge->end_vert->idx]};
            arma::vec3 const& b2{mesh2.TransformedVertexPositions()[half_edge2.end_vert->idx]};

            arma::vec3 normal{arma::cross(b1 - a1, b2 - a2)}; 

            const static double eps = 1000.0 * std::numeric_limits<double>::epsilon();

            double const normal_norm_sqrd{arma::dot(normal, normal)};
            double const e1_norm_sqrd{arma::dot(b1 - a1, b1 - a1)};
            double const e2_norm_sqrd{arma::dot(b2 - a2, b2 - a2)};

            //Skip near parallel edges
            if(normal_norm_sqrd < eps * e1_norm_sqrd * e2_norm_sqrd)
                continue;

            //Project polyhedra onto normal
            std::pair<double, double> const intrvl1{Project(mesh1, normal)};
            std::pair<double, double> const intrvl2{Project(mesh2, normal)};

            double dist;
            if(intrvl2.second < intrvl1.second)
                dist = intrvl1.first - intrvl2.second; 
            else 
                dist = intrvl2.first - intrvl1.second;

            if(dist > max_dist)
            {
                max_dist = dist;
                ref_edge1 = &half_edge1;
                ref_edge2 = &half_edge2;
                ref_normal = normal;
            }
        }
    } 

    return max_dist;
}

std::vector<PairWiseConstraint*> GenerateMeshFaceCollisionInfo (HalfEdgeMesh::Face const* ref_face, arma::vec3 const& ref_normal, unsigned const& ref_vert_idx, MeshCollidable* mesh1, MeshCollidable* mesh2, unsigned const& rb_idx1, unsigned const& rb_idx2) 
{
    std::vector<PairWiseConstraint*> rex;

    //ref_face is assumed to belong to mesh1 while ref_vert belongs to mesh2. 
    std::vector<HalfEdgeMesh::Vertex*> const& verts_at_pos{mesh2->GetVerticesAtPos(ref_vert_idx)};

    if(verts_at_pos.size() == 0)
        throw std::logic_error("No incident faces.");

    //TODO: CHECK WE DON'T NEED MOST ANTI-PARALLEL FACE HERE!
    HalfEdgeMesh::HalfEdge* incident_edge{verts_at_pos[0]->edge};
    if(!incident_edge)
        throw std::logic_error("Incident edge is null.");

    HalfEdgeMesh::Face* incident_face{incident_edge->face};
    if(!incident_face)
        throw std::logic_error("Incident face is null.");

    arma::vec3 const& pos_on_ref_face{mesh1->TransformedVertexPositions()[ref_face->edge->end_vert->idx]};
    HalfEdgeMesh::HalfEdge* edge{incident_face->edge};
    bool last_it{false};

    //Iterate through edges
    for(; !last_it; edge = edge->next_edge)
    {
        if(edge->next_edge == incident_face->edge)
            last_it = true;

        if(!edge)
            throw std::logic_error("Edge is null");

        HalfEdgeMesh::Vertex* vert{edge->end_vert};

        if(!vert)
            throw std::logic_error("Vertex is null");

        arma::vec3 const& pos{mesh2->TransformedVertexPositions()[vert->idx]}; 

        //Check vertex is inside object
        //TODO: ALSO CLIP USING SH-CLIPPING
        //TODO: CAN THIS DOT GIVE 0???
        if(dot(pos - pos_on_ref_face, -ref_normal) < 0.0)
            continue;

        rex.push_back(new ContactConstraint{rb_idx1, rb_idx2, pos, ref_normal});
    }

    return rex;
}

//To perform line-line intersection and for notation, see http://paulbourke.net/geometry/pointlineplane/
//Note that in GenerateMeshEdgeCollisionInfo we assume the intersection exists!

double d_mnop(arma::vec3 const& v_m, arma::vec3 const& v_n, arma::vec3 const& v_o, arma::vec3 const& v_p)
{
    return arma::dot(v_m - v_n, v_o - v_p);
}

std::vector<PairWiseConstraint*> GenerateMeshEdgeCollisionInfo (HalfEdgeMesh::HalfEdge const* ref_edge1, HalfEdgeMesh::HalfEdge const* ref_edge2, arma::vec3 const& ref_normal, MeshCollidable const& mesh1, MeshCollidable const& mesh2, unsigned const& rb_idx1, unsigned const& rb_idx2)
{
    arma::vec3 const& P1{mesh1.TransformedVertexPositions()[ref_edge1->opposite_edge->end_vert->idx]};
    arma::vec3 const& P2{mesh1.TransformedVertexPositions()[ref_edge1->end_vert->idx]};
    arma::vec3 const& P3{mesh2.TransformedVertexPositions()[ref_edge2->opposite_edge->end_vert->idx]};
    arma::vec3 const& P4{mesh2.TransformedVertexPositions()[ref_edge2->end_vert->idx]};

    double const d1343{d_mnop(P1, P3, P4, P3)};
    double const d4321{d_mnop(P4, P3, P2, P1)};
    double const d1321{d_mnop(P1, P3, P2, P1)};
    double const d4343{d_mnop(P4, P3, P4, P3)};
    double const d2121{d_mnop(P2, P1, P2, P1)};

    double const mu_a{(d1343 * d4321 - d1321 * d4343) / (d2121 * d4343 - d4321 * d4321)};

    arma::vec3 const intersection{P1 + mu_a * (P2 - P1)};
     
    return {new ContactConstraint{rb_idx1, rb_idx2, intersection, ref_normal}};
}

std::vector<PairWiseConstraint*> MeshMeshCollision (unsigned const& rb_idx1, unsigned const& rb_idx2, RigidBody* rb1, RigidBody* rb2)
{
    //Only checks face collisions atm. TODO: CHECK EDGES ALSO
    MeshCollidable* const coll1{static_cast<MeshCollidable*>(rb1->GetCollidable())}; 
    MeshCollidable* const coll2{static_cast<MeshCollidable*>(rb2->GetCollidable())}; 

    HalfEdgeMesh::Face const* out_face1, *out_face2;
    arma::vec3 out_normal1, out_normal2;
    unsigned out_vert_idx1, out_vert_idx2;

    double query1{QueryFaceDirections(*coll1, *coll2, out_face1, out_normal1, out_vert_idx1)};
    if(query1 > 0.0)
        return {};

    double query2{QueryFaceDirections(*coll2, *coll1, out_face2, out_normal2, out_vert_idx2)};
    if(query2 > 0.0)
        return {};

    HalfEdgeMesh::HalfEdge const* out_edge1, *out_edge2;
    arma::vec3 out_normal3;

    double query3{QueryEdgeDirections(*coll1, *coll2, out_edge1, out_edge2, out_normal3)};
    if(query3 > 0.0)
        return {};

    DEBUG_MSG("Collision between objects %u, %u", rb_idx1, rb_idx2);

    unsigned max_idx{query2 > query1 ? (query3 > query2 ? 2u : 1u) : 0u}; 

    if(max_idx == 0)
        return GenerateMeshFaceCollisionInfo(out_face1, out_normal1, out_vert_idx1, coll1, coll2, rb_idx1, rb_idx2); 
    else if(max_idx == 1)
        return GenerateMeshFaceCollisionInfo(out_face2, out_normal2, out_vert_idx2, coll2, coll1, rb_idx2, rb_idx1); 
    else 
        return GenerateMeshEdgeCollisionInfo(out_edge1, out_edge2, out_normal3, *coll1, *coll2, rb_idx1, rb_idx2);
}

bool Engine::CollisionDetection (BroadPhase& bp)
{
    OverlapCache::PairCache const& overlappingPairs{bp.FindOverlappingBoxes()};
    for(auto const& overlappingPair: overlappingPairs)
    {
        RigidBody* rb1{bp.RetrieveBox(overlappingPair.rbIdx1).rb};
        RigidBody* rb2{bp.RetrieveBox(overlappingPair.rbIdx2).rb};

        Collidable::Type const& t1{rb1->GetCollidable()->GetType()};
        Collidable::Type const& t2{rb2->GetCollidable()->GetType()};

        std::vector<PairWiseConstraint*> constraints;
        if(t1 == Collidable::Type::eSphere && t2 == Collidable::Type::eSphere)
            constraints = SphereSphereCollision(overlappingPair.rbIdx1, overlappingPair.rbIdx2, rb1, rb2);
        else if(t1 == Collidable::Type::eSphere && t2 == Collidable::Type::eMesh)
            constraints = SphereMeshCollision(overlappingPair.rbIdx1, overlappingPair.rbIdx2, rb1, rb2);
        else if(t1 == Collidable::Type::eMesh && t2 == Collidable::Type::eSphere)
            constraints = SphereMeshCollision(overlappingPair.rbIdx2, overlappingPair.rbIdx1, rb2, rb1);
        else if(t1 == Collidable::Type::eMesh && t2 == Collidable::Type::eMesh)
            constraints = MeshMeshCollision(overlappingPair.rbIdx2, overlappingPair.rbIdx1, rb2, rb1);

        for(PairWiseConstraint* const& constraint: constraints)
        {
            if(!constraint)
                throw std::logic_error("Constraint is null.");

            m_solver->AddConstraint(constraint, 0.0f);
        }
    }

    return true;
}

Engine::RunInfo Engine::Run (double& dt, BroadPhase& bp)
{
    RunInfo rex;

    int idx = 0;
    for(Entity* ent: m_ents)
    {
        RigidBody* rb{ent->GetRigidBody()};
        if(rb->InvMass() > DBL_EPSILON)
            rb->ApplyForce({0.0f, -ent->GetRigidBody()->Mass() * m_grav, 0.0f}); 
    }

    CollisionDetection(bp);

    rex.num_constraints = m_solver->ConstraintCount();

    if(m_solver->ConstraintCount() > 0)
    {
        unsigned n{(unsigned)m_ents.size()};
        arma::vec V(6*n), Fext(6*n);
        for(unsigned i = 0; i < n; ++i)
        {
            RigidBody* rb{m_ents[i]->GetRigidBody()};
            V(6*i + 0) = rb->Velocity()(0);
            V(6*i + 1) = rb->Velocity()(1);
            V(6*i + 2) = rb->Velocity()(2);
            V(6*i + 3) = rb->AngularVelocity()(0);
            V(6*i + 4) = rb->AngularVelocity()(1);
            V(6*i + 5) = rb->AngularVelocity()(2);

            Fext(6*i + 0) = rb->Force()(0);
            Fext(6*i + 1) = rb->Force()(1);
            Fext(6*i + 2) = rb->Force()(2);
            Fext(6*i + 3) = rb->Torque()(0);
            Fext(6*i + 4) = rb->Torque()(1);
            Fext(6*i + 5) = rb->Torque()(2);
        }

        m_solver->GeneralizedReflections(m_ents, V, Fext, dt);

        //Update the rigid bodies from V and using semi-implicit Euler's method
        //I should use std::move here : IMPROVE THIS
        for(unsigned i = 0; i < m_ents.size(); ++i)
        {
            RigidBody* rb{m_ents[i]->GetRigidBody()};
            rb->SetVelocity({V(6*i + 0), V(6*i + 1), V(6*i + 2)});
            rb->SetAngularVelocity({V(6*i + 3), V(6*i + 4), V(6*i + 5)});

            rb->Update(dt);
        }
    }
    else
    {

        for(Entity* ent: m_ents)
        {
            RigidBody* rb{ent->GetRigidBody()};
            rb->IncVelocity(dt * rb->InvMass() * rb->Force());
            rb->IncAngularVelocity(dt * rb->InverseInertia() * rb->Torque());

            rb->Update(dt);
        }
    }

    m_t += dt;
    m_solver->ClearConstraints();

    return rex;
}

void RigidBody::Update (double const& dt)
{
    m_pos += dt * m_vel;

    m_quatOrientation += 0.5f * dt * m_angVel * m_quatOrientation;
    m_quatOrientation.Normalize();

    m_orientation = Quaternion::QuatToMat(m_quatOrientation);
    m_invIntertia = m_orientation * m_invIntertiaBody * m_orientation.t();

    m_force.zeros();
    m_torque.zeros();
}

Spring::Spring (double const& k) 
    : m_rb1{nullptr}, m_rb2{nullptr}, m_k{k}, m_length{0.0f}
{
}

Spring::Spring (RigidBody* rb1, RigidBody* rb2, double const& k)
    : m_rb1{rb1}, m_rb2{rb2}, m_k{k}, m_length{arma::norm(m_rb1->Position() - m_rb2->Position())}
{
}


void Spring::Apply () const
{
    if(!m_rb1 || !m_rb2)
        return;

    arma::vec3 f{m_rb1->Position() - m_rb2->Position()};
    double dist{arma::norm(f)};

    f = arma::normalise(f);
    f *= m_k * (m_length - dist); //Hooke's law

    if(m_rb1->InvMass() > DBL_EPSILON)
        m_rb1->ApplyForce(f);

    if(m_rb2->InvMass() > DBL_EPSILON)
        m_rb2->ApplyForce(-f);
}

inline void Spring::ResetDisplacement ()
{
    if(m_rb1 && m_rb2)
        m_length = arma::norm(m_rb1->Position() - m_rb2->Position());
    else
        m_length = 0.0f;
}
