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
        ASSERT(edge);
        HalfEdgeMesh::Vertex* const face_vert{edge->end_vert};
        arma::vec3 const& face_vert_pos{mesh1.TransformedVertexPositions()[face_vert->idx]};

        unsigned vert_idx;
        arma::vec3 const* vert{GetSupport(mesh2, -normal, vert_idx)};
        ASSERT(vert);
        
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

std::vector<PairWiseConstraint*> GenerateMeshMeshInfo (HalfEdgeMesh::Face const* ref_face, arma::vec3 const& ref_normal, unsigned const& ref_vert_idx, MeshCollidable* mesh1, MeshCollidable* mesh2, unsigned const& rb_idx1, unsigned const& rb_idx2) 
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

    bool is_max1{query1 > query2};

    std::cout<<"Collision between objects "<<rb_idx1<<','<<rb_idx2<<std::endl;

    if(is_max1)
        return GenerateMeshMeshInfo(out_face1, out_normal1, out_vert_idx1, coll1, coll2, rb_idx1, rb_idx2); 
    else 
        return GenerateMeshMeshInfo(out_face2, out_normal2, out_vert_idx2, coll2, coll1, rb_idx2, rb_idx1); 
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

void Engine::Run (double& dt, BroadPhase& bp)
{
    int idx = 0;
    for(Entity* ent: m_ents)
    {
        RigidBody* rb{ent->GetRigidBody()};
        if(rb->InvMass() > DBL_EPSILON)
            rb->ApplyForce({0.0f, -ent->GetRigidBody()->Mass() * m_grav, 0.0f}); 
    }

    std::chrono::time_point<std::chrono::system_clock> start;
    std::chrono::time_point<std::chrono::system_clock> end; 

    start = std::chrono::system_clock::now();

    CollisionDetection(bp);

    end = std::chrono::system_clock::now();
    double timeElapsed = std::chrono::duration<double>(end-start).count();

//    std::cout<<timeElapsed<<std::endl;

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
}

void RigidBody::Update (double const& dt)
{
//    m_vel = m_vel * m_damping + dt * m_force * m_invMass;
    m_pos += dt * m_vel;

//    m_angVel += m_invIntertia * m_torque * dt;
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

//OLD IMPULSE METHOD
//
//void Engine::UpdatePrimary (double const& dt, std::vector<std::pair<arma::vec3, arma::vec3>>& primary) 
//{
////    for(Entity* ent: m_ents)
////    {
////        ent->GetRigidBody()->SetForce(arma::vec3(0.0f));
////        ent->GetRigidBody()->SetTorque(arma::vec3(0.0f));
////    }
////
////    for(IForce* force: m_forces)
////    {
////        force->Apply(m_t);
////    }
//
//    for(unsigned i = 0; i < m_ents.size(); ++i)
//    {
//        RigidBody* rb{m_ents[i]->GetRigidBody()};
//        if(rb->InvMass() > DBL_EPSILON)
//            rb->ApplyForce({0.0f, -rb->Mass() * m_grav, 0.0f});
//
//        primary[i].second = rb->Velocity() * rb->Damping() + dt * rb->Force() * rb->InvMass();
//        primary[i].first = rb->Position() + dt * primary[i].second;
//    }
//
////
////    std::unordered_map<RigidBody*, std::vector<std::pair<arma::vec3, arma::vec3>>> impulses;
////    for(unsigned i = 0; i < m_ents.size(); ++i)
////    {
////        RigidBody* rb{m_ents[i]->GetRigidBody()};
////
////        double rad{static_cast<SphereCollidable*>(rb->GetCollidable())->Radius()};
////        for(unsigned j = i+1; j < m_ents.size(); ++j)
////        {
////            RigidBody* rb2{m_ents[j]->GetRigidBody()};
////            double rad2{static_cast<SphereCollidable*>(rb2->GetCollidable())->Radius()};
////            arma::vec3 diff{rb->Position() - rb2->Position()};
////            double totalDist{arma::length(diff)};
////
////            if(totalDist > (rad+rad2)) //Are the objects not in contact?
////                continue;
////
////            if(totalDist < rad+rad2-DBL_EPSILON) //Is inner-penetration occuring?
////                if(!finalTry)
////                    return false;
////
////            double dist{totalDist - (rad+rad2)};
////            glm::vec3 normOnSphere2{1.0f, 0.0f, 0.0f};
////            if(totalDist > 0.0f)
////                normOnSphere2 = diff / totalDist;
////
////            glm::vec3 contactPt{(rb->Position() + rb2->Position() + (rad2 - rad) * normOnSphere2)/2.0f};
////            double sepVel{glm::dot(normOnSphere2, rb->Velocity() - rb2->Velocity())};
////
////            if(sepVel > 0.0f) //Are objects moving apart from eachother?
////                continue;
////
////            double newSepVel{-sepVel * rb->Restitution() * rb2->Restitution()};
////            double deltaVel{newSepVel - sepVel};
////            double totalInvMass{rb->InvMass() + rb2->InvMass()};
////
////            if(totalInvMass <= DBL_EPSILON) //Are both objects fixed?
////                continue;
////
////			double impulse{deltaVel / totalInvMass};
////            glm::vec3 impulsePerIMass{impulse * normOnSphere2};
////            impulses[rb].push_back({impulsePerIMass, contactPt - rb->Position()});
////            impulses[rb2].push_back({-impulsePerIMass, contactPt - rb2->Position()});
////        }
////    }
////
////    for(auto& impulsePair: impulses)
////    {
////        for(auto& impulse: impulsePair.second)
////        {
////            impulsePair.first->ApplyImpulseAtPoint(impulse.first, impulse.second);
////        }
////    }
////
////    for(unsigned i = 0; i < m_ents.size(); ++i)
////    {
////        RigidBody* rb{m_ents[i]->GetRigidBody()};
////        rb->Update(dt);
////    }
////
////    return true;
//}
//
//OTHER OLD SOLVER
//
//void Engine::SolveConstraints (double& dt)
//{
//    if(m_ents.empty())
//        return;
//
//    std::vector<Contact> contacts;
//    static double const k_maxPenetration{-1e-3};
//
//    std::vector<std::pair<glm::vec3,glm::vec3>> primary(m_ents.size());
//
//    //TODO: FIX BISECTION
//    unsigned i = 0;
//    while(true)
//    {
//        UpdatePrimary(dt, primary);
//        if(CollisionDetection(contacts, primary, k_maxPenetration))
//            break;
//
//        dt /= 2.0f;
//    }
//
//    for(Entity* ent: m_ents)
//    {
//        RigidBody* rb{ent->GetRigidBody()};
//        rb->SetVelocity(rb->Velocity() * rb->Damping() + dt * rb->Force() * rb->InvMass());
//        rb->SetAngularVelocity(rb->AngularVelocity() + rb->InverseInertia() * rb->Torque() * dt);
//    }
//
//    for(Entity* ent: m_ents)
//    {
//        ent->GetRigidBody()->Update(dt); //For position and velocity, we could copy these values from primary 
//    }
//
//    std::vector<Contact> restingContacts;
//
//    std::unordered_map<RigidBody*, std::vector<std::pair<glm::vec3, glm::vec3>>> impulses;
//    for(Contact& contact: contacts)
//    {
//        RigidBody* rb1{contact.rb1}, * rb2{contact.rb2};
//        double sepVel{glm::dot(contact.normalOn2, rb1->Velocity() - rb2->Velocity())};
//
//        std::cout<<"SEP VEL:"<<sepVel<<std::endl;
//
//
//        if(sepVel > DBL_EPSILON) //Are objects moving apart from eachother?
//            continue;
//
//        if(sepVel >= -DBL_EPSILON) //Resting contact
//        {
//            restingContacts.push_back(std::move(contact));
//            continue;
//        }
//
//        //Actual collision
//        double newSepVel{-sepVel * rb1->Restitution() * rb2->Restitution()};
//        double deltaVel{newSepVel - sepVel};
//        double totalInvMass{rb1->InvMass() + rb2->InvMass()};
//
//        if(totalInvMass <= DBL_EPSILON) //Are both objects fixed?
//            continue;
//
//        double impulse{deltaVel / totalInvMass};
//        glm::vec3 impulsePerIMass{impulse * contact.normalOn2};
//
//        if(rb1->InvMass() > DBL_EPSILON)
//            impulses[rb1].push_back({impulsePerIMass, contact.contactPt - rb1->Position()});
//
//        if(rb2->InvMass() > DBL_EPSILON)
//            impulses[rb2].push_back({-impulsePerIMass, contact.contactPt - rb2->Position()});
//    }
//
//    for(auto& impulsePair: impulses)
//    {
//        for(auto& impulse: impulsePair.second)
//        {
//            impulsePair.first->ApplyImpulseAtPoint(impulse.first, impulse.second);
//            impulsePair.first->Update(dt);
//        }
//    }
//
//    size_t n{restingContacts.size()};
//    arma::mat Amat(n, n);
//    arma::vec Bvec(n);
//    arma::vec Fvec(n);
//
////    ComputeMatrixA(Amat);
////    ComputeVectorB(Bvec);
////
////    BaraffSolve(Amat, Bvec, Fvec);
////
////    for(unsigned i = 0; i < n; ++i)
////    {
////        restingContacts[i].rb1->SetForce(Fvec[i] * restingContacts[i].normalOn2);
////    }
//            
//
//    m_t += dt;
//}
