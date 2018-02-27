#include "phys.h"
#include <unordered_map>

//TODO: FIX BISECTION
//NO DAMPING !!!!

#define BIAS 0.0f

Engine::Engine (std::vector<Entity*> const& ents)
    : m_ents{ents}, m_t{0.0}, m_grav{-9.81f}, m_solver{new ConstraintSolver}
{}

Engine::Engine (std::vector<Entity*>&& ents) 
    : m_ents{ents}, m_t{0.0}, m_grav{-9.81f}, m_solver{new ConstraintSolver}
{}
//
//void Engine::UpdatePrimary (float const& dt, std::vector<std::pair<arma::vec3, arma::vec3>>& primary) 
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
//        if(rb->InvMass() > FLT_EPSILON)
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
////        float rad{static_cast<SphereCollidable*>(rb->GetCollidable())->Radius()};
////        for(unsigned j = i+1; j < m_ents.size(); ++j)
////        {
////            RigidBody* rb2{m_ents[j]->GetRigidBody()};
////            float rad2{static_cast<SphereCollidable*>(rb2->GetCollidable())->Radius()};
////            arma::vec3 diff{rb->Position() - rb2->Position()};
////            float totalDist{arma::length(diff)};
////
////            if(totalDist > (rad+rad2)) //Are the objects not in contact?
////                continue;
////
////            if(totalDist < rad+rad2-FLT_EPSILON) //Is inner-penetration occuring?
////                if(!finalTry)
////                    return false;
////
////            float dist{totalDist - (rad+rad2)};
////            glm::vec3 normOnSphere2{1.0f, 0.0f, 0.0f};
////            if(totalDist > 0.0f)
////                normOnSphere2 = diff / totalDist;
////
////            glm::vec3 contactPt{(rb->Position() + rb2->Position() + (rad2 - rad) * normOnSphere2)/2.0f};
////            float sepVel{glm::dot(normOnSphere2, rb->Velocity() - rb2->Velocity())};
////
////            if(sepVel > 0.0f) //Are objects moving apart from eachother?
////                continue;
////
////            float newSepVel{-sepVel * rb->Restitution() * rb2->Restitution()};
////            float deltaVel{newSepVel - sepVel};
////            float totalInvMass{rb->InvMass() + rb2->InvMass()};
////
////            if(totalInvMass <= FLT_EPSILON) //Are both objects fixed?
////                continue;
////
////			float impulse{deltaVel / totalInvMass};
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

bool Engine::CollisionDetection ()
{
    for(unsigned i = 0; i < m_ents.size(); ++i)
    {
        RigidBody* rb1{m_ents[i]->GetRigidBody()};
        float rad1{static_cast<SphereCollidable*>(rb1->GetCollidable())->Radius()};

        for(unsigned j = i+1; j < m_ents.size(); ++j)
        {
            RigidBody* rb2{m_ents[j]->GetRigidBody()};
            float rad2{static_cast<SphereCollidable*>(rb2->GetCollidable())->Radius()};

            arma::fvec3 diff{rb1->Position() - rb2->Position()};
            float totalDist{arma::norm(diff)};
            float dist{totalDist - (rad1 + rad2)}; //Gap between the two objects 

            if(dist > FLT_EPSILON) //Are the objects not in contact?
                continue;

            arma::fvec3 normOnSphere1{-1.0f, 0.0f, 0.0f};
            if(totalDist > 0.0f)
                normOnSphere1 = -diff / totalDist;

            arma::fvec3 contactPt{(rb1->Position() + rb2->Position() + (rad2 - rad1) * normOnSphere1)/2.0f};

            PairWiseConstraint* constraint{new ContactConstraint{i, j, contactPt, normOnSphere1}};
            m_solver->AddConstraint(constraint, 0.0f);

//            if(dist < maxPenetration)
//                return false;
        }
    }

    return true;
}

#define IT_MAX 20 

void Engine::Run (float& dt)
{
    for(Entity* ent: m_ents)
    {
        RigidBody* rb{ent->GetRigidBody()};
        if(rb->InvMass() <= FLT_EPSILON)
            rb->SetForce({0.0f, 0.0f, 0.0f}); 
        else
            rb->SetForce({0.0f, -ent->GetRigidBody()->Mass() * m_grav, 0.0f}); 

        rb->SetTorque({0.0f,0.0f,0.0f}); 
    }

    for(IForce* force: m_forces)
    {
        force->Apply(m_t);
    }

    CollisionDetection();

    if(m_solver->ConstraintCount() > 0)
    {
        unsigned n{(unsigned)m_ents.size()};
        arma::fvec V(6*n), Fext(6*n);
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

        std::chrono::time_point<std::chrono::system_clock> start;
        std::chrono::time_point<std::chrono::system_clock> end; 

        start = std::chrono::system_clock::now();

        m_solver->SolveConstraints(m_ents, V, Fext, dt, IT_MAX);

        end = std::chrono::system_clock::now();
        dt = std::chrono::duration<float>(end-start).count();

        std::cout<<dt<<std::endl;

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

//void Engine::SolveConstraints (float& dt)
//{
//    if(m_ents.empty())
//        return;
//
//    std::vector<Contact> contacts;
//    static float const k_maxPenetration{-1e-3};
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
//        float sepVel{glm::dot(contact.normalOn2, rb1->Velocity() - rb2->Velocity())};
//
//        std::cout<<"SEP VEL:"<<sepVel<<std::endl;
//
//
//        if(sepVel > FLT_EPSILON) //Are objects moving apart from eachother?
//            continue;
//
//        if(sepVel >= -FLT_EPSILON) //Resting contact
//        {
//            restingContacts.push_back(std::move(contact));
//            continue;
//        }
//
//        //Actual collision
//        float newSepVel{-sepVel * rb1->Restitution() * rb2->Restitution()};
//        float deltaVel{newSepVel - sepVel};
//        float totalInvMass{rb1->InvMass() + rb2->InvMass()};
//
//        if(totalInvMass <= FLT_EPSILON) //Are both objects fixed?
//            continue;
//
//        float impulse{deltaVel / totalInvMass};
//        glm::vec3 impulsePerIMass{impulse * contact.normalOn2};
//
//        if(rb1->InvMass() > FLT_EPSILON)
//            impulses[rb1].push_back({impulsePerIMass, contact.contactPt - rb1->Position()});
//
//        if(rb2->InvMass() > FLT_EPSILON)
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

void RigidBody::Update (float const& dt)
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
