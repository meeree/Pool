#ifndef  __PHYS_H__
#define  __PHYS_H__

#include "quaternion.h"
#include "constraints.h"
#include "../SGV3D/src/logger.h"

#include <vector>
#include <float.h>
#include <memory>

class Collidable;

class RigidBody
{
private:
    double m_invMass;
    double m_damping, m_restitution;
    arma::vec3 m_pos, m_vel, m_angVel; 
    arma::vec3 m_force, m_torque;

    arma::mat33 m_invIntertiaBody;
    arma::mat33 m_invIntertia;
    arma::mat33 m_orientation;
    Quaternion m_quatOrientation;

    Collidable* m_col;

public:
    RigidBody (double const& mass, arma::mat33 const& inertia, arma::vec3 const& pos, Collidable* col, arma::vec3 const& vel={0.0f,0.0f,0.0f}, double const& damping=1.0f, double const& restitution=1.0f);

    void Update (double const& dt);

    inline void Fix () {m_invMass = 0.0f;}

    inline double const& InvMass () const {return m_invMass;}
    inline void SetInvMass (double const& invMass) {m_invMass = invMass;}

    inline double Mass () const {return m_invMass <= FLT_EPSILON ? HUGE_VALF : 1.0f/m_invMass;}
    inline void SetMass (double const& mass) {m_invMass = 1.0f / mass;}

    inline double const& Damping () const {return m_damping;}
    inline void SetDamping (double const& damping) {m_damping = clamp(damping, 0.0f, 1.0f);} 

    inline double const& Restitution () const {return m_restitution;}
    inline void SetRestitution (double const& restitution) {m_restitution = clamp(restitution, 0.0f, 1.0f);} 

    inline arma::vec3 const& Position () const {return m_pos;}
    inline void SetPosition (arma::vec3 const& pos) {m_pos = pos;}
    inline void IncPosition (arma::vec3 const& pos) {m_pos += pos;}

    inline arma::vec3 const& Velocity () const {return m_vel;}
    inline void SetVelocity (arma::vec3 const& vel) {m_vel = vel;}
    inline void IncVelocity (arma::vec3 const& vel) {m_vel += vel;}

    inline arma::vec3 const& AngularVelocity () const {return m_angVel;}
    inline void SetAngularVelocity (arma::vec3 const& angVel) {m_angVel = angVel;}
    inline void IncAngularVelocity (arma::vec3 const& angVel) {m_angVel += angVel;}

    inline arma::vec3 LinearMomentum () const {return m_vel / m_invMass;}
    inline arma::vec AngularMomentum () const {return m_invIntertia.i() * m_angVel;}

    inline arma::vec3 const& Force () const {return m_force;}
    inline void SetForce (arma::vec3 const& force) {m_force = force;}
    inline void ApplyForce (arma::vec3 const& force) {m_force += force;}
    void ApplyForceAtPoint (arma::vec3 const& force, arma::vec3 const& pt) {m_force += force; m_torque += arma::cross(pt, force);}

    inline arma::vec3 const& Torque () const {return m_torque;}
    inline void SetTorque (arma::vec3 const& torque) {m_torque = torque;}
    inline void ApplyTorque (arma::vec3 const& torque) {m_torque += torque;}

    inline void ApplyImpulse (arma::vec3 const& impulse) {m_vel += impulse * m_invMass;}
    inline void ApplyTorqueImpulse (arma::vec3 const& torqueImpulse) {m_angVel += m_invIntertia * torqueImpulse;}
    void ApplyImpulseAtPoint (arma::vec3 const& impulse, arma::vec3 const& pt) {ApplyImpulse(impulse); ApplyTorqueImpulse(arma::cross(impulse, pt));}

    inline arma::mat33 const& Orientation () const {return m_orientation;}
    inline void SetOrientation (Quaternion const& orientation) {m_quatOrientation = orientation;}
    inline void RotateOrientation (double const& t, arma::vec3 const& u) {m_quatOrientation = Quaternion::UnitRotation(t, u) * m_quatOrientation;}

    inline Quaternion const& QuaternionOrientation () const {return m_quatOrientation;}

    inline arma::mat33 const& InverseInertia () const {return m_invIntertia;}
    inline arma::mat33 Inertia () const {return m_invIntertia.i();}
    inline arma::mat33 const& InitialInverseInertia () const {return m_invIntertiaBody;}
    inline arma::mat33 InitialInertia () const {return m_invIntertiaBody.i();}

    inline Collidable* GetCollidable () const {return m_col;}
    inline void SetCollidable (Collidable* col) {m_col = col;}
};

class Endpoint;

class Mesh;

class Entity
{
private:
    RigidBody* m_rb;
    Mesh* m_mesh;

public:
    Entity (RigidBody* rb, Mesh* mesh) : m_rb{rb}, m_mesh{mesh} {}

    RigidBody* GetRigidBody () {return m_rb;}
    Mesh* GetMesh () {return m_mesh;}
    void SetRigidBody (RigidBody* rb) {m_rb = rb;}
    void SetMesh (Mesh* mesh) {m_mesh = mesh;}
};

class IForce 
{
protected:
    RigidBody* m_rb;

    IForce (RigidBody* rb) : m_rb{rb} {}

public:
    virtual void Apply (double const& t) = 0;

    void SetRigidBody (RigidBody* rb) {m_rb = rb;}
    RigidBody* GetRigidBody () const {return m_rb;}
};

class GravForce : public IForce 
{
private: 
    double m_force;
public:
    GravForce (double const& force, RigidBody* rb=nullptr) : m_force{force}, IForce{rb} {}
    virtual void Apply (double const& t) override {(void)t; m_rb->ApplyForce({0.0f,-m_force,0.0f});}

    double Force () const {return m_force;}
    void SetForce (double const& force) {m_force = force;}
};

class SimpleForce : public IForce 
{
private: 
    arma::vec3 m_force;
public:
    SimpleForce (arma::vec3 const& force, RigidBody* rb=nullptr) : m_force{force}, IForce{rb} {}
    virtual void Apply (double const& t) override {(void)t; m_rb->ApplyForce(m_force);}

    arma::vec3 Force () const {return m_force;}
    void SetForce (arma::vec3 const& force) {m_force = force;}
};

typedef arma::vec3 (*ForceCallback)(double const&, RigidBody*);
class DynamicForce : public IForce
{
private:
    ForceCallback m_callback;
public:
    DynamicForce (ForceCallback callback=nullptr, RigidBody* rb=nullptr) : m_callback{callback}, IForce{rb} {}
    virtual void Apply (double const& t) override {if(m_callback) m_rb->ApplyForce(m_callback(t, m_rb));}

    ForceCallback GetCallback () const {return m_callback;}
    void SetCallback (ForceCallback const& callback) {m_callback = callback;}
};

class BroadPhase;

class Engine 
{
private:
    std::vector<Entity*> m_ents;
    std::vector<IForce*> m_forces;

    double m_t;
    double m_grav;

    struct Contact 
    {
        RigidBody* rb1, * rb2;
        arma::vec3 contactPt;
        arma::vec3 normalOn2;
    };

    std::unique_ptr<ConstraintSolver> m_solver;

public:
    Engine (unsigned const& solverIterations=10) : m_solver(new ConstraintSolver(solverIterations)) {}
    Engine (std::vector<Entity*> const& ents, unsigned const& solverIterations=10);
    Engine (std::vector<Entity*>&& ents, unsigned const& solverIterations=10);

    bool CollisionDetection (BroadPhase& bp);

//    void UpdatePrimary (double const& dt, std::vector<std::pair<arma::vec3,arma::vec3>>& primary);
//    void SolveConstraints (double& dt);
    void Run (double& dt, BroadPhase& bp);

    void AddForceToSystem (IForce* const& force) {m_forces.push_back(force);}
    void AddForcesToSystem (std::vector<IForce*> const& forces) {m_forces.insert(m_forces.end(), forces.begin(), forces.end());}
    void ClearForces () {m_forces.clear();}
    void SetTime (double const& t) {m_t = t;}

    void AddEntity (Entity* const& ent) {m_ents.push_back(ent);}
    void AddEntities (std::vector<Entity*> const& ents) {m_ents.insert(m_ents.end(), ents.begin(), ents.end());}

    std::vector<Entity*> const& GetEntities () const {return m_ents;}
    double const& Time () const {return m_t;}

    inline void SetGravity (double const& grav) {m_grav = grav;}
    inline double const& GetGravity () const {return m_grav;}
};  

class Spring 
{
private:
    RigidBody* m_rb1, * m_rb2;
    double m_k;
    double m_length;

public:
    Spring (double const& k=1.0f);
    Spring (RigidBody* rb1, RigidBody* rb2, double const& k=1.0f);

    void Apply () const;

    inline void SetBody1 (RigidBody* rb1) {m_rb1 = rb1;}
    inline void SetBody2 (RigidBody* rb2) {m_rb2 = rb2;}
    inline RigidBody* GetBody1 () const {return m_rb1;}
    inline RigidBody* GetBody2 () const {return m_rb2;}

    inline void SetHookesConstant (double const& k) {m_k = k;}
    inline double const& HookesConstant () const {return m_k;}

    inline void ResetDisplacement ();
};

#endif //__PHYS_H__
