#ifndef  __PHYS_H__
#define  __PHYS_H__

#include "quaternion.h"
#include "constraints.h"

#include <vector>
#include <float.h>
#include <memory>

class Collidable;

class RigidBody
{
private:
    float m_invMass;
    float m_damping, m_restitution;
    arma::fvec3 m_pos, m_vel, m_angVel; 
    arma::fvec3 m_force, m_torque;

    arma::fmat33 m_invIntertiaBody;
    arma::fmat33 m_invIntertia;
    arma::fmat33 m_orientation;
    Quaternion m_quatOrientation;

    Collidable* m_col;

public:
    RigidBody (float const& mass, arma::fmat33 const& inertia, arma::fvec3 const& pos, Collidable* col, arma::fvec3 const& vel={0.0f,0.0f,0.0f}, float const& damping=1.0f, float const& restitution=1.0f) 
        : m_invMass{1.0f/mass}, m_invIntertiaBody{inertia.i()}, m_invIntertia{m_invIntertiaBody}, m_orientation{1.0f}, m_damping{damping}, m_restitution{restitution}, m_pos{pos}, m_vel{vel}, m_angVel{0.0f}, m_force{0.0f}, m_torque{0.0f}, m_col{col}, m_quatOrientation{1.0f, 0.0f, 0.0f, 0.0f} {}

    void Update (float const& dt);

    FORCE_INLINE void Fix () {m_invMass = 0.0f;}

    FORCE_INLINE float const& InvMass () const {return m_invMass;}
    FORCE_INLINE void SetInvMass (float const& invMass) {m_invMass = invMass;}

    FORCE_INLINE float Mass () const {return m_invMass <= FLT_EPSILON ? HUGE_VALF : 1.0f/m_invMass;}
    FORCE_INLINE void SetMass (float const& mass) {m_invMass = 1.0f / mass;}

    FORCE_INLINE float const& Damping () const {return m_damping;}
    FORCE_INLINE void SetDamping (float const& damping) {m_damping = clamp(damping, 0.0f, 1.0f);} 

    FORCE_INLINE float const& Restitution () const {return m_restitution;}
    FORCE_INLINE void SetRestitution (float const& restitution) {m_restitution = clamp(restitution, 0.0f, 1.0f);} 

    FORCE_INLINE arma::fvec3 const& Position () const {return m_pos;}
    FORCE_INLINE void SetPosition (arma::fvec3 const& pos) {m_pos = pos;}
    FORCE_INLINE void IncPosition (arma::fvec3 const& pos) {m_pos += pos;}

    FORCE_INLINE arma::fvec3 const& Velocity () const {return m_vel;}
    FORCE_INLINE void SetVelocity (arma::fvec3 const& vel) {m_vel = vel;}
    FORCE_INLINE void IncVelocity (arma::fvec3 const& vel) {m_vel += vel;}

    FORCE_INLINE arma::fvec3 const& AngularVelocity () const {return m_angVel;}
    FORCE_INLINE void SetAngularVelocity (arma::fvec3 const& angVel) {m_angVel = angVel;}
    FORCE_INLINE void IncAngularVelocity (arma::fvec3 const& angVel) {m_angVel += angVel;}

    FORCE_INLINE arma::fvec3 LinearMomentum () const {return m_vel / m_invMass;}
    FORCE_INLINE arma::fvec AngularMomentum () const {return m_invIntertia.i() * m_angVel;}

    FORCE_INLINE arma::fvec3 const& Force () const {return m_force;}
    FORCE_INLINE void SetForce (arma::fvec3 const& force) {m_force = force;}
    FORCE_INLINE void ApplyForce (arma::fvec3 const& force) {m_force += force;}
    void ApplyForceAtPoint (arma::fvec3 const& force, arma::fvec3 const& pt) {m_force += force; m_torque += arma::cross(pt, force);}

    FORCE_INLINE arma::fvec3 const& Torque () const {return m_torque;}
    FORCE_INLINE void SetTorque (arma::fvec3 const& torque) {m_torque = torque;}
    FORCE_INLINE void ApplyTorque (arma::fvec3 const& torque) {m_torque += torque;}

    FORCE_INLINE void ApplyImpulse (arma::fvec3 const& impulse) {m_vel += impulse * m_invMass;}
    FORCE_INLINE void ApplyTorqueImpulse (arma::fvec3 const& torqueImpulse) {m_angVel += m_invIntertia * torqueImpulse;}
    void ApplyImpulseAtPoint (arma::fvec3 const& impulse, arma::fvec3 const& pt) {ApplyImpulse(impulse); ApplyTorqueImpulse(arma::cross(impulse, pt));}

    FORCE_INLINE arma::fmat33 const& Orientation () const {return m_orientation;}
    FORCE_INLINE void SetOrientation (Quaternion const& orientation) {m_quatOrientation = orientation;}
    FORCE_INLINE void RotateOrientation (float const& t, arma::fvec3 const& u) {m_quatOrientation = Quaternion::UnitRotation(t, u) * m_quatOrientation;}

    FORCE_INLINE Quaternion const& QuaternionOrientation () const {return m_quatOrientation;}

    FORCE_INLINE arma::fmat33 const& InverseInertia () const {return m_invIntertia;}
    FORCE_INLINE arma::fmat33 Inertia () const {return m_invIntertia.i();}
    FORCE_INLINE arma::fmat33 const& InitialInverseInertia () const {return m_invIntertiaBody;}
    FORCE_INLINE arma::fmat33 InitialInertia () const {return m_invIntertiaBody.i();}

    FORCE_INLINE Collidable* GetCollidable () const {return m_col;}
    FORCE_INLINE void SetCollidable (Collidable* col) {m_col = col;}
};

class Collidable 
{

};

class SphereCollidable : public Collidable
{
private:
    float m_rad;

public:
    SphereCollidable (float const& rad) : m_rad{rad} {}

    float const& Radius () const {return m_rad;}
};

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
    float m_force;
public:
    GravForce (float const& force, RigidBody* rb=nullptr) : m_force{force}, IForce{rb} {}
    virtual void Apply (double const& t) override {(void)t; m_rb->ApplyForce({0.0f,-m_force,0.0f});}

    float Force () const {return m_force;}
    void SetForce (float const& force) {m_force = force;}
};

class SimpleForce : public IForce 
{
private: 
    arma::fvec3 m_force;
public:
    SimpleForce (arma::fvec3 const& force, RigidBody* rb=nullptr) : m_force{force}, IForce{rb} {}
    virtual void Apply (double const& t) override {(void)t; m_rb->ApplyForce(m_force);}

    arma::fvec3 Force () const {return m_force;}
    void SetForce (arma::fvec3 const& force) {m_force = force;}
};

typedef arma::fvec3 (*ForceCallback)(double const&, RigidBody*);
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

class Engine 
{
private:
    std::vector<Entity*> m_ents;
    std::vector<IForce*> m_forces;

    double m_t;
    float m_grav;

    struct Contact 
    {
        RigidBody* rb1, * rb2;
        arma::fvec3 contactPt;
        arma::fvec3 normalOn2;
    };

    std::unique_ptr<ConstraintSolver> m_solver;

public:
    Engine () {}
    Engine (std::vector<Entity*> const& ents);
    Engine (std::vector<Entity*>&& ents);

    bool CollisionDetection ();

//    void UpdatePrimary (float const& dt, std::vector<std::pair<arma::fvec3,arma::fvec3>>& primary);
//    void SolveConstraints (float& dt);
    void Run (float& dt);

    void AddForceToSystem (IForce* const& force) {m_forces.push_back(force);}
    void AddForcesToSystem (std::vector<IForce*> const& forces) {m_forces.insert(m_forces.end(), forces.begin(), forces.end());}
    void ClearForces () {m_forces.clear();}
    void SetTime (double const& t) {m_t = t;}

    void AddEntity (Entity* const& ent) {m_ents.push_back(ent);}
    void AddEntities (std::vector<Entity*> const& ents) {m_ents.insert(m_ents.end(), ents.begin(), ents.end());}

    std::vector<Entity*> const& GetEntities () const {return m_ents;}
    double const& Time () const {return m_t;}

    FORCE_INLINE void SetGravity (float const& grav) {m_grav = grav;}
    FORCE_INLINE float const& GetGravity () const {return m_grav;}
};  

#endif //__PHYS_H__
