#ifndef  __CONSTRAINTS_H__
#define  __CONSTRAINTS_H__

#include "core.h"
#include <map>

class Entity;

enum ConstraintType : char
{
    eContact=0,
    eCount
};

class PairWiseConstraint 
{
protected:
    unsigned m_rbIdx1, m_rbIdx2; 
    ConstraintType m_type;

    inline PairWiseConstraint (unsigned const& rbIdx1, unsigned const& rbIdx2, ConstraintType type) 
        : m_rbIdx1{rbIdx1}, m_rbIdx2{rbIdx2}, m_type{type} {}

public:
    virtual arma::fvec::fixed<12> ComputeJacobian (std::vector<Entity*> const& ents) const = 0;
    virtual ~PairWiseConstraint() {}

    FORCE_INLINE ConstraintType const& Type () const {return m_type;}
    FORCE_INLINE unsigned const& BodyIndex1 () const {return m_rbIdx1;}
    FORCE_INLINE unsigned const& BodyIndex2 () const {return m_rbIdx2;}
};

///NOTE : NORMAL MIGHT BE INCORRECT SIGN
//NOTE: No jacobian map is needed because inidices are available directly through constraints
class ContactConstraint : public PairWiseConstraint 
{
private:
    arma::fvec3 m_contactPt;
    arma::fvec3 m_normalOn1;

public:
    inline ContactConstraint (unsigned const& rbIdx1, unsigned const& rbIdx2, arma::fvec3 const& contactPt, arma::fvec3 const& normalOn1)
       : m_contactPt{contactPt}, m_normalOn1{normalOn1}, PairWiseConstraint{rbIdx1, rbIdx2, eContact} {}

    virtual arma::fvec::fixed<12> ComputeJacobian (std::vector<Entity*> const& ents) const;

    FORCE_INLINE arma::fvec3 const& ContactPoint () const {return m_contactPt;}
    FORCE_INLINE arma::fvec3 const& NormalOnBody1 () const {return m_normalOn1;}
};

class ConstraintSolver 
{
private:
    arma::fvec m_bias;
    std::vector<PairWiseConstraint*> m_constraints;

    unsigned m_solverIterations;
    
    struct LambdaBounds 
    {
        float boundMin, boundMax;
    }; 
    LambdaBounds m_boundLookup[ConstraintType::eCount];
    static LambdaBounds const ms_defBoundLookup[ConstraintType::eCount];

    std::map<std::pair<unsigned,unsigned>, float> m_lambdaCache;

    FORCE_INLINE float const& LookupMin (PairWiseConstraint* constraint) const {return m_boundLookup[constraint->Type()].boundMin;}
    FORCE_INLINE float const& LookupMax (PairWiseConstraint* constraint) const {return m_boundLookup[constraint->Type()].boundMax;}

public:
    ConstraintSolver (unsigned const& solverIterations);

    void SolveConstraints (std::vector<Entity*> const& ents, arma::fvec& V, arma::fvec& Fext, float const& dt);
    
    void AddConstraint (PairWiseConstraint* constraint, float const& bias);

    void ClearConstraints ();

    inline size_t ConstraintCount () {return m_constraints.size();}

    inline void SetLambdaBounds (LambdaBounds const (&boundLookup)[ConstraintType::eCount]) {std::copy(boundLookup, boundLookup + ConstraintType::eCount, m_boundLookup);}

    inline unsigned const& SolverIterations () const {return m_solverIterations;}
    inline void SetSolverIterations (unsigned const& solverIterations) {m_solverIterations = solverIterations;}
};

#endif //__CONSTRAINTS_H__
