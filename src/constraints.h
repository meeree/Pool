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
    virtual arma::vec::fixed<12> ComputeJacobian (std::vector<Entity*> const& ents) const = 0;
    virtual ~PairWiseConstraint() {}

    inline ConstraintType const& Type () const {return m_type;}
    inline unsigned const& BodyIndex1 () const {return m_rbIdx1;}
    inline unsigned const& BodyIndex2 () const {return m_rbIdx2;}
};

//NOTE: No jacobian map is needed because inidices are available directly through constraints
class ContactConstraint : public PairWiseConstraint 
{
private:
    arma::vec3 m_contactPt;
    arma::vec3 m_normalOn1;

public:
    inline ContactConstraint (unsigned const& rbIdx1, unsigned const& rbIdx2, arma::vec3 const& contactPt, arma::vec3 const& normalOn1)
       : m_contactPt{contactPt}, m_normalOn1{normalOn1}, PairWiseConstraint{rbIdx1, rbIdx2, eContact} {}

    virtual arma::vec::fixed<12> ComputeJacobian (std::vector<Entity*> const& ents) const;

    inline arma::vec3 const& ContactPoint () const {return m_contactPt;}
    inline arma::vec3 const& NormalOnBody1 () const {return m_normalOn1;}
};

class ConstraintSolver 
{
private:
    arma::vec m_bias;
    std::vector<PairWiseConstraint*> m_constraints;

    unsigned m_solverIterations;
    
    struct LambdaBounds 
    {
        double boundMin, boundMax;
    }; 
    LambdaBounds m_boundLookup[ConstraintType::eCount];
    static LambdaBounds const ms_defBoundLookup[ConstraintType::eCount];

    std::map<std::pair<unsigned,unsigned>, double> m_lambdaCache;

    inline double const& LookupMin (PairWiseConstraint* constraint) const {return m_boundLookup[constraint->Type()].boundMin;}
    inline double const& LookupMax (PairWiseConstraint* constraint) const {return m_boundLookup[constraint->Type()].boundMax;}

public:
    ConstraintSolver (unsigned const& solverIterations);

    void SolveConstraints (std::vector<Entity*> const& ents, arma::vec& V, arma::vec& Fext, double const& dt);
    void GeneralizedReflections (std::vector<Entity*> const& ents, arma::vec& V, arma::vec& Fext, double const& dt);
    
    void AddConstraint (PairWiseConstraint* constraint, double const& bias);

    void ClearConstraints ();

    inline size_t ConstraintCount () {return m_constraints.size();}

    inline void SetLambdaBounds (LambdaBounds const (&boundLookup)[ConstraintType::eCount]) {std::copy(boundLookup, boundLookup + ConstraintType::eCount, m_boundLookup);}

    inline unsigned const& SolverIterations () const {return m_solverIterations;}
    inline void SetSolverIterations (unsigned const& solverIterations) {m_solverIterations = solverIterations;}
};

#endif //__CONSTRAINTS_H__
