#include "constraints.h"
#include "phys.h"
#include "../SGV3D/src/logger.h"

ConstraintSolver::ConstraintSolver (unsigned const& solverIterations)
    : m_solverIterations{solverIterations}
{
    std::copy(ms_defBoundLookup, ms_defBoundLookup + ConstraintType::eCount, m_boundLookup);
}

ConstraintSolver::LambdaBounds const ConstraintSolver::ms_defBoundLookup[ConstraintType::eCount]
{
    {0.0, DBL_MAX}
};

void ConstraintSolver::AddConstraint (PairWiseConstraint* constraint, double const& bias) 
{
    m_constraints.push_back(constraint); 
    m_bias.insert_rows(m_bias.n_rows, arma::vec{bias});
}

arma::vec::fixed<12> ContactConstraint::ComputeJacobian (std::vector<Entity*> const& ents) const
{
	arma::vec3 n{m_normalOn1};
    arma::rowvec3 nT{n.t()};

    RigidBody* rb1{ents[m_rbIdx1]->GetRigidBody()}, * rb2{ents[m_rbIdx2]->GetRigidBody()};

    arma::vec3 r1{m_contactPt - rb1->Position()}, r2{m_contactPt - rb2->Position()};
    arma::rowvec3 cross1{-arma::cross(r1, n).t()}, cross2{arma::cross(r2, n).t()};

    return {-nT[0]   , -nT[1]   , -nT[2]    , 
            cross1[0], cross1[1], cross1[2],
            nT[0]    , nT[1]    , nT[2]     , 
            cross2[0], cross2[1], cross2[2]};
}

void ConstraintSolver::SolveConstraints (std::vector<Entity*> const& ents, arma::vec& V, arma::vec& Fext, double const& dt) 
{
    unsigned s{(unsigned)m_constraints.size()};

    size_t n{ents.size()};

    ASSERT(s == m_bias.n_rows);

	arma::sp_mat J(s, 6*n);
	J.zeros();
	for(unsigned i = 0; i < s; ++i)
	{
		PairWiseConstraint* constraint{m_constraints[i]};
		unsigned const& rbIdx1{constraint->BodyIndex1()},
				        rbIdx2{constraint->BodyIndex2()};

		arma::vec::fixed<12> Jmini{constraint->ComputeJacobian(ents)};

		J(i, 6 * rbIdx1 + 0) = Jmini(0);
		J(i, 6 * rbIdx1 + 1) = Jmini(1);
		J(i, 6 * rbIdx1 + 2) = Jmini(2);
		J(i, 6 * rbIdx1 + 3) = Jmini(3);
		J(i, 6 * rbIdx1 + 4) = Jmini(4);
		J(i, 6 * rbIdx1 + 5) = Jmini(5);

		J(i, 6 * rbIdx2 + 0) = Jmini(6);
		J(i, 6 * rbIdx2 + 1) = Jmini(7);
		J(i, 6 * rbIdx2 + 2) = Jmini(8);
		J(i, 6 * rbIdx2 + 3) = Jmini(9);
		J(i, 6 * rbIdx2 + 4) = Jmini(10);
		J(i, 6 * rbIdx2 + 5) = Jmini(11);
	}

	arma::sp_mat Minv(6*n, 6*n);
	Minv.zeros();
	for(unsigned i = 0; i < n; ++i)
	{
		RigidBody* rb{ents[i]->GetRigidBody()};
		Minv(6*i+0, 6*i+0) = rb->InvMass();
		Minv(6*i+1, 6*i+1) = rb->InvMass();
		Minv(6*i+2, 6*i+2) = rb->InvMass();

		arma::mat33 Iinv{rb->InverseInertia()};
		Minv(6*i+3, 6*i+3) = Iinv(0,0); Minv(6*i+3, 6*i+4) = Iinv(0,1); Minv(6*i+3, 6*i+5) = Iinv(0,2);
		Minv(6*i+4, 6*i+3) = Iinv(1,0); Minv(6*i+4, 6*i+4) = Iinv(1,1); Minv(6*i+4, 6*i+5) = Iinv(1,2);
		Minv(6*i+5, 6*i+3) = Iinv(2,0); Minv(6*i+5, 6*i+4) = Iinv(2,1); Minv(6*i+5, 6*i+5) = Iinv(2,2);
	}
    
    arma::vec lambda0(s);
    lambda0.zeros();

    for(unsigned i = 0; i < s; ++i)
    {
		unsigned const& rbIdx1{m_constraints[i]->BodyIndex1()},
				        rbIdx2{m_constraints[i]->BodyIndex2()};
        
        //TWO LOOKUPS HERE! UGLY!
        auto lookup{m_lambdaCache.find({rbIdx1, rbIdx2})};
        if(lookup != m_lambdaCache.end())
        {
            lambda0(i) = lookup->second;
        }
        else
        {
            lookup = m_lambdaCache.find({rbIdx2, rbIdx1});
            if(lookup != m_lambdaCache.end())
            {
                lambda0(i) = lookup->second;
            }
        }
    }

    arma::vec lambda{lambda0};

	arma::sp_mat A{J * Minv * J.t()};
	arma::vec b{1.0f/dt * m_bias - J * (1.0f/dt * V + Minv * Fext)};
	arma::vec diag{arma::diagvec(A)};

	arma::vec lambdaMin(s);
	arma::vec lambdaMax(s);
	for(unsigned i = 0; i < s; ++i) 
	{
		lambdaMin(i) = LookupMin(m_constraints[i]);
		lambdaMax(i) = LookupMax(m_constraints[i]);
	}

	//Gauss-Seidel solution:
    for(unsigned iter = 0; iter < m_solverIterations; ++iter)
    {
		arma::vec dLambda = (b - A*lambda)/diag;
		lambda0 = lambda;
		lambda = arma::max(lambdaMin, arma::min(lambda0 + dLambda, lambdaMax)); //Clamping lambda to [min, max]
		dLambda = lambda - lambda0;
    }
    arma::vec V_true = V + dt * Minv * (J.t() * lambda + Fext);

    //TODO: FIGURE OUT WHY WE HAVE TO MULTIPLY BY 2 HERE!!
    lambda *= 2.0f;
	V += dt * Minv * (J.t() * lambda + Fext);

    //Cache lambdas for next time
    m_lambdaCache.clear();
    for(unsigned i = 0; i < s; ++i)
    {
		unsigned const& rbIdx1{m_constraints[i]->BodyIndex1()},
				        rbIdx2{m_constraints[i]->BodyIndex2()};
        m_lambdaCache[std::make_pair(rbIdx1, rbIdx2)] = lambda(i);
    }
}

void GaussSeidelClampedSolve (arma::sp_mat const& A, arma::vec& x, arma::mat const& b, arma::vec const& x_min, arma::vec const&  x_max, unsigned const& solver_iterations)
{
    arma::vec diag{arma::diagvec(A)};
    for(unsigned iter = 0; iter < solver_iterations; ++iter)
    {
        x += (b - A*x)/diag;
        x = arma::max(x_min, arma::min(x, x_max)); //Clamping lambda to [min, max]
    }
}

void GaussSeidelSolve (arma::sp_mat const& A, arma::vec& x, arma::mat const& b, unsigned const& solver_iterations)
{
    arma::vec diag{arma::diagvec(A)};
    for(unsigned iter = 0; iter < solver_iterations; ++iter)
    {
        x += (b - A*x)/diag;
    }
}

//See paper "Reflections on simultaneous impacts"
void ConstraintSolver::GeneralizedReflections (std::vector<Entity*> const& ents, arma::vec& V, arma::vec& Fext, double const& dt) 
{
    unsigned s{(unsigned)m_constraints.size()};

    size_t n{ents.size()};

    ASSERT(s == m_bias.n_rows);

	arma::sp_mat Minv(6*n, 6*n);
	Minv.zeros();
	for(unsigned i = 0; i < n; ++i)
	{
		RigidBody* rb{ents[i]->GetRigidBody()};
		Minv(6*i+0, 6*i+0) = rb->InvMass();
		Minv(6*i+1, 6*i+1) = rb->InvMass();
		Minv(6*i+2, 6*i+2) = rb->InvMass();

		arma::mat33 Iinv{rb->InverseInertia()};
		Minv(6*i+3, 6*i+3) = Iinv(0,0); Minv(6*i+3, 6*i+4) = Iinv(0,1); Minv(6*i+3, 6*i+5) = Iinv(0,2);
		Minv(6*i+4, 6*i+3) = Iinv(1,0); Minv(6*i+4, 6*i+4) = Iinv(1,1); Minv(6*i+4, 6*i+5) = Iinv(1,2);
		Minv(6*i+5, 6*i+3) = Iinv(2,0); Minv(6*i+5, 6*i+4) = Iinv(2,1); Minv(6*i+5, 6*i+5) = Iinv(2,2);
	}

    std::pair<bool, arma::vec::fixed<12>> violations[s]; //Which constraints are opposing the normal?
    unsigned active_map[s];
    size_t sz; //violation count
    arma::vec::fixed<12> zero;
    zero.zeros();

    V += dt * Minv * Fext; //Update initial velocities
    
    std::map<std::pair<unsigned,unsigned>, double> newLambdaCache;
    while(true)
    {
        std::fill(violations, violations + s, std::make_pair(false, zero));
        sz = 0;

        for(unsigned i = 0; i < s; ++i)
        {
            PairWiseConstraint* constraint{m_constraints[i]};
            arma::vec::fixed<12> jacobian{constraint->ComputeJacobian(ents)};
            unsigned const& rbIdx1{constraint->BodyIndex1()},
                            rbIdx2{constraint->BodyIndex2()};

            arma::vec::fixed<6> p1, p2;

            p1(0) = V(6*rbIdx1 + 0); p1(1) = V(6*rbIdx1 + 1); p1(2) = V(6*rbIdx1 + 2);
            p1(3) = V(6*rbIdx1 + 3); p1(4) = V(6*rbIdx1 + 4); p1(5) = V(6*rbIdx1 + 5);

            p2(0) = V(6*rbIdx2 + 0); p2(1) = V(6*rbIdx2 + 1); p2(2) = V(6*rbIdx2 + 2);
            p2(3) = V(6*rbIdx2 + 3); p2(4) = V(6*rbIdx2 + 4); p2(5) = V(6*rbIdx2 + 5);

            arma::vec::fixed<12> p{arma::join_cols(p1, p2)};

            if(arma::dot(jacobian, p) <= -DBL_EPSILON)
            {
                violations[i] = std::make_pair(true, jacobian);
                active_map[sz] = i;
                ++sz;
            }
        }

        if(sz == 0)
            break;

        arma::sp_mat J(sz, 6*n);
        J.zeros();
        arma::vec bias(sz);
        for(unsigned i = 0; i < sz; ++i)
        {
            unsigned idx{active_map[i]};
            PairWiseConstraint* constraint{m_constraints[idx]};
            unsigned const& rbIdx1{constraint->BodyIndex1()},
                            rbIdx2{constraint->BodyIndex2()};

            arma::vec::fixed<12> Jmini{constraint->ComputeJacobian(ents)};

            J(i, 6 * rbIdx1 + 0) = Jmini(0);
            J(i, 6 * rbIdx1 + 1) = Jmini(1);
            J(i, 6 * rbIdx1 + 2) = Jmini(2);
            J(i, 6 * rbIdx1 + 3) = Jmini(3);
            J(i, 6 * rbIdx1 + 4) = Jmini(4);
            J(i, 6 * rbIdx1 + 5) = Jmini(5);

            J(i, 6 * rbIdx2 + 0) = Jmini(6);
            J(i, 6 * rbIdx2 + 1) = Jmini(7);
            J(i, 6 * rbIdx2 + 2) = Jmini(8);
            J(i, 6 * rbIdx2 + 3) = Jmini(9);
            J(i, 6 * rbIdx2 + 4) = Jmini(10);
            J(i, 6 * rbIdx2 + 5) = Jmini(11);

            bias(i) = m_bias(idx);
        }

        arma::vec lambda(sz);
        lambda.zeros();

        arma::vec lambdaMin(sz);
        arma::vec lambdaMax(sz);

        for(unsigned i = 0; i < sz; ++i)
        {
            unsigned idx{active_map[i]};
            PairWiseConstraint* const& constraint{m_constraints[idx]};
            unsigned const& rbIdx1{constraint->BodyIndex1()},
                            rbIdx2{constraint->BodyIndex2()};
            
            //TWO LOOKUPS HERE! UGLY!
            auto lookup{m_lambdaCache.find({rbIdx1, rbIdx2})};
            if(lookup != m_lambdaCache.end())
            {
                lambda(i) = lookup->second;
            }
            else
            {
                lookup = m_lambdaCache.find({rbIdx2, rbIdx1});
                if(lookup != m_lambdaCache.end())
                {
                    lambda(i) = lookup->second;
                }
            }

            lambdaMin(i) = LookupMin(constraint);
            lambdaMax(i) = LookupMax(constraint);
        }

        arma::sp_mat A{J * Minv * J.t()};
        arma::vec b{1.0/dt * bias - J * V * 1.0/dt};

//		GaussSeidelClampedSolve(A, lambda, b, lambdaMin, lambdaMax, m_solverIterations);
        //TODO: USE SUPERLU HERE!
        arma::spsolve(lambda, A, b);
        lambda = arma::max(lambdaMin, arma::min(lambdaMax, lambda));
		
	
#if 0  //TODO: Determine if I want this 

		arma::vec lambda_err(sz);
		lambda_err.zeros();

		//Solve for error 
		GaussSeidelSolve(A, lambda_err, b - A * lambda, 2);
        lambda -= lambda_err;
#endif 

        lambda *= 2.0;

        //Cache lambdas for next time
        for(unsigned i = 0; i < sz; ++i)
        {
            unsigned idx{active_map[i]};
            unsigned const& rbIdx1{m_constraints[idx]->BodyIndex1()},
                            rbIdx2{m_constraints[idx]->BodyIndex2()};
            newLambdaCache[std::make_pair(rbIdx1, rbIdx2)] = lambda(i);
        }

        V += dt * Minv * J.t() * lambda;
    }

    m_lambdaCache = newLambdaCache;
}

void ConstraintSolver::ClearConstraints ()
{
    for(auto const& constraint: m_constraints)
    {
        if(constraint)
            delete constraint;
    }
    m_constraints.clear();
    m_bias.resize(0);
}
