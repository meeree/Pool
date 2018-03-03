#include "constraints.h"
#include "phys.h"
#include "../../SGV3D/src/logger.h"

ConstraintSolver::ConstraintSolver (unsigned const& solverIterations)
    : m_solverIterations{solverIterations}
{
    std::copy(ms_defBoundLookup, ms_defBoundLookup + ConstraintType::eCount, m_boundLookup);
}

ConstraintSolver::LambdaBounds const ConstraintSolver::ms_defBoundLookup[ConstraintType::eCount]
{
    {0.0f, FLT_MAX}
};

void ConstraintSolver::AddConstraint (PairWiseConstraint* constraint, float const& bias) 
{
    m_constraints.push_back(constraint); 
    m_bias.insert_rows(m_bias.n_rows, arma::fvec{bias});
}

arma::fvec::fixed<12> ContactConstraint::ComputeJacobian (std::vector<Entity*> const& ents) const
{
	arma::fvec3 n{m_normalOn1};
    arma::frowvec3 nT{n.t()};

    RigidBody* rb1{ents[m_rbIdx1]->GetRigidBody()}, * rb2{ents[m_rbIdx2]->GetRigidBody()};

    arma::fvec3 r1{m_contactPt - rb1->Position()}, r2{m_contactPt - rb2->Position()};
    arma::frowvec3 cross1{-arma::cross(r1, n).t()}, cross2{arma::cross(r2, n).t()};

    return {-nT[0]   , -nT[1]   , -nT[2]    , 
            cross1[0], cross1[1], cross1[2],
            nT[0]    , nT[1]    , nT[2]     , 
            cross2[0], cross2[1], cross2[2]};
}

void ConstraintSolver::SolveConstraints (std::vector<Entity*> const& ents, arma::fvec& V, arma::fvec& Fext, float const& dt) 
{
    unsigned s{(unsigned)m_constraints.size()};

    size_t n{ents.size()};

    ASSERT(s == m_bias.n_rows);

	arma::sp_fmat J(s, 6*n);
	J.zeros();
	for(unsigned i = 0; i < s; ++i)
	{
		PairWiseConstraint* constraint{m_constraints[i]};
		unsigned const& rbIdx1{constraint->BodyIndex1()},
				        rbIdx2{constraint->BodyIndex2()};

		arma::fvec::fixed<12> Jmini{constraint->ComputeJacobian(ents)};

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

	arma::sp_fmat Minv(6*n, 6*n);
	Minv.zeros();
	for(unsigned i = 0; i < n; ++i)
	{
		RigidBody* rb{ents[i]->GetRigidBody()};
		Minv(6*i+0, 6*i+0) = rb->InvMass();
		Minv(6*i+1, 6*i+1) = rb->InvMass();
		Minv(6*i+2, 6*i+2) = rb->InvMass();

		arma::fmat33 Iinv{rb->InverseInertia()};
		Minv(6*i+3, 6*i+3) = Iinv(0,0); Minv(6*i+3, 6*i+4) = Iinv(0,1); Minv(6*i+3, 6*i+5) = Iinv(0,2);
		Minv(6*i+4, 6*i+3) = Iinv(1,0); Minv(6*i+4, 6*i+4) = Iinv(1,1); Minv(6*i+4, 6*i+5) = Iinv(1,2);
		Minv(6*i+5, 6*i+3) = Iinv(2,0); Minv(6*i+5, 6*i+4) = Iinv(2,1); Minv(6*i+5, 6*i+5) = Iinv(2,2);
	}
    
    arma::fvec lambda0(s);
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

    arma::fvec lambda{lambda0};

	arma::sp_fmat A{J * Minv * J.t()};
	arma::fvec b{1.0f/dt * m_bias - J * (1.0f/dt * V + Minv * Fext)};
	arma::fvec diag{arma::diagvec(A)};

	arma::fvec lambdaMin(s);
	arma::fvec lambdaMax(s);
	for(unsigned i = 0; i < s; ++i) 
	{
		lambdaMin(i) = LookupMin(m_constraints[i]);
		lambdaMax(i) = LookupMax(m_constraints[i]);
	}

	//Gauss-Seidel solution:
    for(unsigned iter = 0; iter < m_solverIterations; ++iter)
    {
		arma::fvec dLambda = (b - A*lambda)/diag;
		lambda0 = lambda;
		lambda = arma::max(lambdaMin, arma::min(lambda0 + dLambda, lambdaMax)); //Clamping lambda to [min, max]
		dLambda = lambda - lambda0;
    }

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

//void ConstraintSolver::SolveConstraints (std::vector<Entity*> const& ents, float const& dt, unsigned const& itMax) 
//{
//    unsigned s{(unsigned)m_constraints.size()};
//
//    size_t n{ents.size()};
//
//    ASSERT(s == m_bias.n_rows);
//
//	arma::fmat Jsmall(s, 12);
//	for(unsigned i = 0; i < s; ++i)
//	{
//		arma::fvec::fixed<12> Jmini{m_constraints[i]->ComputeJacobian(ents)};
//		Jsmall.col(i) = Jmini;
//	}
//
////	arma::fmat Minv(6*n, 6*n);
////	Minv.zeros();
////	for(unsigned i = 0; i < n; ++i)
////	{
////		RigidBody* rb{ents[i]->GetRigidBody()};
////		Minv(6*i+0, 6*i+0) = rb->InvMass();
////		Minv(6*i+1, 6*i+1) = rb->InvMass();
////		Minv(6*i+2, 6*i+2) = rb->InvMass();
////
////		arma::fmat33 Iinv{rb->InverseInertia()};
////		Minv(6*i+3, 6*i+3) = Iinv(0,0); Minv(6*i+3, 6*i+4) = Iinv(0,1); Minv(6*i+3, 6*i+5) = Iinv(0,2);
////		Minv(6*i+4, 6*i+3) = Iinv(1,0); Minv(6*i+4, 6*i+4) = Iinv(1,1); Minv(6*i+4, 6*i+5) = Iinv(1,2);
////		Minv(6*i+5, 6*i+3) = Iinv(2,0); Minv(6*i+5, 6*i+4) = Iinv(2,1); Minv(6*i+5, 6*i+5) = Iinv(2,2);
////	}
//    
//    arma::fvec lambda0(s);
//    lambda0.zeros();
//
//    for(unsigned i = 0; i < s; ++i)
//    {
//		unsigned const& rbIdx1{m_constraints[i]->BodyIndex1()},
//				        rbIdx2{m_constraints[i]->BodyIndex2()};
//        
//        //TWO LOOKUPS HERE! UGLY!
//        auto lookup{m_lambdaCache.find({rbIdx1, rbIdx2})};
//        if(lookup != m_lambdaCache.end())
//        {
//            lambda0(i) = lookup->second;
//        }
//        else
//        {
//            lookup = m_lambdaCache.find({rbIdx2, rbIdx1});
//            if(lookup != m_lambdaCache.end())
//            {
//                lambda0(i) = lookup->second;
//            }
//        }
//    }
//
//    arma::fvec lambda{lambda0};
//
//	arma::fmat A(s,s);
//	A.zeros();
//	for(unsigned i = 0; i < s; ++i)
//	{
//	}
//
//	arma::fmat A{J * Minv * J.t()};
//
//
//
//
//	arma::fvec b{1.0f/dt * m_bias - J * (1.0f/dt * V + Minv * Fext)};
//	arma::fvec diag(s);
//	for(unsigned i = 0; i < s; ++i) 
//	{
//		diag(i) = A(i,i);
//	}
//
//	arma::fvec lambdaMin(s);
//	arma::fvec lambdaMax(s);
//	for(unsigned i = 0; i < s; ++i) 
//	{
//		lambdaMin(i) = LookupMin(m_constraints[i]);
//		lambdaMax(i) = LookupMax(m_constraints[i]);
//	}
//
//	//Gauss-Seidel solution:
//    for(unsigned iter = 0; iter < itMax; ++iter)
//    {
//		arma::fvec dLambda = (b - A*lambda)/diag;
//		lambda0 = lambda;
//		lambda = arma::max(lambdaMin, arma::min(lambda0 + dLambda, lambdaMax)); //Clamping lambda to [min, max]
//		dLambda = lambda - lambda0;
//    }
//
//	V += dt * Minv * (J.t() * lambda + Fext);
//
//    //Cache lambdas for next time
//    m_lambdaCache.clear();
//    for(unsigned i = 0; i < s; ++i)
//    {
//		unsigned const& rbIdx1{m_constraints[i]->BodyIndex1()},
//				        rbIdx2{m_constraints[i]->BodyIndex2()};
//        m_lambdaCache[std::make_pair(rbIdx1, rbIdx2)] = lambda(i);
//    }
//	std::cout<<V<<std::endl;
//}
