/*
 * TssBdMpi.cpp
 *
 *  Created on: Nov 19, 2015
 *      Author: kibaekkim
 */

//#define DSP_DEBUG

#include <Solver/TssBdMpi.h>
#include "Utility/StoUtility.h"
#include "SolverInterface/SCIPconshdlrBendersMPI.h"
#include "SolverInterface/SolverInterfaceSpx.h"
#include "SolverInterface/SolverInterfaceClp.h"
#include "SolverInterface/SolverInterfaceScip.h"

TssBdMpi::TssBdMpi(MPI_Comm comm):
	TssBd(),
	comm_(comm),
	num_comm_groups_(-1),
	comm_group_(-1),
	parProcIdxSize_(0),
	parProcIdx_(NULL),
	probability_(NULL),
	probabilitySum_(0.0),
	procIdxSizes_(NULL)
{
	MPI_Comm_rank(comm, &comm_rank_);
	MPI_Comm_size(comm, &comm_size_);
}

TssBdMpi::~TssBdMpi()
{
	FREE_ARRAY_PTR(parProcIdx_);
	FREE_ARRAY_PTR(probability_);
	FREE_ARRAY_PTR(procIdxSizes_);
}

/** initialize solver */
STO_RTN_CODE TssBdMpi::initialize()
{
#define FREE_MEMORY \
	FREE_ARRAY_PTR(lprobability); \
	FREE_ARRAY_PTR(recvbuf); \
	FREE_ARRAY_PTR(recvcounts); \
	FREE_ARRAY_PTR(displs);

	/** communication */
	double * lprobability = NULL;
	double * recvbuf = NULL;
	int * recvcounts = NULL;
	int * displs = NULL;

	BGN_TRY_CATCH

	/** parameters */
	parNumCores_    = par_->getIntParam("BD/NUM_CORES");
	parCutPriority_ = par_->getIntParam("BD/CUT_PRIORITY");
	parInitLbAlgo_  = par_->getIntParam("BD/INIT_LB_ALGO");

	if (comm_rank_ == 0)
		procIdxSizes_ = new int [comm_size_];

	int nsubprobs = model_->getNumScenarios();
	int effective_comm_size = comm_size_;

	if (parProcIdxSize_ == 0)
	{
		effective_comm_size = CoinMin(effective_comm_size, nsubprobs);
		if (comm_rank_ < effective_comm_size)
		{
			/** round-robin */
			int size = (int) ((nsubprobs - 1) - comm_rank_) / effective_comm_size + 1;
			assert(size > 0);
			parProcIdxSize_ = size;
			parProcIdx_ = new int [size];
			int i = 0;
			for (int s = comm_rank_; s < nsubprobs; s += effective_comm_size)
				parProcIdx_[i++] = s;
		}
	}

	/** Let the root know how many scenarios each process use. */
	MPI_Gather(&parProcIdxSize_, 1, MPI_INT, procIdxSizes_, 1, MPI_INT, 0, comm_);

	if (comm_rank_ == 0)
	{
		recvcounts = new int [comm_size_];
		displs = new int [comm_size_];
		for (int i = 0; i < comm_size_; ++i)
		{
			recvcounts[i] = procIdxSizes_[i];
			displs[i] = i == 0 ? 0 : displs[i-1] + recvcounts[i-1];
			DSPdebugMessage("recvcounts_[%d] %d displs_[%d] %d\n", i, recvcounts[i], i, displs[i]);
		}
		recvbuf = new double [displs[comm_size_-1]+recvcounts[comm_size_-1]];
	}

	/** collect probability */
	lprobability = new double [parProcIdxSize_];
	for (int i = 0; i < parProcIdxSize_; ++i)
	{
		lprobability[i] = model_->getProbability()[parProcIdx_[i]];
		DSPdebugMessage("[%d] probability[%d] %e\n", comm_rank_, i, lprobability[i]);
	}
	MPI_Gatherv(lprobability, parProcIdxSize_, MPI_DOUBLE, recvbuf, recvcounts, displs, MPI_DOUBLE, 0, comm_);

	if (comm_rank_ == 0)
	{
		probability_ = new double [nsubprobs];
		for (int i = 0, j = 0; i < comm_size_; ++i)
		{
			for (int s = i; s < nsubprobs; s += comm_size_)
			{
				probability_[s] = recvbuf[j++];
				probabilitySum_ += probability_[s];
				DSPdebugMessage("probability[%d] %e\n", s, probability_[s]);
			}
		}
	}

	/** broadcast the sum of probabilities */
	MPI_Bcast(&probabilitySum_, 1, MPI_DOUBLE, 0, comm_);
	DSPdebugMessage("[%d] sum of probabilities %e\n", comm_rank_, probabilitySum_);

	/** set objective coefficients for auxiliary variables */
	CoinZeroN(obj_aux_, naux_);
	if (comm_rank_ == 0)
	{
		for (int s = 0; s < model_->getNumScenarios(); ++s)
			obj_aux_[s % naux_] += probability_[s];
	}

	/** Root process can print out. */
	if (comm_rank_ > 0) message_->logLevel_ = -999;

	END_TRY_CATCH_RTN(FREE_MEMORY,STO_RTN_ERR)

	FREE_MEMORY

	return STO_RTN_OK;
#undef FREE_MEMORY
}

/** solve */
STO_RTN_CODE TssBdMpi::solve()
{
#define FREE_MEMORY    \
	FREE_PTR(tssbdsub)

	assert(model_);
	assert(par_);

	/** subproblems */
	double tic;
	TssBdSub * tssbdsub = NULL; /**< cut generator */
	double lowerbound = 0.0;

	BGN_TRY_CATCH

	/** initialize */
	initialize();

	/** solution time */
	double swtime = CoinGetTimeOfDay();

	message_->print(1, "\nPhase 1:\n");
	message_->print(1, "Creating Benders sub problems ...");
	tic = CoinGetTimeOfDay();

	/** configure Benders cut generator */
	/** This does NOT make deep copies. So, DO NOT RELEASE POINTERS. */
	tssbdsub = new TssBdSub(par_);
	for (int s = 0; s < parProcIdxSize_; ++s)
		tssbdsub->scenarios_.push_back(parProcIdx_[s]);
	STO_RTN_CHECK_THROW(tssbdsub->loadProblem(model_, naugs_, augs_, naux_), "loadProblem", "TssBdSub");

	message_->print(1, " (%.2f sec)\n", CoinGetTimeOfDay() - tic);


	/** solution time */
	double stime = clockType_ ? CoinGetTimeOfDay() : CoinCpuTime();

	message_->print(1, "Finding global lower bound ...");
	tic = CoinGetTimeOfDay();

	/** find lower bound */
	/** TODO: can replace this with TssDd */
	STO_RTN_CHECK_THROW(findLowerBound(lowerbound), "findLowerBound", "TssBdMpi");

	message_->print(1, " (%.2f sec) -> Lower bound %e\n", CoinGetTimeOfDay() - tic, lowerbound);
	message_->print(1, "Creating master problem instance ...");
	tic = CoinGetTimeOfDay();

	/** construct master problem */
	STO_RTN_CHECK_THROW(constructMasterProblem(tssbdsub, lowerbound), "constructMasterProblem", "TssBdMpi");

	message_->print(1, " (%.2f sec)\n", CoinGetTimeOfDay() - tic);
	message_->print(1, "\nPhase 2:\n");

	time_remains_ -= CoinGetTimeOfDay() - swtime;

	/** configure Benders master */
	STO_RTN_CHECK_THROW(configureMaster(tssbdsub), "configureMaster", "TssBdMpi");

	/** run workers */
	STO_RTN_CHECK_THROW(runWorkers(tssbdsub), "runWorkers", "TssBdMpi");

	/** solve Benders master */
	STO_RTN_CHECK_THROW(runMaster(tssbdsub), "runMaster", "TssBdMpi");

	/** solution time */
	solutionTime_ = (clockType_ ? CoinGetTimeOfDay() : CoinCpuTime()) - stime;

	/** broadcast status_ */
	MPI_Bcast(&status_, 1, MPI_INT, 0, comm_);

	message_->print(1, "Solution Status: %d\n", status_);
	message_->print(1, "Collecting results ...");
	tic = CoinGetTimeOfDay();

	/** collect solution */
	switch (status_)
	{
	case STO_STAT_OPTIMAL:
	case STO_STAT_LIM_ITERorTIME:
	case STO_STAT_STOPPED_GAP:
	case STO_STAT_STOPPED_NODE:
	case STO_STAT_STOPPED_TIME:
		{
			/** MPI communication */
			double primalBound = 0.0;
			double * sendbuf = new double [parProcIdxSize_ * model_->getNumCols(1)];
			double * recvbuf = NULL;
			int * recvcounts = NULL;
			int * displs = NULL;

			if (comm_rank_ == 0)
			{
				recvbuf = new double [model_->getNumScenarios() * model_->getNumCols(1)];
				recvcounts = new int [comm_size_];
				displs = new int [comm_size_];

				for (int i = 0; i < comm_size_; ++i)
				{
					recvcounts[i] = procIdxSizes_[i] * model_->getNumCols(1);
					displs[i] = i == 0 ? 0 : displs[i-1] + recvcounts[i-1];
					DSPdebugMessage("recvcounts_[%d] %d displs_[%d] %d\n", i, recvcounts[i], i, displs[i]);
				}
			}

			/** first-stage solution */
			assert(solution_);
			if (comm_rank_ == 0)
				CoinCopyN(si_->getSolution(), model_->getNumCols(0), solution_);
			MPI_Bcast(solution_, model_->getNumCols(0), MPI_DOUBLE, 0, comm_);

			/** second-stage solution */
			double * objval_reco = new double [model_->getNumScenarios()];
			double ** solution_reco = new double * [model_->getNumScenarios()];
			for (int s = 0; s < model_->getNumScenarios(); ++s)
				solution_reco[s] = new double [model_->getNumCols(1)];

			if (comm_rank_ == 0)
			{
				for (int s = 0; s < naugs_; ++s)
					CoinCopyN(si_->getSolution() + model_->getNumCols(0) + s * model_->getNumCols(1),
							model_->getNumCols(1), solution_reco[augs_[s]]);
			}

			/** collect solution */
			tssbdsub->solveRecourse(solution_, objval_reco, solution_reco, parNumCores_);
			for (int s = 0; s < parProcIdxSize_; ++s)
			{
				int ss = parProcIdx_[s];
				CoinCopyN(solution_reco[ss], model_->getNumCols(1), sendbuf + s * model_->getNumCols(1));
			}

			MPI_Gatherv(sendbuf, parProcIdxSize_ * model_->getNumCols(1), MPI_DOUBLE,
					recvbuf, recvcounts, displs, MPI_DOUBLE, 0, comm_);

			if (comm_rank_ == 0)
			{
				for (int i = 0, j = 0; i < comm_size_; ++i)
				{
					if (comm_rank_ == i) continue;
					for (int s = i; s < model_->getNumScenarios(); s += comm_size_)
					{
						CoinCopyN(recvbuf + j * model_->getNumCols(1), model_->getNumCols(1),
								solution_ + model_->getNumCols(0) + model_->getNumCols(1) * s);
						j++;
					}
				}

				/** compute primal bound */
				for (int j = 0; j < model_->getNumCols(0); ++j)
					primalBound += model_->getObjCore(0)[j] * solution_[j];
			}
			for (int s = 0; s < parProcIdxSize_; ++s)
			{
				int ss = parProcIdx_[s];
				primalBound += objval_reco[ss] * model_->getProbability()[ss];
				//DSPdebugMessage("[%d] objval_reco[%d] %e\n", comm_rank_, ss, objval_reco[ss]);
			}
			DSPdebugMessage("[%d] primalBound %e\n", comm_rank_, primalBound);

			/** sum primal bounds from all processes */
			MPI_Reduce(&primalBound, &primalBound_, 1, MPI_DOUBLE, MPI_SUM, 0, comm_);
			if (comm_rank_ == 0)
				DSPdebugMessage("primalBound %e\n", primalBound_);

			/** free memory */
			FREE_ARRAY_PTR(sendbuf);
			FREE_ARRAY_PTR(recvbuf);
			FREE_ARRAY_PTR(recvcounts);
			FREE_ARRAY_PTR(displs);
			FREE_ARRAY_PTR(objval_reco);
			FREE_2D_ARRAY_PTR(model_->getNumScenarios(), solution_reco);
		}
		break;
	default:
		if (comm_rank_ == 0)
			printf("Solution status (%d).\n", status_);
		break;
	}

	message_->print(1, " (%.2f sec)\n", CoinGetTimeOfDay() - tic);

	END_TRY_CATCH_RTN(FREE_MEMORY,STO_RTN_ERR)

	/** free memory */
	FREE_MEMORY

	return STO_RTN_OK;

#undef FREE_MEMORY
}

/** to find a lower bound by solving a set of group subproblems */
STO_RTN_CODE TssBdMpi::findLowerBound(double & lowerbound)
{
#define FREE_MEMORY       \
	FREE_PTR(mat)         \
	FREE_ARRAY_PTR(clbd)  \
	FREE_ARRAY_PTR(cubd)  \
	FREE_ARRAY_PTR(ctype) \
	FREE_ARRAY_PTR(obj)   \
	FREE_ARRAY_PTR(rlbd)  \
	FREE_ARRAY_PTR(rubd)

	/** problem */
	SolverInterface ** si = NULL;
	CoinPackedMatrix * mat = NULL;
	double * clbd = NULL;
	double * cubd = NULL;
	double * obj = NULL;
	char * ctype = NULL;
	double * rlbd = NULL;
	double * rubd = NULL;
	int augs[1];
	bool doNext = true;

	BGN_TRY_CATCH

	/** allocate memory */
	si = new SolverInterface * [parProcIdxSize_];

	/** initialize lower bound */
	double localLB = 0.0;
	lowerbound = 0.0;

	/** create models */
	DSPdebugMessage("[%d] creating lower bouding problems\n", comm_rank_);
	for (int s = 0; s < parProcIdxSize_; ++s)
	{
		/** augmented scenario index */
		augs[0] = parProcIdx_[s];

		/** decompose model */
		STO_RTN_CHECK_THROW(
				model_->decompose(1, &augs[0], 0, NULL,
						NULL, NULL, mat, clbd, cubd, ctype, obj,
						rlbd, rubd), "decompose", "TssModel");
		DSPdebug(mat->verifyMtx(4));

		/** relax second stage? */
		if (parRelaxIntegrality_[1])
		{
			/** relax integrality in the second stage */
			for (int j = 0; j < model_->getNumCols(1); ++j)
				ctype[model_->getNumCols(0) + j] = 'C';
		}

		/** adjust first-stage cost */
		for (int j = 0; j < model_->getNumCols(0); ++j)
			obj[j] *= model_->getProbability()[parProcIdx_[s]] / probabilitySum_;

		/** create solve interface */
		/** TODO Solving MIP creates a load imbalancing. */
		if (parInitLbAlgo_ == SEPARATE_MILP && model_->getNumIntegers(0) > 0)
		{
			si[s] = new SolverInterfaceScip(par_);
			//si[s]->setPrintLevel(CoinMax(CoinMin(parLogLevel_ - 1, 5),0));
			si[s]->setPrintLevel(0);
		}
		else
		{
			si[s] = new SolverInterfaceSpx(par_);
		}

		/** load problem */
		si[s]->loadProblem(mat, clbd, cubd,
				obj, ctype, rlbd, rubd, "BendersLowerBound");

		/** save memory */
		FREE_MEMORY
	}

	/** solve models */
	DSPdebugMessage("[%d] solving lower bouding problems\n", comm_rank_);
#ifdef USE_OMP
	omp_set_num_threads(parNumCores_);
#pragma omp parallel for schedule(dynamic)
#endif
	for (int s = 0; s < parProcIdxSize_; ++s)
	{
		/** solve */
		si[s]->solve();
	}

	/** parse status */
	DSPdebugMessage("[%d] obtaining lower bounds\n", comm_rank_);
	for (int s = 0; s < parProcIdxSize_ && doNext == true; ++s)
	{
		/** solution status */
		status_ = si[s]->getStatus();
		DSPdebugMessage("[%d] lower bound subproblem status %d\n", comm_rank_, status_);

		/** get solution */
		switch (status_)
		{
		case STO_STAT_OPTIMAL:
		case STO_STAT_LIM_ITERorTIME:
			DSPdebugMessage("[%d] lower bound %e\n", comm_rank_, si[s]->getPrimalBound());
			localLB += si[s]->getPrimalBound();
			break;
		case STO_STAT_DUAL_INFEASIBLE:
			/** need to use L-shaped method in this case */
			localLB = -COIN_DBL_MAX;
			doNext = false;
			break;
		default:
			doNext = false;
			break;
		}
	}

	DSPdebugMessage("[%d] lower bound %f\n", comm_rank_, localLB);

	/** collect lower bound */
	MPI_Reduce(&localLB, &lowerbound, 1, MPI_DOUBLE, MPI_SUM, 0, comm_);

	END_TRY_CATCH_RTN(FREE_MEMORY,STO_RTN_ERR)

	FREE_2D_PTR(parProcIdxSize_,si);

	return STO_RTN_OK;
#undef FREE_MEMORY
}

/** construct master problem */
STO_RTN_CODE TssBdMpi::constructMasterProblem(TssBdSub * tssbdsub, double lowerbound)
{
#define FREE_MEMORY       \
	FREE_PTR(mat)         \
	FREE_ARRAY_PTR(clbd)  \
	FREE_ARRAY_PTR(cubd)  \
	FREE_ARRAY_PTR(ctype) \
	FREE_ARRAY_PTR(obj)   \
	FREE_ARRAY_PTR(rlbd)  \
	FREE_ARRAY_PTR(rubd)  \
	FREE_ARRAY_PTR(auxind)  \
	FREE_ARRAY_PTR(auxcoef)

	assert(model_);

	if (comm_rank_ != 0)
		return STO_RTN_OK;

	if (naux_ <= 0 || !obj_aux_ || !clbd_aux_ || !cubd_aux_)
	{
		printf("Warning: Auxiliary column information is required.\n");
		return STO_RTN_ERR;
	}

	/** master problem */
	CoinPackedMatrix * mat = NULL;
	double * clbd   = NULL;
	double * cubd   = NULL;
	double * obj    = NULL;
	char *   ctype  = NULL;
	double * rlbd   = NULL;
	double * rubd   = NULL;
	/** for recourse lower bound */
	int * auxind     = NULL;
	double * auxcoef = NULL;

	BGN_TRY_CATCH

	int ncols = model_->getNumCols(0) + naugs_ * model_->getNumCols(1) + naux_;

	/** number of integer variables in the core */
	int nIntegers = model_->getNumCoreIntegers();

	/** allocate memory */
	auxind = new int [ncols];
	auxcoef = new double [ncols];

	/** decompose model */
	STO_RTN_CHECK_THROW(model_->decompose(
			naugs_, augs_, naux_, clbd_aux_, cubd_aux_, obj_aux_,
			mat, clbd, cubd, ctype, obj, rlbd, rubd),
			"decompose", "TssModel");

	/** convert column types */
	if (parRelaxIntegrality_[0])
	{
		for (int j = 0; j < model_->getNumCols(0); ++j)
		{
			if (ctype[j] != 'C')
				nIntegers--;
			ctype[j] = 'C';
		}
	}
	if (parRelaxIntegrality_[1] && naugs_ > 0)
	{
		for (int j = 0; j < model_->getNumCols(1); ++j)
		{
			if (ctype[model_->getNumCols(0) + j] != 'C')
				nIntegers--;
		}
		CoinFillN(ctype + model_->getNumCols(0), naugs_ * model_->getNumCols(1), 'C');
	}

	if (nIntegers > 0)
	{
		si_ = new SolverInterfaceScip(par_);
		si_->setPrintLevel(CoinMin(parLogLevel_ + 1, 5));
	}
	else
	{
		si_ = new SolverInterfaceSpx(par_);
		si_->setPrintLevel(CoinMax(parLogLevel_ - 1, 0));
	}

	/** load problem data */
	si_->loadProblem(mat, clbd, cubd, obj, ctype, rlbd, rubd, "BdMaster");

	/** add row for lower bound */
	for (int j = 0; j < ncols; ++j)
		auxind[j] = j;
	CoinCopyN(obj, ncols, auxcoef);
	si_->addRow(ncols, auxind, auxcoef, lowerbound, COIN_DBL_MAX);

	/** save memory */
	FREE_MEMORY

	END_TRY_CATCH_RTN(FREE_MEMORY,STO_RTN_ERR)

	return STO_RTN_OK;

#undef FREE_MEMORY
}

/** configure master */
STO_RTN_CODE TssBdMpi::configureMaster(TssBdSub * tssbdsub)
{
	if (comm_rank_ != 0)
		return STO_RTN_OK;

	BGN_TRY_CATCH

	/** use L-shaped method to find lower bound */
	if (model_->getNumCoreIntegers() == 0)
	{
		/** configure LP */
		STO_RTN_CHECK_THROW(configureSLP(), "configureSLP", "TssBdMpi");
	}
	else
	{
		/** configure MILP */
		STO_RTN_CHECK_THROW(configureSMILP(tssbdsub), "configureSMILP", "TssBdMpi");
	}

	END_TRY_CATCH_RTN(;,STO_RTN_ERR)

	return STO_RTN_OK;
}

/** solve Benders master */
STO_RTN_CODE TssBdMpi::runMaster(TssBdSub * tssbdsub)
{
	if (comm_rank_ != 0)
		return STO_RTN_OK;

	BGN_TRY_CATCH

	/** use L-shaped method to find lower bound */
	if (model_->getNumCoreIntegers() == 0)
	{
		/** solve Stochastic LP */
		STO_RTN_CHECK_THROW(solveSLP(tssbdsub, si_), "solveSLP", "TssBd");
	}
	else
	{
		/** solve Stochastic LP */
		//STO_RTN_CHECK_THROW(solveSLP(tssbdsub), "solveSLP", "TssBd");

		/** solve Stochastic MILP */
		STO_RTN_CHECK_THROW(solveSMILP(), "solveSMILP", "TssBd");
	}

	END_TRY_CATCH_RTN(;,STO_RTN_ERR)

	return STO_RTN_OK;
}

/** run workers */
STO_RTN_CODE TssBdMpi::runWorkers(TssBdSub * tssbdsub)
{
#define FREE_MEMORY \
	FREE_ARRAY_PTR(solution); \
	FREE_2D_ARRAY_PTR(model_->getNumScenarios(),cutval); \
	FREE_ARRAY_PTR(cutrhs);

	if (comm_rank_ == 0)
		return STO_RTN_OK;

	OsiCuts cuts, tempcuts;
	int message;
	int ncols;
	double * solution = NULL;
	double ** cutval = NULL;
	double * cutrhs = NULL;
	CoinPackedVector vec;

	BGN_TRY_CATCH

	ncols = model_->getNumCols(0) + naugs_ * model_->getNumCols(1) + naux_;
	solution = new double [ncols];
	cutval = new double * [model_->getNumScenarios()];
	cutrhs = new double [model_->getNumScenarios()];

	/** Wait for message from the master */
	MPI_Bcast(&message, 1, MPI_INT, 0, comm_);
	DSPdebugMessage("[%d]: Received message [%d]\n", comm_rank_, message);

	/** Parse the message */
	while (message == MASTER_NEEDS_CUTS)
	{
		/** Receive master solution */
		MPI_Bcast(solution, ncols, MPI_DOUBLE, 0, comm_);

		/** Generate cuts */
		tssbdsub->generateRawCuts(ncols, solution, cutval, cutrhs);
		for (int s = 0, ss = 0; s < model_->getNumScenarios(); ++s)
		{
			if (cutval[s] == NULL) continue;

			/** initialize vector */
			vec.clear();

			/** set it as sparse */
			for (int j = 0; j < ncols; ++j)
			{
				if (fabs(cutval[s][j]) > 1E-10)
					vec.insert(j, cutval[s][j]);
			}

			/** free memory */
			FREE_ARRAY_PTR(cutval[s]);

			if (fabs(cutrhs[s]) < 1E-10)
				cutrhs[s] = 0.0;

			OsiRowCut rc;
			rc.setRow(vec);
			rc.setUb(COIN_DBL_MAX); /** TODO: for minimization */
			rc.setLb(cutrhs[s]);

			//DSPdebug(rc.print());
			cuts.insert(rc);
		}
		DSPdebugMessage("[%d]: Found %d cuts\n", comm_rank_, cuts.sizeCuts());

		/** Send cuts to the master */
		MPIgatherOsiCuts(comm_, cuts, tempcuts);

		/** cleanup cuts */
		for (int i = 0; i < cuts.sizeCuts(); ++i)
		{
			OsiRowCut * rc = cuts.rowCutPtr(i);
			FREE_PTR(rc);
		}
		cuts.dumpCuts();

		/** Wait for message from the master */
		MPI_Bcast(&message, 1, MPI_INT, 0, comm_);
		DSPdebugMessage("[%d]: Received message [%d]\n", comm_rank_, message);
	}

	END_TRY_CATCH_RTN(FREE_MEMORY,STO_RTN_ERR)

	FREE_MEMORY

	return STO_RTN_OK;
#undef FREE_MEMORY
}

/** configure Phase 1 */
STO_RTN_CODE TssBdMpi::configureSLP()
{
	BGN_TRY_CATCH

	/** TODO: any configuration? */

	END_TRY_CATCH_RTN(;,STO_RTN_ERR)

	return STO_RTN_OK;
}

/** configure Phase 2 (e.g., Benders cut generator) */
STO_RTN_CODE TssBdMpi::configureSMILP(TssBdSub * tssbdsub)
{
	BGN_TRY_CATCH

	/** retrieve solver interface for SCIP */
	SolverInterfaceScip * SiScip = dynamic_cast<SolverInterfaceScip*>(si_);
	assert(SiScip);

	/** get SCIP pointer */
	SCIP * scip = SiScip->getSCIP();

	/** create constraint handler */
	SCIPconshdlrBendersMPI * conshdlr = new SCIPconshdlrBendersMPI(scip, parCutPriority_, comm_);
	conshdlr->assignTssBdSub(tssbdsub);
	conshdlr->initializeMpi(model_->getNumScenarios(), parProcIdxSize_, probability_);
	conshdlr->setOriginalVariables(SiScip->getNumCols(), SiScip->getSCIPvars());

	/** add constraint handler */
	SiScip->addConstraintHandler(conshdlr, true);

	/** set node limit */
	si_->setNodeLimit(parNodeLim_);

	/** set time limit */
	si_->setTimeLimit(time_remains_);

	/** set print level */
	si_->setPrintLevel(CoinMin(parLogLevel_ + 2, 5));

	END_TRY_CATCH_RTN(;,STO_RTN_ERR)

	return STO_RTN_OK;
}

/**
 * solve Phase 1
 *
 * TODO: Need more sophisticated cut management for efficiency.
 */
STO_RTN_CODE TssBdMpi::solveSLP(
		TssBdSub * tssbdsub,
		SolverInterface * si,
		int whereFrom,
		OsiCuts * outcuts)
{
	int iter = 0;
	int message = MASTER_NEEDS_CUTS;
	OsiCuts cuts, cutcollection;
	int ncuts = 0;

	double objval = -COIN_DBL_MAX;
	double effectiveness   = 1E-8;
	double effectivenessLB = 1E-8;

	double stime_cpu  = 0.0;
	double stime_wall = 0.0;

	BGN_TRY_CATCH

	/** initial solve */
	si->solve();

	int niters = 0;

	while (message == MASTER_NEEDS_CUTS)
	{
		/** solution status */
		if (si->getStatus() == STO_STAT_OPTIMAL)
		{
			/** check improvement of objective value */
			effectiveness = fabs(objval - si->getPrimalBound()) < 1E-10 ? effectiveness * 10 : effectivenessLB;

			/** optimal */
			objval = si->getPrimalBound();

			/** print */
			if (parLogLevel_)
			{
				niters += si->getIterationCount();
				printf("Iteration %4d: objective function: %+E iteration %d\n",
						iter++, objval, niters);
			}
#ifdef TSSBENDERS_DEBUG
			PRINT_ARRAY(si->getNumCols(), si->getColSolution());
#endif

			/** stop at iteration limit */
			if (iter >= parIterLim_)
			{
				status_ = STO_STAT_STOPPED_ITER;
				message = MASTER_STOPPED;
				break;
			}

			/** mark start times */
			stime_cpu  = CoinCpuTime();
			stime_wall = CoinGetTimeOfDay();

			/** Tell workers to generate cuts */
			MPI_Bcast(&message, 1, MPI_INT, 0, comm_);

			/** Send solutions to the workers */
			MPI_Bcast(const_cast<double*>(si->getSolution()), si->getNumCols(), MPI_DOUBLE, 0, comm_);

			/** generate cuts */
			tssbdsub->generateCuts(si->getNumCols(), si->getSolution(), &cuts);
			//cuts.printCuts();

			/** Collect cuts */
			MPIgatherOsiCuts(comm_, cuts, cutcollection);

			/** remove cuts */
			for (int i = 0; i < cuts.sizeCuts(); ++i)
			{
				OsiRowCut * rc = cuts.rowCutPtr(i);
				FREE_PTR(rc);
			}
			cuts.dumpCuts();

			/** calculate effectiveness */
			for (int i = 0; i < cutcollection.sizeCuts(); ++i)
			{
				OsiRowCut * rc = cutcollection.rowCutPtr(i);
				if (!rc) continue;

				double violated = rc->violated(si->getSolution());
				rc->setEffectiveness(violated);

				/** store cuts if requested */
				if (outcuts)
					outcuts->insert(*rc);
			}

			/** mark elapsed times */
			stat_.cut_generation_time_cpu_phase1_  += CoinCpuTime() - stime_cpu;
			stat_.cut_generation_time_wall_phase1_ += CoinGetTimeOfDay() - stime_wall;

			/** add cuts */
			int nCutsAdded = si->addCuts(cutcollection, effectiveness);

			/** remove cuts */
			for (int i = 0; i < cutcollection.sizeCuts(); ++i)
			{
				OsiRowCut * rc = cutcollection.rowCutPtr(i);
				FREE_PTR(rc);
			}
			cutcollection.dumpCuts();

			/** if no cut added */
			ncuts += nCutsAdded;
			if (nCutsAdded == 0)
			{
				status_ = STO_STAT_OPTIMAL;
				message = MASTER_STOPPED;
			}

			/** resolve master */
			si->solve();
		}
		else
		{
			status_ = si->getStatus();
			message = MASTER_STOPPED;
		}
	}

	/** Tell workers we are done */
	assert(message == MASTER_STOPPED);
	MPI_Bcast(&message, 1, MPI_INT, 0, comm_);

	/** assign solution if no integer variable */
	if (whereFrom == 0 &&
			(status_ == STO_STAT_OPTIMAL ||
			status_ == STO_STAT_STOPPED_ITER))
	{
		/** primal bound */
		if (status_ == STO_STAT_OPTIMAL)
			primalBound_ = si->getPrimalBound();
		else
			primalBound_ = COIN_DBL_MAX;

		/** dual bound */
		dualBound_ = si->getPrimalBound();
	}

	/** statistics */
	numIterations_ = iter;

	END_TRY_CATCH_RTN(;,STO_RTN_ERR)

	return STO_RTN_OK;
}

STO_RTN_CODE TssBdMpi::solveSMILP()
{
	BGN_TRY_CATCH

	/** solve */
	si_->solve();

	/** Tell workers we are done */
	int message = MASTER_STOPPED;
	MPI_Bcast(&message, 1, MPI_INT, 0, comm_);

	/** solution status */
	status_ = si_->getStatus();

	switch (status_)
	{
	case STO_STAT_OPTIMAL:
	case STO_STAT_LIM_ITERorTIME:
	case STO_STAT_STOPPED_GAP:
	case STO_STAT_STOPPED_NODE:
	case STO_STAT_STOPPED_TIME:
		{
			primalBound_ = si_->getPrimalBound();
			dualBound_ = si_->getDualBound();
			break;
		}
		break;
	default:
		printf("Solution status (%d).\n", status_);
		break;
	}

	/** statistics */
	numIterations_ = si_->getIterationCount();
	numNodes_ = si_->getNumNodes();

	END_TRY_CATCH_RTN(;,STO_RTN_ERR)

	return STO_RTN_OK;
}

/** set auxiliary variable data */
void TssBdMpi::setAuxColData(int size, double * obj, double * clbd, double * cubd)
{
	FREE_ARRAY_PTR(obj_aux_)
	FREE_ARRAY_PTR(clbd_aux_)
	FREE_ARRAY_PTR(cubd_aux_)

	naux_ = size;
	obj_aux_  = new double [naux_];
	clbd_aux_ = new double [naux_];
	cubd_aux_ = new double [naux_];

	CoinCopyN(obj, naux_, obj_aux_);
	CoinCopyN(clbd, naux_, clbd_aux_);
	CoinCopyN(cubd, naux_, cubd_aux_);
}

/** set auxiliary variable data */
void TssBdMpi::setAugScenarios(int size, int * indices)
{
	FREE_ARRAY_PTR(augs_)

	naugs_ = size;
	augs_  = new int [naugs_];

	CoinCopyN(indices, naugs_, augs_);
}

