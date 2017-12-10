//
// Created by Kibaek Kim on 8/27/16.
//

//#define DSP_DEBUG

#include "ilcplex/cplex.h"
/** Coin */
#include "OsiCpxSolverInterface.hpp"
#include "CoinUtility.hpp"
/** Dsp */
#include "Model/TssModel.h"
#include "SolverInterface/OoqpEps.h"
#include "Solver/PSCG/PSCGNodeSolver.h"
#include "Solver/PSCG/DspBranchPSCG.h"
#include "Utility/DspUtility.h"

#if 1
PSCGNodeSolver::PSCGNodeSolver(
			DecModel *   model,  /**< model pointer */
			DspParams *  par,    /**< parameters */
			DspMessage * message, /**< message pointer */
			PSCG* pscg):
DecSolver(model,par,message),pscg_(pscg)
{
}
#endif

PSCGNodeSolver::PSCGNodeSolver(const PSCGNodeSolver& rhs):
DecSolver(rhs),
pscg_(rhs.pscg_){
}

/** copy operator */
#if 0
PSCGNodeSolver& PSCGNodeSolver::operator=(const PSCGNodeSolver& rhs) {
	DecSolver::operator=(rhs);
	useBarrier_ = rhs.useBarrier_;
	phase_ = rhs.phase_;
	auxcolindices_ = rhs.auxcolindices_;
	worker_ = rhs.worker_;
	ncols_orig_ = rhs.ncols_orig_;
	ncols_start_ = rhs.ncols_start_;
	nrows_ = rhs.nrows_;
	nrows_orig_ = rhs.nrows_orig_;
	nrows_conv_ = rhs.nrows_conv_;
	nrows_core_ = rhs.nrows_core_;
	nrows_branch_ = rhs.nrows_branch_;
	branch_row_to_col_ = rhs.branch_row_to_col_;
	clbd_orig_ = rhs.clbd_orig_;
	cubd_orig_ = rhs.cubd_orig_;
	obj_orig_ = rhs.obj_orig_;
	ctype_orig_ = rhs.ctype_orig_;
	rlbd_orig_ = rhs.rlbd_orig_;
	rubd_orig_ = rhs.rubd_orig_;
	clbd_node_ = rhs.clbd_node_;
	cubd_node_ = rhs.cubd_node_;
	itercnt_ = rhs.itercnt_;
	ngenerated_ = rhs.ngenerated_;
	t_start_ = rhs.t_start_;
	t_total_ = rhs.t_total_;
	t_master_ = rhs.t_master_;
	t_colgen_ = rhs.t_colgen_;
	status_subs_ = rhs.status_subs_;
	mat_orig_ = new CoinPackedMatrix(*(rhs.mat_orig_));
	for (auto it = rhs.cols_generated_.begin(); it != rhs.cols_generated_.end(); it++)
		cols_generated_.push_back(new DwCol(**it));
	return *this;
}
#endif

PSCGNodeSolver::~PSCGNodeSolver() {
#if 0
	FREE_PTR(mat_orig_);
	for (unsigned i = 0; i < cols_generated_.size(); ++i)
		FREE_PTR(cols_generated_[i]);
#endif
}

DSP_RTN_CODE PSCGNodeSolver::init() {
#if 0
//Nothing to do here. This is all done in the constructor of PSCG
#endif
	return DSP_RTN_OK;
#undef FREE_MEMORY
}


DSP_RTN_CODE PSCGNodeSolver::solve() {
	BGN_TRY_CATCH
	pscg_->setMaxNoSteps(40);
	pscg_->computeBound();
	if(pscg_->statusIsFeasible()){ 
	    status_=DSP_STAT_FEASIBLE;
	    dualobj_=pscg_->getLagrBound();
	    bestdualobj_=dualobj_;
	    primobj_=pscg_->getPrimalObjVal();
	}
#if 0
	if(dualobj_ > bestdualobj_){
	    bestdualobj_ = dualobj_;
	}
#endif
	if(primobj_ < bestprimobj_){
	    bestprimobj_ = primobj_;
	}
cout << "After processing this node: LB: " << bestdualobj_ << " and UB: " << bestprimobj_ << endl;
#if 0

	itercnt_ = 0;
	t_start_ = CoinGetTimeOfDay();
	t_total_ = 0.0;
	t_master_ = 0.0;
	t_colgen_ = 0.0;

	if (cols_generated_.size() == 0) {
		/** generate initial columns */
		DSP_RTN_CHECK_RTN_CODE(initialColumns());
		bestdualobj_ = std::max(bestdualobj_, dualobj_);

		DSP_RTN_CHECK_RTN_CODE(solvePhase1());
	}

	if (phase_ == 2) {
		DSP_RTN_CHECK_RTN_CODE(solvePhase2());
		if (status_ == DSP_STAT_PRIM_INFEASIBLE) {
			DSPdebugMessage("Converting to Phase 1.\n");
			DSP_RTN_CHECK_RTN_CODE(solvePhase1());
		}
	} else
		DSP_RTN_CHECK_RTN_CODE(solvePhase1());

	if (phase_ == 1) {
		if (status_ == DSP_STAT_FEASIBLE || status_ == DSP_STAT_OPTIMAL) {
			if (primobj_ > feastol_)
				status_ = DSP_STAT_PRIM_INFEASIBLE;
			else {
				DSPdebugMessage("Converting to Phase 2.\n");
				DSP_RTN_CHECK_RTN_CODE(solvePhase2());
			}
		}
	}

	/** switch to phase 2 */
	DSP_RTN_CHECK_RTN_CODE(switchToPhase2());
#endif

	END_TRY_CATCH_RTN(;,DSP_RTN_ERR)

	return DSP_RTN_OK;
}

#if 0
DSP_RTN_CODE PSCGNodeSolver::heuristics() {
	return DSP_RTN_OK;
}
#endif
void PSCGNodeSolver::setBestPrimalSolution(const double* solution) {
	bestprimsol_.assign(solution, solution+ncols_orig_);
}

void PSCGNodeSolver::setPrimalSolution(const double* solution) {
	primsol_.assign(solution, solution+ncols_orig_);
}

void PSCGNodeSolver::findNewBranchInfo(int &br_rank, int &br_scen, int &br_index, double &br_lb, double &br_val, double &br_ub){
    BranchingVarInfo brInfo = pscg_->findBranchingIndex();
    br_rank = brInfo.rank;
    br_scen = brInfo.scen;
    br_index = brInfo.index;
    br_val = brInfo.brVal;
    br_lb = brInfo.brLB;
    br_ub = brInfo.brUB;
} 

DspBranchPSCG * PSCGNodeSolver::generateCurrentBranchingInfo(){ //returns a newly allocated DspBranchPSCG*
    DspBranchPSCG *retval = new DspBranchPSCG();
    int nBounds = pscg_->branchingBDs_.inds.size();
    for(int ii=0; ii<nBounds; ii++){
	retval->addbranch(pscg_->branchingBDs_.ranks[ii],pscg_->branchingBDs_.sps[ii], pscg_->branchingBDs_.inds[ii],pscg_->branchingBDs_.lbs[ii],pscg_->branchingBDs_.ubs[ii]);
    }
    return retval;
}

void PSCGNodeSolver::setBranchingObjects(const DspBranch* branchobj) {
	/** shouldn't be null */
	assert(branchobj != NULL);
	const DspBranchPSCG *br = dynamic_cast<const DspBranchPSCG*>(branchobj);
	int nBranchObjs = br->index_.size();
	pscg_->restoreOriginalVarBounds();
	for(int ii=0; ii<nBranchObjs; ii++){
	    pscg_->addBranchVarBd(br->mpiRanks_[ii],br->spIndices_[ii],br->index_[ii],br->lb_[ii],br->ub_[ii]);
	}
	//read omega into pscg_
	//set best Lagr bound into pscg_
#if 0
	std::vector<int> delrows;
	std::vector<int> delcols;

	/** adding columns */
	std::vector<int> col_inds;
	std::vector<double> col_elems;

	BGN_TRY_CATCH

	/** remove all the branching rows */
	if (nrows_branch_ > 0) {
		delrows.reserve(nrows_branch_);
		for (int i = nrows_core_; i < nrows_; ++i)
			delrows.push_back(i);
		si_->deleteRows(nrows_branch_, &delrows[0]);
		DSPdebugMessage("Deleted %d rows in the master.\n", nrows_branch_);
		nrows_branch_ = 0;
	}

	/** remove all columns */
	int ndelcols = si_->getNumCols();
	delcols.reserve(ndelcols);
	for (int j = 0; j < ndelcols; ++j)
		delcols.push_back(j);
	si_->deleteCols(ndelcols, &delcols[0]);
	DSPdebugMessage("Deleted %d columns in the master.\n", ndelcols);

	/** count nrows_branch_ */
	for (unsigned j = 0; j < branchobj->index_.size(); ++j) {
		if (branchobj->lb_[j] > clbd_orig_[branchobj->index_[j]]) {
			branch_row_to_col_[nrows_core_ + nrows_branch_] = branchobj->index_[j];
			si_->addRow(0, NULL, NULL, branchobj->lb_[j], COIN_DBL_MAX);
			nrows_branch_++;
		}
		if (branchobj->ub_[j] < cubd_orig_[branchobj->index_[j]]) {
			branch_row_to_col_[nrows_core_ + nrows_branch_] = branchobj->index_[j];
			si_->addRow(0, NULL, NULL, -COIN_DBL_MAX, branchobj->ub_[j]);
			nrows_branch_++;
		}
	}

	/** update number of rows */
	nrows_ = nrows_core_ + nrows_branch_;
	DSPdebugMessage("nrows_ %d nrows_core_ %d nrows_branch_ %d\n", nrows_, nrows_core_, nrows_branch_);

	/** reserve vector sizes */
	col_inds.reserve(nrows_);
	col_elems.reserve(nrows_);

	/** add branching rows */
	for (auto it = cols_generated_.begin(); it != cols_generated_.end(); it++) {
		if ((*it)->active_) {
			/** create a column for core rows */
			col_inds.clear();
			col_elems.clear();
			for (int i = 0; i < (*it)->col_.getNumElements(); ++i) {
				if ((*it)->col_.getIndices()[i] < nrows_core_) {
					col_inds.push_back((*it)->col_.getIndices()[i]);
					col_elems.push_back((*it)->col_.getElements()[i]);
				}
			}

			/** append column elements for the branching rows */
			for (unsigned j = 0, i = 0; j < branchobj->index_.size(); ++j) {
				int sparse_index = (*it)->x_.findIndex(branchobj->index_[j]);
				double val = 0.0;
				if (sparse_index > -1)
					val = (*it)->x_.getElements()[sparse_index];
				if (branchobj->lb_[j] > clbd_orig_[branchobj->index_[j]]) {
					if (fabs(val) > 1.0e-10) {
						col_inds.push_back(nrows_core_+i);
						col_elems.push_back(val);
					}
					i++;
				}
				if (branchobj->ub_[j] < cubd_orig_[branchobj->index_[j]]) {
					if (fabs(val) > 1.0e-10) {
						col_inds.push_back(nrows_core_+i);
						col_elems.push_back(val);
					}
					i++;
				}
			}

			/** assign the core-row column */
			(*it)->col_.setVector(col_inds.size(), &col_inds[0], &col_elems[0]);

			/** add column */
			si_->addCol((*it)->col_, 0.0, COIN_DBL_MAX, (*it)->obj_);
		}
	}
	DSPdebugMessage("Appended dynamic columns in the master (%d / %u cols).\n", si_->getNumCols(), cols_generated_.size());

	/** restore column bounds */
	clbd_node_ = clbd_orig_;
	cubd_node_ = cubd_orig_;

	/** update column bounds at the current node */
	for (unsigned j = 0, irow = nrows_core_; j < branchobj->index_.size(); ++j) {
		clbd_node_[branchobj->index_[j]] = branchobj->lb_[j];
		cubd_node_[branchobj->index_[j]] = branchobj->ub_[j];
#ifdef DSP_DEBUG
		printf("Branch Obj: index %d lb %e ub %e\n", branchobj->index_[j], branchobj->lb_[j], branchobj->ub_[j]);
		CoinShallowPackedVector row = si_->getMatrixByRow()->getVector(irow);
		for (int i = 0, j = 0; i < row.getNumElements(); ++i) {
			if (j > 0 && j % 5 == 0) printf("\n");
			printf("  [%6d] %+e", row.getIndices()[i], row.getElements()[i]);
			j++;
		}
		printf("\n");
		irow++;
#endif
	}

	/** apply column bounds */
	std::vector<int> ncols_inds(ncols_orig_);
	CoinIotaN(&ncols_inds[0], ncols_orig_, 0);
	worker_->setColBounds(ncols_orig_, &ncols_inds[0], &clbd_node_[0], &cubd_node_[0]);

	/** set known best bound */
	bestdualobj_ = COIN_DBL_MAX;

	END_TRY_CATCH(;)
#endif
}

void PSCGNodeSolver::printIterInfo() {
#if 0
	message_->print(2, "[Phase %d] Iteration %3d: Master objective %e, ", phase_, itercnt_, primobj_);
	if (phase_ == 2) {
		if (bestdualobj_ > -1.0e+50)
			message_->print(2, "Lb %e (gap %.2f %%), ", bestdualobj_, relgap_*100);
		else
			message_->print(2, "Lb -Inf, ");
	}
	message_->print(2, "nrows %d, ncols %d, ", si_->getNumRows(), si_->getNumCols());
	if (!useBarrier_)
		message_->print(2, "itercnt %d, ", si_->getIterationCount());
	message_->print(2, "timing (total %.2f, master %.2f, gencols %.2f), statue %d\n",
			t_total_, t_master_, t_colgen_, status_);
#endif
}

