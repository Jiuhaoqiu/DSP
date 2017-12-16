/*
 * PSCGModel.cpp
 *
 *  Created on: 27 Nov 2017
 *      Author: Brian Dandurand, Postdoctoral Appointee, MSC Division, Argonne National Laboratory
 */

//#define DSP_DEBUG

#include <PSCG/PSCGModel.h>
//#include <PSCG/PSCGHeuristic.h>
#include "Model/TssModel.h"
#include <AlpsKnowledgeBroker.h>

PSCGModel::PSCGModel(): DspModel(){}

PSCGModel::PSCGModel(DecSolver* solver): DspModel(solver){
	pscgSolver_ = dynamic_cast<PSCGNodeSolver*>(solver);
	primsol_.resize(pscgSolver_->ncols_orig_);
	mpiRank_=pscgSolver_->getMPIRank();
#if 0
	//heuristics_.push_back(new DwRounding("Rounding", *this));
	heuristics_.push_back(new DwSmip("Smip", *this));
#endif
}

PSCGModel::~PSCGModel() {
#if 0
	for (unsigned i = 0; i < heuristics_.size(); ++i)
		delete heuristics_[i];
#endif
}

DSP_RTN_CODE PSCGModel::solve() {
	if(pscgSolver_->getMPIRank()==0){
	    cout << "LB: " << getKnowledgeBroker()->getBestNode()->getQuality() << " and UB: " << getKnowledgeBroker()->getIncumbentValue() << endl;
	}
	BGN_TRY_CATCH
#if 1

	/** set best primal objective value */
	pscgSolver_->setBestPrimalObjective(bestprimobj_);

	/** solve node subproblem */
        int treeDepth=getKnowledgeBroker()->getTreeDepth();
        pscgSolver_->setTCritParam(pow(0.5,treeDepth));
	pscgSolver_->solve();

	status_ = pscgSolver_->getStatus();

	switch (status_) {
	case DSP_STAT_OPTIMAL:
	case DSP_STAT_FEASIBLE:
	case DSP_STAT_LIM_ITERorTIME: {

		bestprimobj_ = pscgSolver_->getBestPrimalObjective();
		primobj_=bestprimobj_;
		if(primobj_ >= 1.0e+20) primobj_= 9.99e+19;
		bestdualobj_ = pscgSolver_->getBestDualObjective();
		dualobj_ = bestdualobj_; 

		if (primobj_ < 1.0e+20) {
			/** parse solution */
#if 0
			int cpos = 0;

			std::fill(primsol_.begin(), primsol_.begin() + master_->ncols_orig_, 0.0);
			for (auto it = master_->cols_generated_.begin(); it != master_->cols_generated_.end(); it++) {
				if ((*it)->active_) {
					if (fabs(master_->getPrimalSolution()[cpos]) > 1.0-10) {
						for (int i = 0; i < (*it)->x_.getNumElements(); ++i) {
							if ((*it)->x_.getIndices()[i] < master_->ncols_orig_)
								primsol_[(*it)->x_.getIndices()[i]] += (*it)->x_.getElements()[i] * master_->getPrimalSolution()[cpos];
						}
					}
					cpos++;
				}
			}
			//DspMessage::printArray(cpos, master_->getPrimalSolution());

			/** calculate infeasibility */
			infeasibility_ = 0.0;
			for (int j = 0; j < master_->ncols_orig_; ++j)
				if (master_->ctype_orig_[j] != 'C') {
					infeasibility_ += fabs(primsol_[j] - floor(primsol_[j] + 0.5));
				}
			printf("Infeasibility: %+e\n", infeasibility_);

			for (int j = 0; j < master_->ncols_orig_; ++j) {
				double viol = std::max(master_->clbd_node_[j] - primsol_[j], primsol_[j] - master_->cubd_node_[j]);
				if (viol > 1.0e-6) {
					printf("Violated variable at %d by %e (%+e <= %+e <= %+e)\n", j, viol,
							master_->clbd_node_[j], primsol_[j], master_->cubd_node_[j]);
				}
			}

			/** run heuristics */
			if (par_->getBoolParam("DW/HEURISTICS") && infeasibility_ > 1.0e-6) {
				/** FIXME */
				bestprimobj_ = COIN_DBL_MAX;
				for (auto it = heuristics_.begin(); it != heuristics_.end(); it++) {
					solver_->getMessagePtr()->print(1, "Running [%s] heuristic:\n", (*it)->name());
					int found = (*it)->solution(bestprimobj_, bestprimsol_);
					//printf("found %d bestprimobj %+e\n", found, bestprimobj_);
				}
			}
#endif
		}

		break;
	}
	default:
		break;
	}

#endif
	END_TRY_CATCH_RTN(;,DSP_RTN_ERR)

	return DSP_RTN_OK;
}

bool PSCGModel::chooseBranchingObjects(
		DspBranch*& branchingUpBase, /**< [out] branching-up object */
		DspBranch*& branchingDnBase  /**< [out] branching-down object */) {
	bool branched=false;
	DspBranchPSCG *branchingUp = dynamic_cast<DspBranchPSCG*>(branchingUpBase);
	DspBranchPSCG *branchingDn = dynamic_cast<DspBranchPSCG*>(branchingDnBase);
	int br_rank=-1, br_scen=-1, br_index=-1;
	double br_val=0.0, br_lbUp=0.0, br_ubUp=0.0, br_lbDn=0.0, br_ubDn=0.0;
	BGN_TRY_CATCH
	FREE_PTR(branchingUp)
	FREE_PTR(branchingDn)
	//branchingUp = new DspBranchPSCG(*(dynamic_cast<const DspBranchPSCG*>(currentBranchObj_)) ); //TODO
	//branchingDn = new DspBranchPSCG(*(dynamic_cast<const DspBranchPSCG*>(currentBranchObj_)) ); //TODO

	pscgSolver_->findNewBranchInfo(br_rank, br_scen, br_index, br_val, br_lbUp, br_ubUp, br_lbDn, br_ubDn); 
        if(br_index >=0){
	    branchingUp = pscgSolver_->generateCurrentBranchingInfo();
	    branchingDn = pscgSolver_->generateCurrentBranchingInfo();
#if 0
	if(pscgSolver_->getMPIRank()==0){
	      std::cout << br_lbUp << "," << br_ubUp << std::endl;
	}
#endif
	    branchingUp->addbranch(br_rank, br_scen, br_index, br_lbUp, br_ubUp);
#if 0
	if(pscgSolver_->getMPIRank()==0){
	      std::cout << br_lbDn << "," << br_ubDn << std::endl;
	}
#endif
	    branchingDn->addbranch(br_rank, br_scen, br_index, br_lbDn, br_ubDn); 
	    branchingUp->bestBound_ = pscgSolver_->getBestDualObjective();
	    branchingDn->bestBound_ = pscgSolver_->getBestDualObjective();
		
	    pscgSolver_->writeOmega(branchingUp->dualsol_);
	    pscgSolver_->writeOmega(branchingDn->dualsol_);
	    branchingUpBase=branchingUp;
	    branchingDnBase=branchingDn;

	    if(pscgSolver_->getMPIRank()==0){
	      std::cout << "Done choosing two branching objects" << endl;
	      branchingUp->printBranchBounds();
	      branchingDn->printBranchBounds();
	    }
	    branched=true;
	}
	else{
	    branchingUpBase=NULL;
	    branchingDnBase=NULL;
	    branched=false;
	}
#if 0

	/** cleanup */

	if (solver_->getModelPtr()->isStochastic()) {
		/** two-stage stochastic model */
		tss = dynamic_cast<TssModel*>(solver_->getModelPtr());
		ncols_first_stage = tss->getNumScenarios() * tss->getNumCols(0);
	}
	DSPdebugMessage("ncols_first_stage %d\n", ncols_first_stage);

	findPhase = 0;
	while (findPhase < 2 && branchingIndex < 0) {
		/** most fractional value */
		for (int j = 0; j < master_->ncols_orig_; ++j) {
			if (findPhase == 0 && j > ncols_first_stage)
				break;
			if (master_->ctype_orig_[j] == 'C') continue;
			dist = fabs(primsol_[j] - floor(primsol_[j] + 0.5));
			if (dist > maxdist) {
				maxdist = dist;
				branchingIndex = j;
				branchingValue = primsol_[j];
			}
		}

		/** for the first pass of smip, look through expected first-stage integer variable values */
		if (ncols_first_stage > 0 && findPhase == 0 && branchingIndex < 0) {
			maxdist = 1.0e-8;
			for (int j = 0; j < tss->getNumCols(0); ++j) {
				if (master_->ctype_orig_[j] == 'C') continue;
				double expval = 0.0;
				for (int s = 0; s < tss->getNumScenarios(); ++s)
					expval += primsol_[tss->getNumCols(0) * s + j] / tss->getNumScenarios();
				dist = fabs(expval - floor(expval + 0.5));
				if (dist > maxdist) {
					maxdist = dist;
					branchingIndex = j;
					branchingValue = primsol_[j];
				}
			}
		}

		findPhase++;
	}

	if (branchingIndex > -1) {

		branched = true;

		/** get branching index in first stage */
		if (branchingIndex < ncols_first_stage)
			branchingFirstStage = branchingIndex % tss->getNumCols(0);

		/** creating branching objects */
		branchingUp = new DspBranch();
		branchingDn = new DspBranch();
		for (int j = 0; j < master_->ncols_orig_; ++j) {
			if (master_->ctype_orig_[j] == 'C') continue;
			/** NOTE: branching on all the first-stage variables */
			if (branchingIndex == j || branchingFirstStage == j % tss->getNumCols(0)) {
				DSPdebugMessage("Creating branch objects on column %d (value %e).\n", j, branchingValue);
				branchingUp->push_back(j, ceil(branchingValue), master_->cubd_node_[j]);
				branchingDn->push_back(j, master_->clbd_node_[j], floor(branchingValue));
			} else if (master_->clbd_node_[j] > master_->clbd_orig_[j] || master_->cubd_node_[j] < master_->cubd_orig_[j]) {
				/** store any bound changes made in parent nodes */
				branchingUp->push_back(j, master_->clbd_node_[j], master_->cubd_node_[j]);
				branchingDn->push_back(j, master_->clbd_node_[j], master_->cubd_node_[j]);
			}
		}
		branchingUp->bestBound_ = master_->getBestDualObjective();
		branchingDn->bestBound_ = master_->getBestDualObjective();
		branchingUp->dualsol_.assign(master_->getBestDualSolution(), master_->getBestDualSolution() + master_->nrows_);
		branchingDn->dualsol_.assign(master_->getBestDualSolution(), master_->getBestDualSolution() + master_->nrows_);
	} else {
		DSPdebugMessage("No branch object is found.\n");
	}

#endif
	END_TRY_CATCH_RTN(;,false)

	return branched;
}

