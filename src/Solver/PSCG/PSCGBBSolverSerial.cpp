/*
 * PSCGSolverSerial.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: kibaekkim
 */

#include "AlpsKnowledgeBrokerSerial.h"
#include <Solver/PSCG/PSCGBBSolverSerial.h>
#include <Solver/PSCG/PSCGNodeSolver.h>
#include <PSCG.h>

PSCGBBSolverSerial::PSCGBBSolverSerial(
		DecModel *   model,  /**< model pointer */
		DspParams *  par,    /**< parameters */
		DspMessage * message /**< message pointer */):
DecSolver(model, par, message), alps_(NULL), pscgSolver_(NULL), nodeSolver_(NULL){}

PSCGBBSolverSerial::~PSCGBBSolverSerial() {
	FREE_PTR(alps_);
}

DSP_RTN_CODE PSCGBBSolverSerial::init() {
	BGN_TRY_CATCH

	/** create PSCG solver */
	pscgSolver_ = new PSCG(*(dynamic_cast<DecTssModel*>(model_)));

	/** create node solver */
	nodeSolver_ = new PSCGNodeSolver(model_,par_,message_,pscgSolver_);

#if 0
	/** initialize master */
	DSP_RTN_CHECK_THROW(master_->init());

	/** create an Alps model */
	alps_ = new PSCGModel(master_);

	/** parameter setting */
	DspParams* par = alps_->getSolver()->getParPtr();
	alps_->AlpsPar()->setEntry(AlpsParams::searchStrategy, par->getIntParam("ALPS/SEARCH_STRATEGY"));
	alps_->AlpsPar()->setEntry(AlpsParams::nodeLogInterval, par->getIntParam("ALPS/NODE_LOG_INTERVAL"));
	alps_->AlpsPar()->setEntry(AlpsParams::nodeLimit, par->getIntParam("ALPS/NODE_LIM"));
	alps_->AlpsPar()->setEntry(AlpsParams::timeLimit, par->getDblParam("ALPS/TIME_LIM"));
	alps_->AlpsPar()->setEntry(AlpsParams::clockType, AlpsClockTypeWallClock);
#endif

	END_TRY_CATCH_RTN(;,DSP_RTN_ERR)
	return DSP_RTN_OK;
}

DSP_RTN_CODE PSCGBBSolverSerial::solve() {
#if 0
	BGN_TRY_CATCH

	/** solve */
	AlpsKnowledgeBrokerSerial alpsBroker(0, NULL, *alps_);
    alpsBroker.search(alps_);

//    DspNodeSolution* solution = dynamic_cast<DspNodeSolution*>(alpsBroker.getBestKnowledge(AlpsKnowledgeTypeSolution).first);
//    solution->print(std::cout);

	END_TRY_CATCH_RTN(;,DSP_RTN_ERR)
#endif
	return DSP_RTN_OK;
}

DSP_RTN_CODE PSCGBBSolverSerial::finalize() {
#if 0
	BGN_TRY_CATCH

	/** finalize master */
	DSP_RTN_CHECK_THROW(master_->finalize());

	END_TRY_CATCH_RTN(;,DSP_RTN_ERR)
#endif
	return DSP_RTN_OK;
}

void PSCGBBSolverSerial::setBranchingObjects(const DspBranch* branchobj){;}

