/*
 * PSCGBBSolverSerial.h
 *
 *  Created on: 1 Dec 2017
 *      Author: Brian Dandurand
 */

#ifndef SRC_SOLVER_PSCGSOLVERSERIAL_H_
#define SRC_SOLVER_PSCGSOLVERSERIAL_H_

#include <DecSolver.h>
#include <TreeSearch/DspNodeSolution.h>
#include <TreeSearch/DspBranch.h>
#include <Solver/PSCG/PSCGNodeSolver.h>
#include <Solver/PSCG/PSCGModel.h>

class PSCGBBSolverSerial: public DecSolver {
public:

    /** default constructor */
	PSCGBBSolverSerial(
			DecModel *   model,  /**< model pointer */
			DspParams *  par,    /**< parameters */
			DspMessage * message /**< message pointer */);

	virtual PSCGBBSolverSerial* clone() const {return NULL;}

	virtual ~PSCGBBSolverSerial();

	/** initialize */
	virtual DSP_RTN_CODE init();

	/** solve */
	virtual DSP_RTN_CODE solve();

	/** finalize */
	virtual DSP_RTN_CODE finalize();

protected:

	PSCGNodeSolver* nodeSolver_;
	PSCG *pscgSolver_;
	PSCGModel * alps_; /**< Alps model pointer */

};

#endif /* SRC_SOLVER_DANTZIGWOLFE_DWSOLVERSERIAL_H_ */
