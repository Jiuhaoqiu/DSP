/*
 * PSCGModel.h
 *
 *  Created on: 27 Nov 2017
 *      Author: Brian Dandurand, Postdoctoral Appointee, MSC Division, Argonne National Laboratory
 */

#ifndef SRC_SOLVER_PSCG_PSCGMODEL_H_
#define SRC_SOLVER_PSCG_PSCGMODEL_H_

#include <TreeSearch/DspModel.h>
//#include <PSCG.h>
#include <Solver/PSCG/PSCGNodeSolver.h>

class PSCGModel: public DspModel {
public:
	/** default constructor */
	PSCGModel();

	/** default constructor with solver */
	PSCGModel(DecSolver* solver);

	/** default destructor */
	virtual ~PSCGModel();

	/** solve model */
    virtual DSP_RTN_CODE solve();

    virtual bool chooseBranchingObjects(
    			DspBranch*& branchingUp, /**< [out] branching-up object */
    			DspBranch*& branchingDn  /**< [out] branching-down object */);

private:

    PSCGNodeSolver* pscgSolver_;
    //double infeasibility_;
};

#endif /* SRC_SOLVER_PSCG_PSCGMODEL_H_ */
