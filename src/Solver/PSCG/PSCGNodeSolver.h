//
// Created by Kibaek Kim on 8/27/16.
//

#ifndef DSP_PSCGNODESOLVER_H
#define DSP_PSCGNODESOLVER_H

/** Coin */
//#include "CoinWarmStartBasis.hpp"
/** Dsp */
#include "Solver/PSCG/DspBranchPSCG.h"
#include "Solver/DecSolver.h"
#include <PSCG.h>

/**
 * This creates the master problem data and solves the master problem.
 *
 * The master problem structure is
 *   lb^0 <= \sum_{j=1}^{r^1} H^1 x_j^1 lambda_j^1 + ... + \sum_{j=1}^{r^k} H^k x_j^k lambda_j^k <= ub^0
 *      1 <= \sum_{j=1}^{r^1}           lambda_j^1                                               <= 1
 *    ...
 *      1 <=                                               \sum_{j=1}^{r^k}           lambda_j^k <= 1
 *      0 <=                            lambda_j^1, ...                               lambda_j^k
 *
 * TODO: Branching may be performed on \sum_{j=1}^{r^k} lambda_j^k.
 */

/**
 * TODO: Another way is considering the master problem in the form
 *   lb^0 <= \sum_{j=1}^r (\sum_{i=1}^k H^i x_j^i) lambda_j <= ub^0
 *      1 <= \sum_{j=1}^r                          lambda_j <= 1
 *
 * But not implemented here.
 */
class PSCGNodeSolver : public DecSolver {
public:

    /** constructor with worker */
    PSCGNodeSolver(
			DecModel *   model,  /**< model pointer */
			DspParams *  par,    /**< parameters */
			DspMessage * message, /**< message pointer */
			PSCG* solver);

    /** copy constructor */
    PSCGNodeSolver(const PSCGNodeSolver& rhs);

#if 0

    /** copy operator */
    DwMaster& operator=(const DwMaster& rhs);
#endif

	virtual PSCGNodeSolver* clone() const {
		return new PSCGNodeSolver(*this);
	}
    /** default destructor */
    virtual ~PSCGNodeSolver();

    /** initialize */
    virtual DSP_RTN_CODE init();

    /** solve */
    virtual DSP_RTN_CODE solve();

    /** finalize */
    virtual DSP_RTN_CODE finalize() {return DSP_RTN_OK;}

    /** switch to phase 2 */
    DSP_RTN_CODE switchToPhase2();

	/** set branching objects */
	virtual void setBranchingObjects(const DspBranch* branchobj);

	/** set best primal solution */
	void setBestPrimalSolution(const double* solution);

	/** set primal solution */
	void setPrimalSolution(const double* solution);

    void setTCritParam(double val){tCritParam_=val;}
	
    void findNewBranchInfo(int &br_rank, int &br_scen, int &br_index, double &br_val, double &br_lbUp, double &br_ubUp, double &br_lbDn, double &br_ubDn); 
   
    DspBranchPSCG * generateCurrentBranchingInfo(); //returns a newly allocated DspBranchPSCG*
 
    int getMPIRank(){return pscg_->getMPIRank();}

protected:

#if 0
	/** Generate initial columns */
	virtual DSP_RTN_CODE initialColumns();

    /** This creates a master problem. */
	virtual DSP_RTN_CODE createProblem();

	/** initial solver setting */
	virtual DSP_RTN_CODE initialOsiSolver();

    /** solve phase 1 */
    virtual DSP_RTN_CODE solvePhase1();

    /** solve phase 2 */
    virtual DSP_RTN_CODE solvePhase2();

    /** guts of solve */
    virtual DSP_RTN_CODE gutsOfSolve();

    /** solver master */
    virtual DSP_RTN_CODE solveMaster();

    /** restore columns: adding all the columns back */
    virtual DSP_RTN_CODE restoreCols();

    /** reduce columns (e.g., reduced cost fixing) */
    virtual DSP_RTN_CODE reduceCols();

    /** generate columns */
    virtual DSP_RTN_CODE generateCols();

    /** calculate piA */
    virtual DSP_RTN_CODE calculatePiA(
			std::vector<double>& piA /**< [out] pi^T A */);

    /** Add columns */
    virtual DSP_RTN_CODE addCols(
    		const double* piA,                   /**< [in] pi^T A */
    		std::vector<int>& indices,           /**< [in] subproblem indices corresponding to cols*/
			std::vector<int>& statuses,          /**< [in] subproblem solution status */
			std::vector<double>& cxs,            /**< [in] solution times original objective coefficients */
			std::vector<double>& objs,           /**< [in] subproblem objective values */
    		std::vector<CoinPackedVector*>& sols /**< [in] subproblem solutions */);

    /** Calculate Lagrangian bound */
    virtual DSP_RTN_CODE getLagrangianBound(
			std::vector<double>& objs /**< [in] subproblem objective values */);

    /** update master */
    virtual DSP_RTN_CODE updateModel();

    /** termination test */
    virtual bool terminationTest();

#endif

#if 0
    /** Run heuristics */
    virtual DSP_RTN_CODE heuristics();

    /** pre-process for heuristics */
    virtual DSP_RTN_CODE preHeuristic(
    		double*& rlbd,    /**< [out] original row lower bounds */
    		double*& rubd,    /**< [out] original row lower bounds */
    		double*& primsol, /**< [out] original primal solution */
    		double& primobj,  /**< [out] original primal objective */
    		double& dualobj,  /**< [out] original dual objective */
    		int& status       /**< [out] original solution status */);

    /** post-process for heuristics */
    virtual DSP_RTN_CODE postHeuristic(
    		double*& rlbd,    /**< [out] original row lower bounds */
    		double*& rubd,    /**< [out] original row lower bounds */
    		double*& primsol, /**< [out] original primal solution */
    		double& primobj,  /**< [out] original primal objective */
    		double& dualobj,  /**< [out] original dual objective */
    		int& status       /**< [out] original solution status */);

    /** trivial heuristic */
    virtual DSP_RTN_CODE heuristicTrivial();

    /** FP-type heuristic */
    virtual DSP_RTN_CODE heuristicFp(int direction);

    /** Dive heuristic */
    virtual DSP_RTN_CODE heuristicDive();

    /** guts of Dive heuristic */
    virtual DSP_RTN_CODE gutsOfDive(
    		std::vector<CoinTriple<int,int,double> > branchList,
    		int depth);
#endif
    /** print iteration information */
    virtual void printIterInfo();

protected:

    bool useBarrier_;

    int phase_; /**< phase 1 or 2? */
    std::vector<int> auxcolindices_;

    PSCG* pscg_; /**< subproblem solver */
    double tCritParam_;

    const double feastol_ = 1.0e-5;

public:

    int ncols_orig_; /**< number of columns in the original master */
#if 0
    int ncols_start_; /**< index that generated columns start added. */

    int nrows_;        /**< number of rows */
    int nrows_orig_;   /**< number of rows in the original master */
    int nrows_conv_;   /**< number of rows representing the convexification constraints */
    int nrows_core_;   /**< nrows_orig_ + nrows_conv_ */
    int nrows_branch_; /**< number of rows representing integer columns in the original master */
    std::map<int,int> branch_row_to_col_; /**< maps each branching row to column in the original master */

    //std::vector<DwCol*> cols_generated_; /**< columns generated */

    /**@name original master problem data */
    CoinPackedMatrix* mat_orig_; /**< constraint matrix */
    std::vector<double> clbd_orig_;
    std::vector<double> cubd_orig_;
    std::vector<double> obj_orig_;
    std::vector<char> ctype_orig_;
    std::vector<double> rlbd_orig_;
    std::vector<double> rubd_orig_;
	std::vector<double> clbd_node_; /** current column lower bounds */
	std::vector<double> cubd_node_; /** current column upper bounds */

    int itercnt_;
    int ngenerated_;


    /**@name Time stamps */
    double t_start_; /**< solution start time */
    double t_total_; /**< total time */
    double t_master_; /**< master solution time */
    double t_colgen_; /**< column generation time */

protected:

    std::vector<int> status_subs_; /**< subproblem status */
#endif

};


#endif //DSP_DWMASTER_H
