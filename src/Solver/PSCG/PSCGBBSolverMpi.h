/*
 * DwSolverMpi.h
 *
 *  Created on: Dec 5, 2016
 *      Author: kibaekkim
 */

#ifndef SRC_SOLVER_PSCGSOLVERMPI_H_
#define SRC_SOLVER_PSCGSOLVERMPI_H_

#include "mpi.h"
#include <PSCG/PSCGBBSolverSerial.h>

class PSCGBBSolverMpi: public PSCGBBSolverSerial {
public:

    /** default constructor */
	PSCGBBSolverMpi(
			DecModel*   model,   /**< model pointer */
			DspParams*  par,     /**< parameters */
			DspMessage* message, /**< message pointer */
			MPI_Comm    comm     /**< MPI communicator */);

	virtual ~PSCGBBSolverMpi();

	/** initialize */
	virtual DSP_RTN_CODE init();

	/** solve */
	virtual DSP_RTN_CODE solve();

	/** finalize */
	virtual DSP_RTN_CODE finalize();

protected:

	MPI_Comm comm_;
	int comm_size_;
	int comm_rank_;
};

#endif /* SRC_SOLVER_PSCGSOLVERMPI_H_ */
