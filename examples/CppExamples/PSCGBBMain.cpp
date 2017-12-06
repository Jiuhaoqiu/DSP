/*
Finds the optimal solution to the Lagrangian dual of a two-stage stochastic
integer program using a Frank-Wolfe-based Method of Multipliers approach.
*/

#include "PSCG/PSCGModel.h"
#include "PSCG/PSCGBBSolverMpi.h"
//#include "CPLEXsolverSCG.h"
//#include "PSCGModelScen.h"
//#include "Stopwatch.h"
#include "DecTssModel.h"
//#include "DspNodeDesc.h"
#include "AlpsKnowledgeBrokerSerial.h"
#include "DspApiEnv.h"
#include "DspCInterface.h"

//#define OUTER_LOOP_TERMINATION 1e-10
//#define TIME_TYPES 2
//#define DEFAULT_MAX_OUTER_LOOP 20 


using namespace std;


int main(int argc, char **argv) {
#if 1
	//******************Wall Timing Setup****************

	//Start timing. More refined timing is possible for debugging.
#if 0
	Stopwatch totalTimer;
   	Stopwatch interpTimer;
   	Stopwatch updateTimer;
   	double totalTimeQP [TIME_TYPES] = {0,0};
   	double totalTimeMIP [TIME_TYPES] = {0,0};

   	totalTimer.start();
   	double totalTimeAllStep [TIME_TYPES] = {0,0};
#endif

	//******************MPI Setup**********************
   	//identifying whether the code is going to run in  parallel or not
	MPI_Init(&argc, &argv);
	int mpiSize=1;
	int mpiRank=0;
	MPI_Comm_rank(MPI_COMM_WORLD, &mpiRank);
        MPI_Comm_size(MPI_COMM_WORLD, &mpiSize);

	
#if 1
	DspApiEnv* env = createEnv();
	
	DspParams par;
	DspMessage message(0);
	//par.readParameters(argc, argv);
	//par.setMPIParams(mpiSize,mpiRank);
	
	if(argc<2){ 
	    cerr << "Need input file path/name" << endl;
	    return 1;
	}
	DecTssModel *smps_model = new DecTssModel();
	smps_model->readSmps(argv[1]);
	env->model_=smps_model;


	int n1 = dynamic_cast<DecTssModel*>(getModelPtr(env))->getNumCols(0);
	int n2 = dynamic_cast<DecTssModel*>(getModelPtr(env))->getNumCols(1);
cout << "Number of vars: " << n1 << " + " << n2 << endl;
	int nS = dynamic_cast<DecTssModel*>(getModelPtr(env))->getNumScenarios();
cout << "Number of scenarios: " << nS << endl;
	//setNumberOfScenarios(env,nS);
	 n1 = dynamic_cast<DecTssModel*>(getModelPtr(env))->getNumCols(0);
	 n2 = dynamic_cast<DecTssModel*>(getModelPtr(env))->getNumCols(1);
cout << "Number of vars: " << n1 << " + " << n2 << endl;
	int *proc_idx = new int[nS];
	int proc_idx_size=0;
	for(int ii=mpiRank; ii<nS; ii+=mpiSize){ 
	    proc_idx[proc_idx_size]=ii;
	    proc_idx_size++;
	}
	setIntPtrParam(env, "ARR_PROC_IDX", proc_idx_size, proc_idx);
	delete [] proc_idx;
	solvePSCGMpi(env, MPI_COMM_WORLD);


#endif
	MPI_Finalize();
	

#endif
	return 0;
}

