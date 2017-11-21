/*
Finds the optimal solution to the Lagrangian dual of a two-stage stochastic
integer program using a Frank-Wolfe-based Method of Multipliers approach.
*/

#include "DantzigWolfe/DwModel.h"
#include "DantzigWolfe/DwSolverMpi.h"
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
	
	setNumberOfScenarios(env,500);
	TssModel *smps_model = getTssModel(env);
	if(argc<2){ 
	    cerr << "Need input file path/name" << endl;
	    return 1;
	}
	readSmps(env, argv[1]);
	int *proc_idx = new int[500];
	int proc_idx_size=0;
	for(int ii=mpiRank; ii<500; ii+=mpiSize){ 
	    proc_idx[proc_idx_size]=ii;
	    proc_idx_size++;
	}
	setIntPtrParam(env, "ARR_PROC_IDX", proc_idx_size, proc_idx);
	delete [] proc_idx;
	solveDwMpi(env, MPI_COMM_WORLD);
	//solveDw(env);
	//DwSolverMpi solver(&smps_model, &par, &message, MPI_COMM_WORLD);
	//solver.init();
	//solver.solve();
	//solver.finalize();
	//DwModel dwmodel(&solver);
	//double *z = new double[model.n1];
	#if 0
	vector<double*> omega;
	for(int tS=0; tS<model.nNodeSPs; tS++){
	    omega.push_back(new double[model.n1]);
	    for(int ii=0;ii<model.n1; ii++) omega[tS][ii]=0.0;
	}
	#endif
	//model.readZIntoModel(z);
	//model.readOmegaIntoModel(omega);
	//PSCGNodeDesc *desc = new PSCGNodeDesc(&model);
	//desc->installSubproblemFromNodeDesc();
	//model.installSubproblem(z,omega,NULL,NULL,NULL,0);
	//AlpsKnowledgeBrokerSerial broker(dwmodel);
	//broker.setNodeSelection(new AlpsNodeSelectionBreadth());
	//broker.setNodeSelection(new AlpsNodeSelectionBest());
	//broker.setRampUpNodeSelection(new AlpsNodeSelectionDepth());
        //broker.setPhase(AlpsPhaseRampup);
//cout << "Begining search...." << endl;
//	broker.search(&dwmodel);
	//dwmodel.finalPrintout();
//cout << "...Ending search." << endl;
	//model.upBranchAllSPsAt(9,1.0);

	

#endif
	MPI_Finalize();
	

#endif
	return 0;
}

