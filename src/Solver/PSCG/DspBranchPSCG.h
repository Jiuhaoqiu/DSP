/*
 * DspBranchDw.h
 *
 *  Created on: 4 Dec 2017
 *      Author: Brian Dandurand
 */

#ifndef SRC_TREESEARCH_DSPBRANCHPSCG_H_
#define SRC_TREESEARCH_DSPBRANCHPSCG_H_

#include "TreeSearch/DspBranch.h"

//struct DspBranch {
class DspBranchPSCG : public DspBranch{
    public:
	std::vector<int> mpiRanks;
	std::vector<int> spIndices;
	std::vector<double*> dualsol_;

	DspBranchPSCG():DspBranch(){;}
	~DspBranchPSCG(){;}
};

#endif /* SRC_TREESEARCH_DSPBRANCHPSCG_H_ */
