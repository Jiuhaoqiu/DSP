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
	std::vector<double*> &dualsol_;
	int mpiRank;
	int spNo;

	DspBranchPSCG(std::vector<double*> &omega):DspBranch(), dualsol_(omega){;}
	~DspBranchPSCG(){;}
};

#endif /* SRC_TREESEARCH_DSPBRANCHPSCG_H_ */
