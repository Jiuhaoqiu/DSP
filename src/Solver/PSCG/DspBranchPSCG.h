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
	std::vector<int> mpiRanks_;
	std::vector<int> spIndices_;
	std::vector<double*> dualsol_;

	DspBranchPSCG():DspBranch(){;}
	DspBranchPSCG(const DspBranchPSCG &rhs):DspBranch(rhs){
	    mpiRanks_.resize(rhs.mpiRanks_.size());	
	    spIndices_.resize(rhs.spIndices_.size());
	    dualsol_.resize(rhs.dualsol_.size());
	    std::copy(rhs.mpiRanks_.begin(),rhs.mpiRanks_.end(),mpiRanks_.begin());
	    std::copy(rhs.spIndices_.begin(),rhs.spIndices_.end(),spIndices_.begin());
	    std::copy(rhs.dualsol_.begin(),rhs.dualsol_.end(),dualsol_.begin());
	}
	~DspBranchPSCG(){;}
};

#endif /* SRC_TREESEARCH_DSPBRANCHPSCG_H_ */
