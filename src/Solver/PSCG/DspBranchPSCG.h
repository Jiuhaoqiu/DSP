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
	~DspBranchPSCG(){
	}
	void addbranch(int br_rank, int br_scen, int br_index, double br_lb, double br_ub){
	   int jjj=-1;
	   for(int iii=0; iii< index_.size(); iii++){
		if(mpiRanks_[iii]==br_rank && spIndices_[iii]==br_scen && index_[iii]==br_index){
		    jjj=iii;
		    break;
		}
	   }
	   if(jjj!=-1){
		modify_bound(jjj,br_lb,br_ub);
	   }
	   else{
	       mpiRanks_.push_back(br_rank);
	       spIndices_.push_back(br_scen);
	       push_back(br_index,br_lb,br_ub);
	   }
	}
        void printBranchBounds() const{
	    std::cout << "Printing branches: " << std::endl;
	    int nBr = index_.size();
	    for(int ii=0; ii<nBr; ii++){
		std::cout << mpiRanks_[ii] << " " << spIndices_[ii] << " " << index_[ii] << " " << lb_[ii] << " " << ub_[ii] << std::endl;
	    }
	    std::cout << "Branch LagrBD: " << bestBound_ << std::endl;
	}
};

#endif /* SRC_TREESEARCH_DSPBRANCHPSCG_H_ */
