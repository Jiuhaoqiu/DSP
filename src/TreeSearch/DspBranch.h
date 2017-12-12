/*
 * DspBranch.h
 *
 *  Created on: Oct 11, 2016
 *      Author: kibaekkim
 */

#ifndef SRC_TREESEARCH_DSPBRANCH_H_
#define SRC_TREESEARCH_DSPBRANCH_H_

#include <vector>

//struct DspBranch {
class DspBranch{//Might inherit from AlpsKnowledge?
public:
	std::vector<int> index_;
	std::vector<double> lb_;
	std::vector<double> ub_;
	double bestBound_; /**< best bound */
	//std::vector<double> dualsol_;

	DspBranch(){;}
	DspBranch(const DspBranch &rhs){
	    index_.resize(rhs.index_.size());
	    lb_.resize(rhs.lb_.size());
	    ub_.resize(rhs.ub_.size());
	    std::copy(rhs.index_.begin(),rhs.index_.end(),index_.begin());
	    std::copy(rhs.lb_.begin(),rhs.lb_.end(), lb_.begin());
	    std::copy(rhs.ub_.begin(),rhs.ub_.end(), ub_.begin());
	    bestBound_ = rhs.bestBound_;
	}
	~DspBranch(){;}
	virtual void push_back(int index, double lb, double ub) {
#if 1
		index_.push_back(index);
		lb_.push_back(lb);
		ub_.push_back(ub);
#endif
#if 0
		int jjj=-1
		for(iii=0; iii<index_.size(); iii++){
		    if(index_[iii]==index){ 
			jjj=iii;
			break;
		    }
		}
		if(jjj!=-1){
		    assert(index_[jjj]==index);
		    lb_[jjj] = lb;
		    ub_[jjj] = ub;
		}
		else{
		    index_.push_back(index);
		    lb_.push_back(lb);
		    ub_.push_back(ub);
		}
		return jjj;
#endif
	}
        virtual void modify_bound(int jjj, double lb, double ub){
	    lb_[jjj]=lb;
	    ub_[jjj]=ub;
	}
};

#endif /* SRC_TREESEARCH_DSPBRANCH_H_ */
