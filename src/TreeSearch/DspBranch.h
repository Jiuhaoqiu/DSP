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
	~DspBranch(){;}
	virtual void push_back(int index, double lb, double ub) {
		int iii;
		for(iii=0; iii<index_.size(); iii++){
		    if(index_[iii]==index) break;
		}
		if(iii<index_.size()){
		    assert(index_[iii]==index);
		    lb_[iii] = lb;
		    ub_[iii] = ub;
		}
		else{
		    index_.push_back(index);
		    lb_.push_back(lb);
		    ub_.push_back(ub);
		}
	}
};

#endif /* SRC_TREESEARCH_DSPBRANCH_H_ */
