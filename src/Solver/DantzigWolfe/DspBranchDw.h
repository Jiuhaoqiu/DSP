/*
 * DspBranchDw.h
 *
 *  Created on: 4 Dec 2017
 *      Author: Brian Dandurand
 */

#ifndef SRC_TREESEARCH_DSPBRANCHDW_H_
#define SRC_TREESEARCH_DSPBRANCHDW_H_

#include "TreeSearch/DspBranch.h"

//struct DspBranch {
class DspBranchDw : public DspBranch{
    public:
	std::vector<double> dualsol_;

	DspBranchDw():DspBranch(){;}
	~DspBranchDw(){;}
};

#endif /* SRC_TREESEARCH_DSPBRANCHDW_H_ */
