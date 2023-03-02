#ifndef DIST_VOXEL_INFO_H_
#define DIST_VOXEL_INFO_H_

#include "../open_chisel/DistVoxel.h"

using namespace std;

class DistVoxelInfo
{
public:
	DistVoxelInfo(chisel::DistVoxel & iDistVoxel){
		//Copy the information.
		this->m_nSDF = iDistVoxel.GetSDF();
		this->m_nWeight = iDistVoxel.GetWeight();

	}

	//Default constructor.
	DistVoxelInfo(){

	}

	chisel::DistVoxel & GenerateDistVoxel(){
		chisel::DistVoxel * pDistVoxel = new chisel::DistVoxel();
		pDistVoxel->SetSDF(this->m_nSDF);
		pDistVoxel->SetWeight(this->m_nWeight);
		return *pDistVoxel;
	}


public:
    float m_nSDF;
    float m_nWeight;
	
	
};




#endif