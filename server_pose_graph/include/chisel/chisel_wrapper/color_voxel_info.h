#ifndef COLOR_VOXEL_INFO_H_
#define COLOR_VOXEL_INFO_H_

#include "../open_chisel/ColorVoxel.h"

using namespace std;

class ColorVoxelInfo
{
public:
	ColorVoxelInfo(chisel::ColorVoxel & iColorVoxel){
		//Copy the information.
		this->m_nRed = iColorVoxel.GetRed();
		this->m_nGreen = iColorVoxel.GetGreen();
		this->m_nBlue = iColorVoxel.GetBlue();
		this->m_nWeight = iColorVoxel.GetWeight();

	}

	//Default constructor.
	ColorVoxelInfo(){

	}

	chisel::ColorVoxel & GenerateColorVoxel(){
		chisel::ColorVoxel * pColorVoxel = new chisel::ColorVoxel();
		pColorVoxel->SetRed(this->m_nRed);
		pColorVoxel->SetGreen(this->m_nGreen);
		pColorVoxel->SetBlue(this->m_nBlue);
		pColorVoxel->SetWeight(this->m_nWeight);
		return *pColorVoxel;
	}


public:
    uint8_t m_nRed;
    uint8_t m_nGreen;
    uint8_t m_nBlue;
    uint8_t m_nWeight;
	
	
};




#endif