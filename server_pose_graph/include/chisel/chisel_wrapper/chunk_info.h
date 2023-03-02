#ifndef CHUNK_INFO_H_
#define CHUNK_INFO_H_

#include "../open_chisel/Chunk.h"
#include "color_voxel_info.h"
#include "dist_voxel_info.h"
#include <vector>

using namespace std;
class ChunkInfo
{
public:
	ChunkInfo(chisel::ChunkPtr pChunk){
		this->m_mChunkID = pChunk->GetID();
		this->m_mNumVoxels = pChunk->GetNumVoxels();
		this->m_nResolution = pChunk->GetVoxelResolutionMeters();
		//Copy voxels.
		vector<chisel::DistVoxel> gDistVoxels = pChunk->GetVoxels();
		vector<chisel::ColorVoxel> gColorVoxels = pChunk->GetColorVoxels();
		this->m_gVoxels.reserve(gDistVoxels.size());
		for (int i=0;i<gDistVoxels.size();i++){
			chisel::DistVoxel iVoxel = gDistVoxels[i];
			chisel::ColorVoxel iColorVoxel = gColorVoxels[i];
			if (iVoxel.GetWeight() > 1e-5){
				this->m_gVoxels.push_back(DistVoxelInfo(iVoxel));
				this->m_gColors.push_back(ColorVoxelInfo(iColorVoxel));
				this->m_gIndices.push_back(i);
			}
		}
		//Copy the origin.
		this->m_mOrigin = pChunk->GetOrigin();
	}

	ChunkInfo(){}

	chisel::ChunkPtr GenerateChunk(){
		// bool bUseColor = this->m_gColors.size() > 0;
		bool bUseColor = true;
		//Generate the chunk.
		chisel::ChunkPtr pChunk = chisel::ChunkPtr(new chisel::Chunk(
				this->m_mChunkID,
				this->m_mNumVoxels,
				this->m_nResolution,
				bUseColor));

		//Generate dist voxels.
		for (int i=0;i<this->m_gVoxels.size();i++){
			int nIndex = this->m_gIndices[i];
			chisel::DistVoxel & iVoxel = pChunk->GetDistVoxelMutable(nIndex);
			chisel::DistVoxel & iVoxelNew = this->m_gVoxels[i].GenerateDistVoxel();
			iVoxel = iVoxelNew;
		}
		//Generate color voxels.
		for (int i=0;i<this->m_gColors.size();i++){
			int nIndex = this->m_gIndices[i];
			pChunk->GetColorVoxelMutable(nIndex) = this->m_gColors[i].GenerateColorVoxel();
		}
		return pChunk;
	}


public:
    
    Eigen::Vector3i m_mChunkID;  //ID
    Eigen::Vector3i m_mNumVoxels;   //numVoxels
    float m_nResolution;    //voxelResolutionMeters
    vector<DistVoxelInfo> m_gVoxels;  //voxels
    vector<ColorVoxelInfo> m_gColors;    //colors
    vector<int> m_gIndices;
    Eigen::Vector3f m_mOrigin;    //origin
	
};


#endif