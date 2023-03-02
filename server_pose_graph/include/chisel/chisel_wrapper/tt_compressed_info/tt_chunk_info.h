#ifndef TT_CHUNK_INFO_H_
#define TT_CHUNK_INFO_H_

#include "../../open_chisel/Chunk.h"
// #include "color_voxel_info.h"
// #include "dist_voxel_info.h"
#include <vector>

using namespace std;

class TTChunkInfo
{
public:
	

	TTChunkInfo(chisel::ChunkPtr pChunk){
		this->m_mChunkID = pChunk->GetID();
		this->m_mNumVoxels = pChunk->GetNumVoxels();
		this->m_nResolution = pChunk->GetVoxelResolutionMeters();
		//Copy voxels.
		//Copy the origin.
		this->m_mOrigin = pChunk->GetOrigin();
	}

	TTChunkInfo(){}


	chisel::ChunkPtr GenerateChunk(bool bUseColor = true){
		// bool bUseColor = true;
		//Generate the chunk.
		chisel::ChunkPtr pChunk = chisel::ChunkPtr(new chisel::Chunk(
				this->m_mChunkID,
				this->m_mNumVoxels,
				this->m_nResolution,
				bUseColor));
		
		return pChunk;
	}



public:
    
    int GetSize(){
    	int nSize = 0;
    	nSize += sizeof(m_mChunkID);
    	nSize += sizeof(m_mNumVoxels);
    	nSize += sizeof(m_nResolution);
    	nSize += sizeof(m_mOrigin);
    	

    	


    	return nSize;
    }

    Eigen::Vector3i m_mChunkID;  //ID
    Eigen::Vector3i m_mNumVoxels;   //numVoxels
    float m_nResolution;    //voxelResolutionMeters
    Eigen::Vector3f m_mOrigin;    //origin

    // int m_nVoxelSize;
	
};


#endif