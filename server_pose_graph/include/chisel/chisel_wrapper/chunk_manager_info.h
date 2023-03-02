#ifndef CHUNK_MANAGER_INFO_H_
#define CHUNK_MANAGER_INFO_H_

#include "../open_chisel/ChunkManager.h"
#include "chunk_info.h"

using namespace std;

class ChunkManagerInfo
{
public:
	ChunkManagerInfo(){

	}

	ChunkManagerInfo(chisel::ChunkManager & iManager){
		chisel::ChunkMap iMap = iManager.GetChunks();
		for(auto & iChunkPair : iMap){
			// this->m_gChunkKeys.push_back(iChunkPair.first);
			this->m_gChunkValues.push_back(new ChunkInfo(iChunkPair.second));
    	}

    	this->m_mChunkSize = iManager.GetChunkSize();
    	this->m_nVoxelResolutionMeters = iManager.GetResolution();
    	this->m_bUseColor = iManager.GetUseColor();
	}


	chisel::ChunkManager & GenerateChunkManager(){
		chisel::ChunkManager * pManager = new chisel::ChunkManager(this->m_mChunkSize, this->m_nVoxelResolutionMeters, this->m_bUseColor);
		for (int i=0;i<this->m_gChunkValues.size();i++){
			// if (this->m_gChunkValues[i]->m_mOrigin(2) > 1.0){
			// 	continue;
			// }
			pManager->AddChunk(this->m_gChunkValues[i]->GenerateChunk());
		}
		return *pManager;		
	}




// typedef std::unordered_map<ChunkID, ChunkPtr, ChunkHasher> ChunkMap;
// typedef std::unordered_map<ChunkID, bool, ChunkHasher> ChunkSet;
// typedef std::unordered_map<ChunkID, MeshPtr, ChunkHasher> MeshMap;

public:
	// vector<Eigen::Vector3i> m_gChunkKeys;
	vector<ChunkInfo *> m_gChunkValues;  //ChunkMap chunks;
    Eigen::Vector3i m_mChunkSize;        //chunkSize;
    float m_nVoxelResolutionMeters;      //voxelResolutionMeters;
    bool  m_bUseColor;                               //useColor;

    // vector<Eigen::Vector3f> m_gCentroids; //Vec3List centroids;
    // Eigen::Matrix<int, 3, 8> cubeIndexOffsets;
    // MeshMap allMeshes;
    
	
	
};

#endif