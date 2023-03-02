#ifndef CHUNK_MAP_INFO_H_
#define CHUNK_MAP_INFO_H_

#include "../open_chisel/Chisel.h"
#include "chunk_manager_info.h"
using namespace std;

class ChiselMapInfo
{
public:
	ChiselMapInfo(){}

	ChiselMapInfo(chisel::Chisel * pChiselMap){
		this->m_pChunkManagerInfo = new ChunkManagerInfo(pChiselMap->GetMutableChunkManager());
		// chisel::ChunkSet iSet = pChiselMap->GetMutableMeshesToUpdate();
		// for (auto & iChunkPair : iSet){
		// 	if (iChunkPair.second == true){
		// 		this->m_gChunkIDs.push_back(iChunkPair.first);
		// 	}
		// }

	}

	chisel::Chisel * GenerateChiselMap(){
		chisel::Chisel * pChiselMap = new chisel::Chisel();
		pChiselMap->SetChunkManager(m_pChunkManagerInfo->GenerateChunkManager());

		//Update meshes to be updated.
		chisel::ChunkSet & iMeshesToUpdate = pChiselMap->GetMutableMeshesToUpdate();

		
		
		//Get all chunks of the manager
		chisel::ChunkManager &iManager = pChiselMap->GetMutableChunkManager(); 
		chisel::ChunkMap iMap = iManager.GetChunks();
		chisel::ChunkMap::iterator pIter;
		
		for (pIter = iMap.begin(); pIter != iMap.end(); pIter++){
			chisel::ChunkPtr pChunk = pIter->second;
			chisel::ChunkID mID = pChunk->GetID();
			for (int dx = -1; dx <= 1; dx++)
	        {
	            for (int dy = -1; dy <= 1; dy++)
	            {
	                for (int dz = -1; dz <= 1; dz++)
	                {
	                    iMeshesToUpdate[mID + chisel::ChunkID(dx, dy, dz)] = true;
	                }
	            }
	        }			
		}

		return pChiselMap;
	}

public:

	ChunkManagerInfo * m_pChunkManagerInfo;    //chunkManager;
	// vector<Eigen::Vector3i> m_gChunkIDs;     //meshesToUpdate;
	
};


#endif