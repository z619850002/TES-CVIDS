#ifndef TT_CHUNK_MAP_INFO_H_
#define TT_CHUNK_MAP_INFO_H_

#include "../../open_chisel/Chisel.h"
#include "tt_chunk_manager_info.h"
using namespace std;

class TTChiselMapInfo
{
public:
	TTChiselMapInfo(){}

	TTChiselMapInfo(chisel::Chisel * pChiselMap, bool bUseColor = true){
		this->m_pChunkManagerInfo = new TTChunkManagerInfo(pChiselMap->GetMutableChunkManager(), bUseColor);

		// if (!bUseColor){
		// 	this->m_pChunkManagerInfo->m_bUseColor = false;
		// }
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

	int GetSize(){
		return this->m_pChunkManagerInfo->GetSize();
	}

	// void GetValidRatio(){
	// 	vector<ChunkInfo *> gChunks = this->m_pChunkManagerInfo->m_gChunkValues;
	// 	long long int nTotalSize = 0;
	// 	long long int nInvalidSize = 0;
	// 	for (ChunkInfo * pChunkInfo : gChunks){
	// 		vector<DistVoxelInfo> gVoxels = pChunkInfo->m_gVoxels; 
 //    		vector<ColorVoxelInfo> gColors = pChunkInfo->m_gColors;
 //    		int nSubInvalidSize1 = 0;
 //    		int nSubInvalidSize2 = 0;
 //    		int nSubTotalSize = gVoxels.size();
 //    		for (int i=0;i<gVoxels.size();i++){
 //    			DistVoxelInfo iInfo = gVoxels[i];
 //    			if (iInfo.m_nWeight > 1e-5){
 //    				continue;
 //    			}
 //    			nSubInvalidSize1++;
 //    		}

 //    		for (int i=0;i<gColors.size();i++){
 //    			ColorVoxelInfo iInfo = gColors[i];	
 //    			if (iInfo.m_nWeight > 0){
 //    				continue;
 //    			}
 //    			nSubInvalidSize2++;
 //    		}
 //    		if (nSubInvalidSize1 != nSubInvalidSize2){
 //    			cout << "Error!" << endl;
 //    		}
 //    		nInvalidSize+= nSubInvalidSize1;
 //    		nTotalSize += nSubTotalSize;
	// 	}
	// 	cout << "Invalid ratio is: " << (double)nInvalidSize / (double)nTotalSize << endl;
	// }

	TTChunkManagerInfo * m_pChunkManagerInfo;    //chunkManager;
	// vector<Eigen::Vector3i> m_gChunkIDs;     //meshesToUpdate;
	
};


#endif