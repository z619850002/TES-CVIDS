#ifndef TT_CHUNK_MANAGER_INFO_H_
#define TT_CHUNK_MANAGER_INFO_H_

#include "../../open_chisel/ChunkManager.h"
#include "tt_chunk_info.h"
// #include "tt_compression.h"



#include <Python.h>

#include <numpy/arrayobject.h>


using namespace std;

class TTChunkManagerInfo
{
public:
	TTChunkManagerInfo(){

	}

	TTChunkManagerInfo(chisel::ChunkManager & iManager, bool bUseColor = true){
		chisel::ChunkMap iMap = iManager.GetChunks();
		
		for(auto & iChunkPair : iMap){
			// this->m_gChunkKeys.push_back(iChunkPair.first);
			this->m_gChunkValues.push_back(new TTChunkInfo(iChunkPair.second));
    	}

    	this->m_mChunkSize = iManager.GetChunkSize();
    	this->m_nVoxelResolutionMeters = iManager.GetResolution();
    	this->m_bUseColor = iManager.GetUseColor();
    	if (!bUseColor){
    		this->m_bUseColor = false;
    	}


    	//Compress the info of all chunks.
    	// chisel::InitializePythonEnvironment();
   //  	if (this->m_bUseColor){
   //  		vector<PyArrayObject *> gK1s, gK2s, gK3s;
   //  		// cout << "Decompose color" << endl;
			// DecomposeChunksColor(iManager, gK1s, gK2s, gK3s);	
			// // cout << "Finish decompose color" << endl;
			// PyArrayObject * pDistArrayK1 = gK1s[0];
			// PyArrayObject * pDistArrayK2 = gK2s[0];
			// PyArrayObject * pDistArrayK3 = gK3s[0];


			// this->m_pDimDistK1 = PyArray_SHAPE(pDistArrayK1);
		 //    this->m_pDimDistK2 = PyArray_SHAPE(pDistArrayK2);
		 //    this->m_pDimDistK3 = PyArray_SHAPE(pDistArrayK3);

		 //    this->m_pDistK1 = (float *)PyArray_DATA(pDistArrayK1);
		 //    this->m_pDistK2 = (float *)PyArray_DATA(pDistArrayK2);
		 //    this->m_pDistK3 = (float *)PyArray_DATA(pDistArrayK3);

		 //    //RGB
		 //    for (int i=1;i<4;i++){
			// 	PyArrayObject * pColorArrayK1 = gK1s[i];
			// 	PyArrayObject * pColorArrayK2 = gK2s[i];
			// 	PyArrayObject * pColorArrayK3 = gK3s[i];		    	

			// 	npy_intp * pDimColorK1 = PyArray_SHAPE(pColorArrayK1);
			//     npy_intp * pDimColorK2 = PyArray_SHAPE(pColorArrayK2);
			//     npy_intp * pDimColorK3 = PyArray_SHAPE(pColorArrayK3);

			// 	this->m_gDimsColorK1.push_back(pDimColorK1);	    	
			// 	this->m_gDimsColorK2.push_back(pDimColorK2);	    	
			// 	this->m_gDimsColorK3.push_back(pDimColorK3);

			// 	float * pColorK1 = (float *)PyArray_DATA(pColorArrayK1);
			//     float * pColorK2 = (float *)PyArray_DATA(pColorArrayK2);
			//     float * pColorK3 = (float *)PyArray_DATA(pColorArrayK3);

			//     this->m_gColorK1s.push_back(pColorK1);
			//     this->m_gColorK2s.push_back(pColorK2);
			//     this->m_gColorK3s.push_back(pColorK3);

		 //    }
		    
   //  	}else{
   //  		//Not use color.
   //  		PyArrayObject * pK1, * pK2, * pK3;
   //  		DecomposeChunks(iManager, pK1, pK2, pK3);	

			// this->m_pDimDistK1 = PyArray_SHAPE(pK1);
		 //    this->m_pDimDistK2 = PyArray_SHAPE(pK2);
		 //    this->m_pDimDistK3 = PyArray_SHAPE(pK3);

		 //    this->m_pDistK1 = (float *)PyArray_DATA(pK1);
		 //    this->m_pDistK2 = (float *)PyArray_DATA(pK2);
		 //    this->m_pDistK3 = (float *)PyArray_DATA(pK3);

   //  	}
		
	}


	chisel::ChunkManager & GenerateChunkManager(){
		chisel::ChunkManager * pManager = new chisel::ChunkManager(this->m_mChunkSize, this->m_nVoxelResolutionMeters, this->m_bUseColor);
		for (int i=0;i<this->m_gChunkValues.size();i++){
			// if (this->m_gChunkValues[i]->m_mOrigin(2) > 1.5){
			// 	continue;
			// }
			pManager->AddChunk(this->m_gChunkValues[i]->GenerateChunk());
		}

		//Load the compressed infos.
		// if (this->m_bUseColor){
		// 	//Load distance infos.
		// 	PyObject * pDistK1, * pDistK2, * pDistK3;
		// 	pDistK1 = PyArray_SimpleNewFromData(2, this->m_pDimDistK1, NPY_FLOAT32, this->m_pDistK1);
		// 	pDistK2 = PyArray_SimpleNewFromData(3, this->m_pDimDistK2, NPY_FLOAT32, this->m_pDistK2);
		// 	pDistK3 = PyArray_SimpleNewFromData(2, this->m_pDimDistK3, NPY_FLOAT32, this->m_pDistK3);
		// 	PyArrayObject * pDistResult =  ComposeArray(pDistK1, pDistK2, pDistK3, false);

		// 	//Load color infos.
		// 	vector<PyArrayObject *> gColorResults;
		// 	gColorResults.reserve(3);
		// 	for (int i=0;i<3;i++){
		// 		PyObject * pColorK1, * pColorK2, * pColorK3;
		// 		pColorK1 = PyArray_SimpleNewFromData(2, this->m_gDimsColorK1[i], NPY_FLOAT32, this->m_gColorK1s[i]);
		// 		pColorK2 = PyArray_SimpleNewFromData(3, this->m_gDimsColorK2[i], NPY_FLOAT32, this->m_gColorK2s[i]);
		// 		pColorK3 = PyArray_SimpleNewFromData(2, this->m_gDimsColorK3[i], NPY_FLOAT32, this->m_gColorK3s[i]);
		// 		// cout << "Compose color" << endl;
		// 		PyArrayObject * pResult =  ComposeArray(pColorK1, pColorK2, pColorK3, true);				
		// 		// cout << "Finish compose color" << endl;
		// 		gColorResults.push_back(pResult);
		// 	}

		// 	//Update all chunks.
		// 	cout << "Convert array to chunks" << endl;
		// 	ConvertArrayToChunksColor(
		// 		pDistResult, 
		// 		gColorResults[0], 
		// 		gColorResults[1], 
		// 		gColorResults[2], 
		// 		pManager);
		// 	cout << "Finish convert array to chunks" << endl;

		// }else{
		// 	//Load distance infos.
		// 	PyObject * pDistK1, * pDistK2, * pDistK3;
		// 	pDistK1 = PyArray_SimpleNewFromData(2, this->m_pDimDistK1, NPY_FLOAT32, this->m_pDistK1);
		// 	pDistK2 = PyArray_SimpleNewFromData(3, this->m_pDimDistK2, NPY_FLOAT32, this->m_pDistK2);
		// 	pDistK3 = PyArray_SimpleNewFromData(2, this->m_pDimDistK3, NPY_FLOAT32, this->m_pDistK3);
		// 	PyArrayObject * pDistResult =  ComposeArray(pDistK1, pDistK2, pDistK3, false);

		// 	ConvertArrayToChunks(pDistResult, pManager);

		// }

		return *pManager;		
	}




// typedef std::unordered_map<ChunkID, ChunkPtr, ChunkHasher> ChunkMap;
// typedef std::unordered_map<ChunkID, bool, ChunkHasher> ChunkSet;
// typedef std::unordered_map<ChunkID, MeshPtr, ChunkHasher> MeshMap;

public:


	int GetSize(){
		int nSize = 0;
		nSize += sizeof(m_mChunkSize);
		nSize += sizeof(m_nVoxelResolutionMeters);
		nSize += sizeof(m_bUseColor);
		for (TTChunkInfo * pInfo : this->m_gChunkValues){
			nSize += pInfo->GetSize();
		}
		return nSize;
	}

	// vector<Eigen::Vector3i> m_gChunkKeys;
	vector<TTChunkInfo *> m_gChunkValues;  //ChunkMap chunks;
    Eigen::Vector3i m_mChunkSize;        //chunkSize;
    float m_nVoxelResolutionMeters;      //voxelResolutionMeters;
    bool  m_bUseColor;                               //useColor;

    



    //Compressed Info.
	// npy_intp * m_pDimDistK1, *m_pDimDistK2, *m_pDimDistK3;
	// float * m_pDistK1, *m_pDistK2, *m_pDistK3;
	// vector<npy_intp *> m_gDimsColorK1, m_gDimsColorK2, m_gDimsColorK3;
	// vector<float *> m_gColorK1s, m_gColorK2s, m_gColorK3s;
	
};

#endif