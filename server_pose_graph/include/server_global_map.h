#ifndef SERVER_GLOBAL_MAP_H_
#define SERVER_GLOBAL_MAP_H_

#include "./chisel/open_chisel/chisel_wrapper.h"
#include "./server_keyframe.h"
#include "./server_pose_graph.h"

#include <chisel_msg/ChiselMap.h>
#include <chisel_msg/TTUncompressedChiselMap.h>
#include <sstream>
#include "../include/chisel/chisel_wrapper/chisel_map_info.h"
#include "../include/chisel/chisel_wrapper/tt_compressed_info/tt_chisel_map_info.h"
#include <queue>
#include <mutex>

using namespace std;

class ServerSubMap
{
public:
	ServerSubMap(	chisel_msg::ChiselMapConstPtr & pMapMsg,
					vector<ServerKeyFrame *> & gKeyFrames,
					Sophus::SE3 mRelativeTransform_wr);

	ServerSubMap(	chisel_msg::TTUncompressedChiselMapConstPtr & pMapMsg,
					vector<ServerKeyFrame *> & gKeyFrames,
					Sophus::SE3 mRelativeTransform_wr);

	chisel::ProjectionIntegrator * SetupIntegrator(
		chisel::ChiselPtr pChiselMap, 
		chisel::TruncatorPtr pTruncator, 
		uint16_t nWeight, bool bUseCarving, float nCarvingDist);


	void RegenerateMap();

	void RegenerateMapQuickly();

	void Integrate(cv::Mat & mDepthImage, cv::Mat & mColorImage, Eigen::Affine3d & mPose_wc){
		this->m_pDepthImage.reset(new chisel::DepthImage<DepthData>(this->m_nWidth, this->m_nHeight));
		this->m_pColorImage.reset(new chisel::ColorImage<ColorData>(this->m_nWidth, this->m_nHeight, 3));

		CVToDepthMap(mDepthImage, this->m_pDepthImage.get());
		CVToColorMap(mColorImage, this->m_pColorImage.get());

		Eigen::Affine3d mRefPoseAffine_wc;
		mRefPoseAffine_wc.linear() = this->m_mRefPose_wc.rotation_matrix();
		mRefPoseAffine_wc.translation() = this->m_mRefPose_wc.translation();

		Eigen::Affine3d mRelPose_wc = mRefPoseAffine_wc.inverse() * mPose_wc;

		chisel::Transform iTransform_wc = GenerateTransform(mRelPose_wc);

		ROS_INFO("Start to integrate depth scan color");
		this->m_pChisel->IntegrateDepthScanColor<DepthData, ColorData>(
						*(this->m_pProjectionIntegrator), 
						this->m_pDepthImage, iTransform_wc, this->m_iDepthCamera, 
						this->m_pColorImage, iTransform_wc, this->m_iColorCamera);
		ROS_INFO("Finish to integrate depth scan color");

	}

	//Recover depth maps to all keyframes.
	void RecoverDepthMaps();

	void AddKeyFrame(ServerKeyFrame * pKeyFrame){
		this->m_gKeyFrames.push_back(pKeyFrame);
	}

	void SaveMesh(string aFilename){
		this->m_pChisel->UpdateMeshesInstantly();
		this->m_pChisel->SaveAllMeshesToPLY(aFilename);
	}


	void ConvertSubMapInfo(
	    chisel_msg::ChiselMap & iMapMsg,
	    Eigen::Affine3d & mRefPose_wc,
	    chisel::ChiselPtr & pChisel);



	void ConvertSubMapInfo(
	    chisel_msg::TTUncompressedChiselMap & iMapMsg,
	    Eigen::Affine3d & mRefPose_wc,
	    chisel::ChiselPtr & pChisel);

	chisel::ChiselPtr  m_pChisel;
	vector<ServerKeyFrame *>  m_gKeyFrames;
	vector<Sophus::SE3> m_gCompositePoses_wc;
	Sophus::SE3  m_mRefPose_wc;
	int m_nClientID;

	chisel::TruncatorPtr m_pTruncator;
	chisel::ProjectionIntegrator * m_pProjectionIntegrator;

	//Camera models.
	chisel::PinholeCamera m_iColorCamera, m_iDepthCamera;

	std::shared_ptr<chisel::DepthImage<DepthData>> m_pDepthImage;
	std::shared_ptr<chisel::ColorImage<ColorData>> m_pColorImage;
	int m_nHeight, m_nWidth;

	Eigen::Vector3i m_mChunkSize;
	bool m_bUseColor;
	float m_nResolution;

	bool m_bRecoveredDepth;


	
};

class ServerGlobalMap
{
public:
	ServerGlobalMap(ServerPoseGraph * pPoseGraph){
		//Default settings of the global map.
		bool bColor = true; 
		float nResolution = 0.03; 
		int nChunkSize = 8;
		this->m_mGlobalRefPose = Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0 , 0.0 , 0.0));
		this->m_pGlobalChisel.reset(new chisel::Chisel(Eigen::Vector3i(nChunkSize, nChunkSize, nChunkSize), nResolution, bColor));
		this->m_pPoseGraph = pPoseGraph;
		//Original mapping
		this->m_tGlobalMapping = std::thread(&ServerGlobalMap::Run, this);
		//compressed mapping
		// this->m_tGlobalMapping = std::thread(&ServerGlobalMap::RunTT, this);
		this->m_aProjectPath = pPoseGraph->m_aProjectPath;
	}

	void AddTTSubMap(chisel_msg::TTUncompressedChiselMapConstPtr & pMapMsg,
					vector<ServerKeyFrame *> & gKeyFrames, 
					Sophus::SE3 mRelativeTransform_wr){
		auto start_time = std::chrono::system_clock::now(); 
        
		ServerSubMap * pSubMap = new ServerSubMap(pMapMsg, gKeyFrames, mRelativeTransform_wr);

		std::chrono::duration<double> elapsed1 = std::chrono::system_clock::now() - start_time;
        printf("construction cost 1 is: %f\n", elapsed1.count() * 1000);

		pSubMap->RecoverDepthMaps();
		this->m_gSubMaps.push_back(pSubMap);

		std::chrono::duration<double> elapsed2 = std::chrono::system_clock::now() - start_time;
        printf("recover cost 2 is: %f\n", elapsed2.count() * 1000);



        // if (this->m_gMessageBuf.size() <=5 && this->m_gSubMaps.size() > 1){

			int nIndex = this->m_gSubMaps.size();
			stringstream ss;
			string aIndex = "";
			ss << nIndex;
			ss >> aIndex;
			string aPath = "/home/kyrie/Documents/DataSet/Submap/";
			string aTotalFilename = aPath  + aIndex + ".ply";
			
			// pSubMap->m_pGlobalChisel->GetMutableChunkManager().CutChunks(1.3);

			pSubMap->m_pChisel->UpdateMeshesInstantly();
			pSubMap->m_pChisel->SaveAllMeshesToPLY(aTotalFilename);
			// this->SavePoseGraph();
		


		this->MergeToGlobalMap(pSubMap);

		std::chrono::duration<double> elapsed3 = std::chrono::system_clock::now() - start_time;
        printf("Merge cost 3 is: %f\n", elapsed3.count() * 1000);

		// int nIndex = this->m_gSubMaps.size();
		// stringstream ss;
		// string aIndex = "";
		// ss << nIndex;
		// ss >> aIndex;
		// string aPath = "/home/kyrie/Documents/DataSet/Submap/";
		// string aTotalFilename = aPath + "final_inverse.ply";
		// this->m_pGlobalChisel->UpdateMeshesInstantly();
		// this->m_pGlobalChisel->SaveAllMeshesToPLY(aTotalFilename);
	}

	void AddSubMap(chisel_msg::ChiselMapConstPtr & pMapMsg,
					vector<ServerKeyFrame *> & gKeyFrames, 
					Sophus::SE3 mRelativeTransform_wr){
		auto start_time = std::chrono::system_clock::now(); 
        
		ServerSubMap * pSubMap = new ServerSubMap(pMapMsg, gKeyFrames, mRelativeTransform_wr);

		std::chrono::duration<double> elapsed1 = std::chrono::system_clock::now() - start_time;
        printf("construction cost 1 is: %f\n", elapsed1.count() * 1000);

		this->m_gSubMaps.push_back(pSubMap);

		std::chrono::duration<double> elapsed2 = std::chrono::system_clock::now() - start_time;
        printf("recover cost 2 is: %f\n", elapsed2.count() * 1000);


		this->MergeToGlobalMap(pSubMap);

		this->SendMeshes();


	}

	void MergeToGlobalMap(ServerSubMap * pSubMap){
		Sophus::SE3 mRelPose_wc = this->m_mGlobalRefPose.inverse() *  pSubMap->m_mRefPose_wc;
		Eigen::Affine3d mPoseAffine_wc;
		mPoseAffine_wc.linear() = mRelPose_wc.rotation_matrix();
		mPoseAffine_wc.translation() = mRelPose_wc.translation();
		Eigen::Affine3f mPoseAffineFloat_wc = mPoseAffine_wc.cast<float>();
		cout << "Start merge chunks!" << endl;
		
		if (pSubMap->m_nClientID <= 3 ){
			pSubMap->m_pChisel->GetMutableChunkManager().CutChunks(1.3);
		}
		if (pSubMap->m_nClientID == 4 &&  pSubMap->m_gKeyFrames[0]->m_nPublishIndex < 500)
		{
			pSubMap->m_pChisel->GetMutableChunkManager().CutChunks(1.3);
		}
		

		// if (pSubMap->m_nClientID <= 3 ){
		// 	pSubMap->m_pChisel->GetMutableChunkManager().CutChunks(1.3);
		// }
		// if (pSubMap->m_nClientID == 3 &&  pSubMap->m_gKeyFrames[0]->m_nPublishIndex < 500){
		// 	pSubMap->m_pChisel->GetMutableChunkManager().CutChunks(1.3);
		// 	string aPath = "/home/kyrie/Documents/DataSet/Submap/";
			
		// 	int nIndex = this->m_gSubMaps.size();
		// 	stringstream ss;
		// 	string aIndex = "";
		// 	ss << nIndex;
		// 	ss >> aIndex;
		// 	string aTotalFilename = aPath  + aIndex + "_sub_map.ply";

		// 	// this->m_pGlobalChisel->GetMutableChunkManager().CutChunks(1.3);

		// 	pSubMap->m_pChisel->UpdateMeshesInstantly();
		// 	pSubMap->m_pChisel->SaveAllMeshesToPLY(aTotalFilename);
		
		// }

		this->m_mChiselMutex.lock();
		chisel::MergeChunks(this->m_pGlobalChisel, pSubMap->m_pChisel, mPoseAffineFloat_wc);
		this->m_mChiselMutex.unlock();
		cout << "Finish merge chunks!" << endl;

		// if (this->m_gMessageBuf.size() <=5 && this->m_gSubMaps.size() > 1){

		// 	int nIndex = this->m_gSubMaps.size();
		// 	stringstream ss;
		// 	string aIndex = "";
		// 	ss << nIndex;
		// 	ss >> aIndex;
		// 	// string aPath = "/home/kyrie/Documents/DataSet/Submap/";
		// 	string aTotalFilename = this->m_aProjectPath  + "/final_inverse.ply";
		// 	for (int i=0;i<10;i++){
		// 		cout << "SavePath is: " << aTotalFilename << endl;
		// 	}
		// 	// this->m_pGlobalChisel->GetMutableChunkManager().CutChunks(1.3);


		// 	this->m_pGlobalChisel->UpdateMeshesInstantly();
		// 	this->m_pGlobalChisel->SaveAllMeshesToPLY(aTotalFilename);
		// 	// this->SavePoseGraph();
		// }else {
		// 	cout << "Message buf size: " << this->m_gMessageBuf.size() << endl;
		// }

	}

	void InverseMergeToGlobalMap(ServerSubMap * pSubMap){
		Sophus::SE3 mRelPose_wc = this->m_mGlobalRefPose.inverse() *  pSubMap->m_mRefPose_wc;
		Eigen::Affine3d mPoseAffine_wc;
		mPoseAffine_wc.linear() = mRelPose_wc.rotation_matrix();
		mPoseAffine_wc.translation() = mRelPose_wc.translation();
		Eigen::Affine3f mPoseAffineFloat_wc = mPoseAffine_wc.cast<float>();
		this->m_mChiselMutex.lock();
		chisel::InverseMergeChunks(this->m_pGlobalChisel, pSubMap->m_pChisel, mPoseAffineFloat_wc);
		this->m_mChiselMutex.unlock();
		// stringstream ss;
		// int nIndex = this->m_gSubMaps.size();
		// string aIndex = "";
		// ss << nIndex;
		// ss >> aIndex;
		// string aPath = "/home/kyrie/Documents/DataSet/Submap/";
		// string aTotalFilename = aPath + aIndex + "_inverse.ply";
		// this->m_pGlobalChisel->UpdateMeshesInstantly();
		// this->m_pGlobalChisel->SaveAllMeshesToPLY(aTotalFilename);

	}

	void Run(){
		while (1){


			//Take a rest.	
	        std::chrono::milliseconds nDura(300);
	        std::this_thread::sleep_for(nDura);

	        
	  //       this->m_mTTMessageMutex.lock();
			
			// if (!this->m_gTTMessageBuf.empty()){
			// 	chisel_msg::TTUncompressedChiselMapConstPtr pUCMessage = this->m_gTTMessageBuf.front();
			// 	this->m_gTTMessageBuf.pop();

			// 	//Process with the pUCMessage.

			// }
			

			// this->m_mTTMessageMutex.unlock();


			if (this->m_gSubMaps.size() > 0 && this->m_pPoseGraph->GetUpdateGlobalMap()){
				this->RegenerateGlobalMap();
				this->m_pPoseGraph->SetUpdateGlobalMap(false);
			}

			if (this->m_gMessageBuf.empty()){
				//No message need to be processed.
				continue;
			}

			bool bGet = false;

			for (int nSubMapInd = 0 ; nSubMapInd < this->m_gMessageBuf.size();nSubMapInd++){
				chisel_msg::ChiselMapConstPtr pSubmapMsg = this->m_gMessageBuf[nSubMapInd];
				//Get corresponding keyframes.
				this->m_pPoseGraph->m_mKeyFrameListMutex.lock();
	            vector<int> gEraseIndices;
	            chisel_msg::ChiselMap iMapMsg = *pSubmapMsg;

	            vector<int> gKeyFrameIndices;
	            for (int i=0;i< iMapMsg.publishIndices.size();i++){
	                gKeyFrameIndices.push_back(iMapMsg.publishIndices[i]);
	            }
	            int nSubMapClientID = iMapMsg.agentNum;

	            if (!this->m_pPoseGraph->Aligned(nSubMapClientID)){
					this->m_pPoseGraph->m_mKeyFrameListMutex.unlock();
	            	
	            	continue;
	            }

	            set<int> sKeyFrameSet(gKeyFrameIndices.begin(), gKeyFrameIndices.end());
	            vector<ServerKeyFrame *> gSubMapKeyFrames;
	            gSubMapKeyFrames.reserve(gKeyFrameIndices.size());

	            int nFind = 0;
	            for (ServerKeyFrame * pKeyFrame : this->m_pPoseGraph->m_gAllKeyFrames){
	                if (pKeyFrame->m_nClientID == nSubMapClientID && 
	                    sKeyFrameSet.find(pKeyFrame->m_nPublishIndex)!=sKeyFrameSet.end()){
	                    nFind++;
	                    gSubMapKeyFrames.push_back(pKeyFrame);
	                }
	            }

	            Sophus::SE3 mRelativeTransform_wr = this->m_pPoseGraph->GetTransformation(nSubMapClientID);
	            this->m_pPoseGraph->m_mKeyFrameListMutex.unlock();

				if (nFind == gKeyFrameIndices.size() || (nFind >= gKeyFrameIndices.size()-2  && gSubMapKeyFrames[0]->m_nPublishIndex<=2)){
	                //Add submaps to the global map
	                this->AddSubMap(pSubmapMsg, gSubMapKeyFrames, mRelativeTransform_wr);
	                bGet = true;
	                this->m_gMessageBuf.erase(this->m_gMessageBuf.begin()+nSubMapInd);
					
	                //Clear space.
	                 //Generate chisel map.
    

					break;
	            }
	            else {
	            	cout << "Wrong find: " << nFind << " / " << gKeyFrameIndices.size() << endl;
	            	cout << "Need: " << endl;
	            	for (auto item : gKeyFrameIndices){
	            		cout << item << " ";
	            	}
	            	cout << endl;
	            	cout << "Fact: " << endl;
	            	for (ServerKeyFrame * pKeyFrame : gSubMapKeyFrames){
	            		cout << pKeyFrame->m_nPublishIndex << " ";
	            	}
	            	cout << endl;
	            }


			}

			
		}
	}


	void RunTT(){
		while (1){


			//Take a rest.	
	        std::chrono::milliseconds nDura(30);
	        std::this_thread::sleep_for(nDura);

	        
	  //       this->m_mTTMessageMutex.lock();
			
			// if (!this->m_gTTMessageBuf.empty()){
			// 	chisel_msg::TTUncompressedChiselMapConstPtr pUCMessage = this->m_gTTMessageBuf.front();
			// 	this->m_gTTMessageBuf.pop();

			// 	//Process with the pUCMessage.
				
			// }
			

			// this->m_mTTMessageMutex.unlock();


			if (this->m_gSubMaps.size() > 0 && this->m_pPoseGraph->GetUpdateGlobalMap()){
				this->RegenerateGlobalMap();
				this->m_pPoseGraph->SetUpdateGlobalMap(false);
			}

			if (this->m_gTTMessageBuf.empty()){
				//No message need to be processed.
				continue;
			}

			bool bGet = false;

			// for (int i=0;i<10;i++){
			// 	cout << "Enter1" << endl;
			// }

			for (int nSubMapInd = 0 ; nSubMapInd < this->m_gTTMessageBuf.size();nSubMapInd++){
				chisel_msg::TTUncompressedChiselMapConstPtr pSubmapMsg = this->m_gTTMessageBuf[nSubMapInd];
				//Get corresponding keyframes.
				this->m_pPoseGraph->m_mKeyFrameListMutex.lock();
	            
	            vector<int> gEraseIndices;
	            chisel_msg::TTUncompressedChiselMap iMapMsg = *pSubmapMsg;

	            vector<int> gKeyFrameIndices;

	            for (int i=0;i< iMapMsg.publishIndices.size();i++){
	                gKeyFrameIndices.push_back(iMapMsg.publishIndices[i]);
	            }
	            int nSubMapClientID = iMapMsg.agentNum;

	            if (!this->m_pPoseGraph->Aligned(nSubMapClientID)){
	            	continue;
	            }

	            set<int> sKeyFrameSet(gKeyFrameIndices.begin(), gKeyFrameIndices.end());
	            vector<ServerKeyFrame *> gSubMapKeyFrames;
	            gSubMapKeyFrames.reserve(gKeyFrameIndices.size());

	            int nFind = 0;
	            for (ServerKeyFrame * pKeyFrame : this->m_pPoseGraph->m_gAllKeyFrames){
	                if (pKeyFrame->m_nClientID == nSubMapClientID && 
	                    sKeyFrameSet.find(pKeyFrame->m_nPublishIndex)!=sKeyFrameSet.end()){
	                    nFind++;
	                    gSubMapKeyFrames.push_back(pKeyFrame);
	                }
	            }

	            Sophus::SE3 mRelativeTransform_wr = this->m_pPoseGraph->GetTransformation(nSubMapClientID);
	            this->m_pPoseGraph->m_mKeyFrameListMutex.unlock();
	            
	            if (nFind == gKeyFrameIndices.size()){
	                //Add submaps to the global map

					// for (int i=0;i<10;i++){
					// 	cout << "Enter2" << endl;
					// }
	                this->AddTTSubMap(pSubmapMsg, gSubMapKeyFrames, mRelativeTransform_wr);
			                
					// for (int i=0;i<10;i++){
					// 	cout << "Enter3" << endl;
					// }
	                bGet = true;
	                this->m_gTTMessageBuf.erase(this->m_gTTMessageBuf.begin()+nSubMapInd);
					       
					// for (int i=0;i<10;i++){
					// 	cout << "Enter4" << endl;
					// }
					break;
	            }


			}

		}
	}


	void SendMeshes(){
    	visualization_msgs::Marker marker;
    	visualization_msgs::Marker marker2;
    	for (int i=0;i<10;i++){
    		cout << "Send Meshes!" << endl;
    	}
		this->m_mChiselMutex.lock();

		// this->m_pGlobalChisel->GetMutableChunkManager().CutChunks(1.3);

		this->FillMarkerTopicWithMeshes(&marker, &marker2);
		this->m_mChiselMutex.unlock();
		cout << "Finish filling" << endl;

	    if (!marker2.points.empty())
	    {
	        this->m_pMeshPublisher->publish(marker);
	        this->m_pMeshPublisher->publish(marker2);
	        cout << "Finish send mesh" << endl;
	    }

	}

	void FillMarkerTopicWithMeshes(visualization_msgs::Marker *marker, visualization_msgs::Marker *marker2)
	{

		this->m_pGlobalChisel->UpdateMeshesInstantly();

		string baseTransform = "/base";
	    assert(marker != nullptr);
	    assert(marker2 != nullptr);
	    marker2->header.stamp = ros::Time::now();
	    marker2->header.frame_id = baseTransform;
	    marker2->ns = "grid";
	    marker2->type = visualization_msgs::Marker::CUBE_LIST;
	    marker2->scale.x = this->m_pGlobalChisel->GetChunkManager().GetResolution();
	    marker2->scale.y = this->m_pGlobalChisel->GetChunkManager().GetResolution();
	    marker2->scale.z = this->m_pGlobalChisel->GetChunkManager().GetResolution();
	    marker2->pose.orientation.x = 0;
	    marker2->pose.orientation.y = 0;
	    marker2->pose.orientation.z = 0;
	    marker2->pose.orientation.w = 1;
	    marker2->color.r = 1.0;
	    marker2->color.g = 0.0;
	    marker2->color.b = 0.0;
	    marker2->color.a = 1.0;

	    marker->header.stamp = ros::Time::now();
	    marker->header.frame_id = baseTransform;
	    marker->ns = "mesh";
	    marker->scale.x = 1;
	    marker->scale.y = 1;
	    marker->scale.z = 1;
	    marker->pose.orientation.x = 0;
	    marker->pose.orientation.y = 0;
	    marker->pose.orientation.z = 0;
	    marker->pose.orientation.w = 1;
	    marker->type = visualization_msgs::Marker::TRIANGLE_LIST;
	    const chisel::MeshMap &meshMap = this->m_pGlobalChisel->GetChunkManager().GetAllMeshes();

	    if (meshMap.size() == 0)
	    {
	        ROS_INFO("No Mesh");
	        return;
	    }

	    chisel::Vec3 lightDir(0.8f, -0.2f, 0.7f);
	    lightDir.normalize();
	    chisel::Vec3 lightDir1(-0.5f, 0.2f, 0.2f);
	    lightDir.normalize();
	    const chisel::Vec3 ambient(0.2f, 0.2f, 0.2f);
	    //int idx = 0;
	    for (const std::pair<chisel::ChunkID, chisel::MeshPtr> &meshes : meshMap)
	    {
	        const chisel::MeshPtr &mesh = meshes.second;
	        for (size_t i = 0; i < mesh->grids.size(); i++)
	        {
	            const chisel::Vec3 &vec = mesh->grids[i];
	            geometry_msgs::Point pt;
	            pt.x = vec[0];
	            pt.y = vec[1];
	            pt.z = vec[2];
	            marker2->points.push_back(pt);
	        }
	        for (size_t i = 0; i < mesh->vertices.size(); i++)
	        {
	            const chisel::Vec3 &vec = mesh->vertices[i];
	            geometry_msgs::Point pt;
	            pt.x = vec[0];
	            pt.y = vec[1];
	            pt.z = vec[2];
	            marker->points.push_back(pt);

	            if (mesh->HasColors())
	            {
	                const chisel::Vec3 &meshCol = mesh->colors[i];
	                std_msgs::ColorRGBA color;
	                color.r = meshCol[0];
	                color.g = meshCol[1];
	                color.b = meshCol[2];
	                color.a = 1.0;
	                marker->colors.push_back(color);
	            }
	            else
	            {
	                if (mesh->HasNormals())
	                {
	                    const chisel::Vec3 normal = mesh->normals[i];
	                    std_msgs::ColorRGBA color;
	                    chisel::Vec3 lambert = chisel::LAMBERT(normal, lightDir) + chisel::LAMBERT(normal, lightDir1) + ambient;
	                    color.r = fmin(lambert[0], 1.0);
	                    color.g = fmin(lambert[1], 1.0);
	                    color.b = fmin(lambert[2], 1.0);
	                    color.a = 1.0;
	                    marker->colors.push_back(color);
	                }
	                else
	                {
	                    std_msgs::ColorRGBA color;
	                    color.r = vec[0] * 0.25 + 0.5;
	                    color.g = vec[1] * 0.25 + 0.5;
	                    color.b = vec[2] * 0.25 + 0.5;
	                    color.a = 1.0;
	                    marker->colors.push_back(color);
	                }
	            }
	            //marker->indicies.push_back(idx);
	            //idx++;
	        }
	    }
	}


	void RegenerateGlobalMap(){
		this->m_mChiselMutex.lock();

		//Reset the global map.
		bool bColor = true; 
		float nResolution = 0.03; 
		int nChunkSize = 8;
		this->m_mGlobalRefPose = Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0 , 0.0 , 0.0));
		this->m_pGlobalChisel.reset(new chisel::Chisel(Eigen::Vector3i(nChunkSize, nChunkSize, nChunkSize), nResolution, bColor));

		//Fuse all sub maps.
		int nIndex = 0;
		int nSize = this->m_gSubMaps.size();


    	#pragma omp parallel for
		for (int i=0;i<this->m_gSubMaps.size();i++){
			ServerSubMap * pSubMap = this->m_gSubMaps[i];
			ROS_INFO("Start to regenerate map");
			pSubMap->RegenerateMap();
		}


		this->m_mChiselMutex.unlock();

		for (ServerSubMap * pSubMap : this->m_gSubMaps){
			// cout << "Sub map size: " << this->m_gSubMaps.size() << endl;
			
			// nIndex++;

			// stringstream sSize;
			// stringstream sIndex;
			// sSize << nSize;
			// sIndex << nIndex;
			// string aIndex;
			// string aSize;
			// sSize >> aSize;
			// sIndex >> aIndex;
			
			// stringstream ss;
			// nIndex++;
			// string aIndex = "";
			// ss << nIndex;
			// ss >> aIndex;
			// string aPath = "/home/kyrie/Documents/DataSet/Submap/";
			// string aTotalFilename = aPath + aIndex + "_regenerate.ply";
			// pSubMap->SaveMesh(aTotalFilename);
			
			// ROS_INFO("Finish to regenerate map");
			this->MergeToGlobalMap(pSubMap);
			// string aPath = "/home/kyrie/Documents/DataSet/Submap/submaps";
			// // ROS_INFO("Finish to merge map");
			// string aTotalFilename = aPath + aSize + "_" + aIndex  + "_merged.ply";
			// pSubMap->SaveMesh(aTotalFilename);
			// this->m_pGlobalChisel->SaveAllMeshesToPLY(aTotalFilename);
		

		}



		this->SendMeshes();
	}


	int GetSubMapSize(){
		return this->m_gSubMaps.size();
	}

	void AddMessage(chisel_msg::ChiselMapConstPtr pChiselMap){
		this->m_mMessageMutex.lock();
		this->m_gMessageBuf.push_back(pChiselMap);
		
		this->m_mMessageMutex.unlock();
	}


	void AddTTMessage(chisel_msg::TTUncompressedChiselMapConstPtr pTTChiselMap){
		this->m_mTTMessageMutex.lock();
		this->m_gTTMessageBuf.push_back(pTTChiselMap);
		this->m_mTTMessageMutex.unlock();
	}


	void SavePoseGraph(){
		this->m_pPoseGraph->m_pPlotter->SavePoseGraph();
	}


	vector<ServerSubMap *> m_gSubMaps;

	chisel::ChiselPtr  m_pGlobalChisel;

	mutex m_mChiselMutex;
	Sophus::SE3 m_mGlobalRefPose;

	vector<chisel_msg::ChiselMapConstPtr> m_gMessageBuf;
	vector<chisel_msg::TTUncompressedChiselMapConstPtr> m_gTTMessageBuf;

	mutex m_mMessageMutex;
	mutex m_mTTMessageMutex;
	ServerPoseGraph * m_pPoseGraph;

	
	std::thread m_tGlobalMapping;

	string m_aProjectPath;

	ros::Publisher * m_pMeshPublisher;

	void BindPublisher(ros::Publisher * pPublisher){
		this->m_pMeshPublisher = pPublisher;
	}
	
};





#endif