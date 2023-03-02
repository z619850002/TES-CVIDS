#include "../include/server_global_map.h"
#include "../include/chisel/open_chisel/chisel_wrapper.h"
using namespace std;

void ConvertArrayToChunksColor(
    float * pResult,  
    float * pResultR,
    float * pResultG,
    float * pResultB,
    int nShapeX, int nShapeY, int nShapeZ,
    chisel::ChunkManager * pManagerRef){
    
    int nSize = nShapeX * nShapeY * nShapeZ;
    // float * pResult = (float *)PyArray_DATA(pRetArray);
    // float * pResultR = (float *)PyArray_DATA(pRetArrayR);
    // float * pResultG = (float *)PyArray_DATA(pRetArrayG);
    // float * pResultB = (float *)PyArray_DATA(pRetArrayB);

    // int nCount1 = 0;
    // for (int i=0;i<nSize;i++){
    //     if (pResult[i] < 0.5 && pResult[i] > -0.5){
    //         nCount1++;
    //     }
    // }
    // cout << "Count1 Num is: " << nCount1 << endl;

    // ChunkManager &iManagerRef = pChiselMapRef->GetMutableChunkManager(); 
    chisel::Vec3List gCentroids = pManagerRef->GetCentroids();
    //Get all chunks of the merge manager
    chisel::ChunkMap iMap = pManagerRef->GetChunks();

    //Get all chunks of the merge manager
    chisel::ChunkMap::iterator pIter;
    vector<chisel::ChunkPtr> gChunks;
    // gChunks.reserve(iMap.size());
    for (pIter = iMap.begin(); pIter != iMap.end(); pIter++){
        chisel::ChunkPtr pChunk = pIter->second;
        gChunks.push_back(pChunk);
    }     

    float nResolution = pManagerRef->GetResolution();

    chisel::AABB iBoundingBox = gChunks[0]->ComputeBoundingBox();
    for (int i=1;i<gChunks.size();i++){
        chisel::AABB iBox = gChunks[i]->ComputeBoundingBox();
        iBoundingBox.Merge(iBox);
    }

    int nLargeNumX = ((iBoundingBox.max(0) - iBoundingBox.min(0) + 1e-3)/nResolution);
    int nLargeNumY = ((iBoundingBox.max(1) - iBoundingBox.min(1) + 1e-3)/nResolution);
    int nLargeNumZ = ((iBoundingBox.max(2) - iBoundingBox.min(2) + 1e-3)/nResolution);
    
    int nTotalMapSize = nLargeNumX * nLargeNumY * nLargeNumZ;  
    int nCount = 0;
    for (chisel::ChunkPtr  pChunk : gChunks){
        Eigen::Vector3i mNum = pChunk->GetNumVoxels();
        int nNumX = mNum(0);
        int nNumZ = mNum(1);
        int nNumY = mNum(2);

        int nTotalSize = nNumX*nNumY*nNumZ;

        for (int i=0;i<nTotalSize;i++){
            chisel::DistVoxel & iVoxel = pChunk->GetDistVoxelMutable(i);
            chisel::ColorVoxel & iColorVoxel = pChunk->GetColorVoxelMutable(i);
            float nSDF = iVoxel.GetSDF();
            if (isnan(nSDF)){
                iVoxel.SetSDF(99999);
                nSDF = 99999.0;
            }
            
            chisel::Vec3 mShift = gCentroids[i];
            chisel::Vec3 mHalfResolution = chisel::Vec3(nResolution, nResolution, nResolution) * 0.5f;
            chisel::Vec3 mFinalPosition = pChunk->GetOrigin() + mShift - mHalfResolution;

            int nX = ((mFinalPosition(0) - iBoundingBox.min(0) + 1e-3)/nResolution);
            int nY = ((mFinalPosition(1) - iBoundingBox.min(1) + 1e-3)/nResolution);
            int nZ = ((mFinalPosition(2) - iBoundingBox.min(2) + 1e-3)/nResolution);

            int nIndex = (nX*nLargeNumY+nY)*nLargeNumZ+nZ;
            if (nIndex >= nSize){
                continue;
            }
            float nCurrentSDF = pResult[nIndex];
            float nCurrentR = pResultR[nIndex];
            float nCurrentG = pResultG[nIndex];
            float nCurrentB = pResultB[nIndex];
            int nRed = (int)(nCurrentR*1.0);
            int nGreen = (int)(nCurrentG*1.0);
            int nBlue = (int)(nCurrentB*1.0);
            if (nRed > 255){
                nRed = 255;
            }
            if (nGreen > 255){
                nGreen = 255;
            }
            if (nBlue > 255){
                nBlue = 255;
            }

            if (nRed < 0){
                nRed = 0;
            }
            if (nGreen < 0){
                nGreen = 0;
            }
            if (nBlue < 0){
                nBlue = 0;
            }
            nCount++;
            iVoxel.SetSDF(nCurrentSDF);
            iVoxel.SetWeight(5);
            iColorVoxel.SetRed((uint8_t)nRed);
            iColorVoxel.SetGreen((uint8_t)nGreen);
            iColorVoxel.SetBlue((uint8_t)nBlue);
            iColorVoxel.SetWeight(5);

        }
    }


}



ServerSubMap::ServerSubMap(	chisel_msg::ChiselMapConstPtr & pMapMsg,
							vector<ServerKeyFrame *> & gKeyFrames,
							Sophus::SE3 mRelativeTransform_wr){
	
	this->m_bRecoveredDepth = false;
	//Copy the info from the message.
	chisel_msg::ChiselMap iMapMsg = *pMapMsg;
	this->m_pChisel.reset(new chisel::Chisel());
	Eigen::Affine3d mPoseAffine;
	this->ConvertSubMapInfo(iMapMsg, mPoseAffine, this->m_pChisel);
	this->m_gKeyFrames.clear();
	this->m_mRefPose_wc = Sophus::SE3(mPoseAffine.rotation(), mPoseAffine.translation());


	//Copy the initial information.
	chisel::ChunkManager &iManagerRef = this->m_pChisel->GetMutableChunkManager(); 
	this->m_mChunkSize = iManagerRef.GetChunkSize();
	this->m_bUseColor = iManagerRef.GetUseColor();
	this->m_nResolution = iManagerRef.GetResolution();


	//Initialize poses
	this->m_gKeyFrames.reserve(gKeyFrames.size());
	this->m_gCompositePoses_wc.reserve(gKeyFrames.size());
	for (ServerKeyFrame * pKeyFrame : gKeyFrames){
		this->m_gKeyFrames.push_back(pKeyFrame);
		Eigen::Matrix3d mInitialRotation_wc;
		Eigen::Vector3d mInitialTranslation_wc;
		pKeyFrame->GetInitialCameraPose(mInitialTranslation_wc, mInitialRotation_wc);

		Sophus::SE3 mSubMapPose_wc = Sophus::SE3(mInitialRotation_wc, mInitialTranslation_wc);
		Sophus::SE3 mRelPose_wc = this->m_mRefPose_wc.inverse() * mSubMapPose_wc;
		this->m_gCompositePoses_wc.push_back(mRelPose_wc);
	}
	this->m_mRefPose_wc  = mRelativeTransform_wr * this->m_mRefPose_wc;
	if (this->m_gKeyFrames.size() > 0){
		this->m_nClientID = this->m_gKeyFrames[0]->m_nClientID;
	}



	//Define the submap
	float nWeight = 1.0;
	int nChiselHeight = 480;
	int nChiselWidth = 640;
	bool bUseCarving = true;
	float nCarvingDist = 0.0;
	double nTruncationDistScale = 2.0;

	//Initialize the submap
	this->m_pTruncator.reset(new chisel::InverseTruncator(nTruncationDistScale));
	this->SetupIntegrator(this->m_pChisel, this->m_pTruncator, nWeight, bUseCarving, nCarvingDist);
	this->m_pDepthImage.reset(new chisel::DepthImage<DepthData>(nChiselWidth, nChiselHeight));
    this->m_pColorImage.reset(new chisel::ColorImage<ColorData>(nChiselWidth, nChiselHeight, 3));
    this->m_nWidth = nChiselWidth;
	this->m_nHeight = nChiselHeight;

	if (gKeyFrames.size() <1){
		ROS_WARN("Null submap!");
		return ;
	}

	ServerCamera * pCamera = gKeyFrames[0]->m_pServerCamera;
	ServerCamera * pDepthCamera = gKeyFrames[0]->m_pDepthCamera;
	Eigen::Matrix3d mK = pCamera->GetK();
	Eigen::Matrix3d mDepthK = pDepthCamera->GetK();
	int nImageWidth, nImageHeight;
	int nDepthWidth, nDepthHeight;
	pCamera->GetSize(nImageWidth, nImageHeight);
	pDepthCamera->GetSize(nDepthWidth, nDepthHeight);


	//Generate camera model.
    float nFocalX = mK(0 , 0) / (float)nImageWidth * (float)nChiselWidth;
    float nFocalY = mK(1 , 1) / (float)nImageHeight * (float)nChiselHeight;
    float nPrincipalX = mK(0 , 2) / (float)nImageWidth * (float)nChiselWidth;
    float nPrincipalY = mK(1 , 2) / (float)nImageHeight * (float)nChiselHeight;

    float nDepthFocalX = mDepthK(0 , 0)/ (float)nDepthWidth * (float)nChiselWidth;
    float nDepthFocalY = mDepthK(1 , 1)/ (float)nDepthHeight * (float)nChiselHeight;
    float nDepthPrincipalX = mDepthK(0 , 2)/ (float)nDepthWidth * (float)nChiselWidth;
    float nDepthPrincipalY = mDepthK(1 , 2)/ (float)nDepthHeight * (float)nChiselHeight;

    this->m_iColorCamera = GenerateCamera(
		nFocalX, nFocalY, 
		nPrincipalX, nPrincipalY,
		nChiselWidth, nChiselHeight);

	this->m_iDepthCamera = GenerateCamera(
		nDepthFocalX, nDepthFocalY, 
		nDepthPrincipalX, nDepthPrincipalY,
		nChiselWidth, nChiselHeight);

	float nNearDist = 0.3;
	float nFarDist = 3.0;

    this->m_iColorCamera.SetNearPlane(nNearDist);
    this->m_iColorCamera.SetFarPlane(nFarDist);
    this->m_iDepthCamera.SetNearPlane(nNearDist);
    this->m_iDepthCamera.SetFarPlane(nFarDist);
}


ServerSubMap::ServerSubMap(	chisel_msg::TTUncompressedChiselMapConstPtr & pMapMsg,
							vector<ServerKeyFrame *> & gKeyFrames,
							Sophus::SE3 mRelativeTransform_wr){
	

	this->m_bRecoveredDepth = false;
	//Copy the info from the message.
	chisel_msg::TTUncompressedChiselMap iMapMsg = *pMapMsg;
	this->m_pChisel.reset(new chisel::Chisel());
	Eigen::Affine3d mPoseAffine;
	this->ConvertSubMapInfo(iMapMsg, mPoseAffine, this->m_pChisel);
	this->m_gKeyFrames.clear();
	this->m_mRefPose_wc = Sophus::SE3(mPoseAffine.rotation(), mPoseAffine.translation());


	//Copy the initial information.
	chisel::ChunkManager &iManagerRef = this->m_pChisel->GetMutableChunkManager(); 
	this->m_mChunkSize = iManagerRef.GetChunkSize();
	this->m_bUseColor = iManagerRef.GetUseColor();
	this->m_nResolution = iManagerRef.GetResolution();


	//Initialize poses
	this->m_gKeyFrames.reserve(gKeyFrames.size());
	this->m_gCompositePoses_wc.reserve(gKeyFrames.size());
	for (ServerKeyFrame * pKeyFrame : gKeyFrames){
		this->m_gKeyFrames.push_back(pKeyFrame);
		Eigen::Matrix3d mInitialRotation_wc;
		Eigen::Vector3d mInitialTranslation_wc;
		pKeyFrame->GetInitialCameraPose(mInitialTranslation_wc, mInitialRotation_wc);

		Sophus::SE3 mSubMapPose_wc = Sophus::SE3(mInitialRotation_wc, mInitialTranslation_wc);
		Sophus::SE3 mRelPose_wc = this->m_mRefPose_wc.inverse() * mSubMapPose_wc;
		this->m_gCompositePoses_wc.push_back(mRelPose_wc);
	}
	this->m_mRefPose_wc  = mRelativeTransform_wr * this->m_mRefPose_wc;
	if (this->m_gKeyFrames.size() > 0){
		this->m_nClientID = this->m_gKeyFrames[0]->m_nClientID;
	}



	//Define the submap
	float nWeight = 1.0;
	int nChiselHeight = 480;
	int nChiselWidth = 640;
	bool bUseCarving = false;
	float nCarvingDist = 0.0;
	double nTruncationDistScale = 2.0;

	//Initialize the submap
	this->m_pTruncator.reset(new chisel::InverseTruncator(nTruncationDistScale));
	this->SetupIntegrator(this->m_pChisel, this->m_pTruncator, nWeight, bUseCarving, nCarvingDist);
	this->m_pDepthImage.reset(new chisel::DepthImage<DepthData>(nChiselWidth, nChiselHeight));
    this->m_pColorImage.reset(new chisel::ColorImage<ColorData>(nChiselWidth, nChiselHeight, 3));
    this->m_nWidth = nChiselWidth;
	this->m_nHeight = nChiselHeight;

	if (gKeyFrames.size() <1){
		ROS_WARN("Null submap!");
		return ;
	}

	ServerCamera * pCamera = gKeyFrames[0]->m_pServerCamera;
	ServerCamera * pDepthCamera = gKeyFrames[0]->m_pDepthCamera;
	Eigen::Matrix3d mK = pCamera->GetK();
	Eigen::Matrix3d mDepthK = pDepthCamera->GetK();
	int nImageWidth, nImageHeight;
	int nDepthWidth, nDepthHeight;
	pCamera->GetSize(nImageWidth, nImageHeight);
	pDepthCamera->GetSize(nDepthWidth, nDepthHeight);


	//Generate camera model.
    float nFocalX = mK(0 , 0) / (float)nImageWidth * (float)nChiselWidth;
    float nFocalY = mK(1 , 1) / (float)nImageHeight * (float)nChiselHeight;
    float nPrincipalX = mK(0 , 2) / (float)nImageWidth * (float)nChiselWidth;
    float nPrincipalY = mK(1 , 2) / (float)nImageHeight * (float)nChiselHeight;

    float nDepthFocalX = mDepthK(0 , 0)/ (float)nDepthWidth * (float)nChiselWidth;
    float nDepthFocalY = mDepthK(1 , 1)/ (float)nDepthHeight * (float)nChiselHeight;
    float nDepthPrincipalX = mDepthK(0 , 2)/ (float)nDepthWidth * (float)nChiselWidth;
    float nDepthPrincipalY = mDepthK(1 , 2)/ (float)nDepthHeight * (float)nChiselHeight;

    this->m_iColorCamera = GenerateCamera(
		nFocalX, nFocalY, 
		nPrincipalX, nPrincipalY,
		nChiselWidth, nChiselHeight);

	this->m_iDepthCamera = GenerateCamera(
		nDepthFocalX, nDepthFocalY, 
		nDepthPrincipalX, nDepthPrincipalY,
		nChiselWidth, nChiselHeight);

	float nNearDist = 0.3;
	float nFarDist = 3.0;

    this->m_iColorCamera.SetNearPlane(nNearDist);
    this->m_iColorCamera.SetFarPlane(nFarDist);
    this->m_iDepthCamera.SetNearPlane(nNearDist);
    this->m_iDepthCamera.SetFarPlane(nFarDist);
}




chisel::ProjectionIntegrator * ServerSubMap::SetupIntegrator(
		chisel::ChiselPtr pChiselMap, 
		chisel::TruncatorPtr pTruncator, 
		uint16_t nWeight, bool bUseCarving, float nCarvingDist){

	this->m_pProjectionIntegrator = new chisel::ProjectionIntegrator();

	this->m_pProjectionIntegrator->SetCentroids(pChiselMap->GetChunkManager().GetCentroids());
    this->m_pProjectionIntegrator->SetTruncator(pTruncator);
    this->m_pProjectionIntegrator->SetWeighter(chisel::WeighterPtr(new chisel::ConstantWeighter(nWeight)));
    this->m_pProjectionIntegrator->SetCarvingDist(nCarvingDist);
    this->m_pProjectionIntegrator->SetCarvingEnabled(bUseCarving);
    return this->m_pProjectionIntegrator;
}



void ServerSubMap::RegenerateMapQuickly(){
	//Just update the ref pose.
	//Choose the first pose.
	int nBaseIndex = 0;
	ServerKeyFrame * pKeyFrame = this->m_gKeyFrames[nBaseIndex];
	Eigen::Vector3d mNewTranslation_wc;
	Eigen::Matrix3d mNewRotation_wc;
	pKeyFrame->GetCameraPose(mNewTranslation_wc, mNewRotation_wc);

	Eigen::Affine3d mNewPose_wc;
	mNewPose_wc.linear() = mNewRotation_wc;
	mNewPose_wc.translation() = mNewTranslation_wc;
	Eigen::Affine3d mSubmapPose_wc;
	mSubmapPose_wc.linear() = this->m_mRefPose_wc.rotation_matrix();
	mSubmapPose_wc.translation() = this->m_mRefPose_wc.translation();

	Sophus::SE3 mLastCompositePose_wc = this->m_gCompositePoses_wc[nBaseIndex];
	Sophus::SE3 mCurrentCameraPose_wc(mNewRotation_wc, mNewTranslation_wc);

	//  T_wc * Tcom_rc.inv  =   T_wr  

	//Tcom_rc = T_wr.inv * T_wc 

	Sophus::SE3 mCurrentRef_wc =   mCurrentCameraPose_wc * mLastCompositePose_wc.inverse();
	this->m_mRefPose_wc = mCurrentRef_wc;
	
}



void ServerSubMap::RegenerateMap(){

	bool bQuickly = true;
	//Check the relative pose between the first frame and the last frame

	Eigen::Matrix3d mFirstRotation_wc, mLastRotation_wc;
	Eigen::Vector3d mFirstTranslation_wc, mLastTranslation_wc;

	ServerKeyFrame * pFirstKeyFrame = this->m_gKeyFrames[0]; 
	Sophus::SE3 mFirstCompositePose_wc = this->m_gCompositePoses_wc[0];

	ServerKeyFrame * pLastKeyFrame = this->m_gKeyFrames[this->m_gKeyFrames.size()-1];
	Sophus::SE3 mLastCompositePose_wc = this->m_gCompositePoses_wc[this->m_gKeyFrames.size()-1];
	

	pFirstKeyFrame->GetCameraPose(mFirstTranslation_wc, mFirstRotation_wc);
	pLastKeyFrame->GetCameraPose(mLastTranslation_wc, mLastRotation_wc);

	Sophus::SE3 mFirstPose_wc(mFirstRotation_wc, mFirstTranslation_wc);
	Sophus::SE3 mLastPose_wc(mLastRotation_wc, mLastTranslation_wc);

	Sophus::SE3 mCurrentRelativePose = mFirstPose_wc.inverse() * mLastPose_wc;
	Sophus::SE3 mOriginalRelativePose = mFirstCompositePose_wc.inverse() * mLastCompositePose_wc;

	Sophus::SE3 mShift = mCurrentRelativePose.inverse() * mOriginalRelativePose;
	double nError = mShift.translation().norm();

	cout << "nError is: " << nError << endl;
	if (nError > 0.05){
		bQuickly = false;
	}

	// bQuickly = true;

	if (bQuickly){
		for (int i=0;i<10;i++){
			cout << "regenerate map quickly" << endl;
		}
		this->RegenerateMapQuickly();
		return;
	}


	for (int i=0;i<10;i++){
		cout << "regenerate map originally" << endl;
	}


	if (!this->m_bRecoveredDepth){
		this->RecoverDepthMaps();
	}

	// cout << "Regenerate map, chunk size is: " << endl << this->m_mChunkSize << endl;
	// cout << "Resolution is: " << m_nResolution << endl;
	// cout << "Use color is: " << m_bUseColor << endl;
	this->m_pChisel.reset(new chisel::Chisel(this->m_mChunkSize, this->m_nResolution, this->m_bUseColor));
	// void Integrate(cv::Mat & mDepthImage, cv::Mat & mColorImage, Eigen::Affine3d & mPose_wc){
	
	for (int i=0;i<this->m_gKeyFrames.size();i+=1){
		ServerKeyFrame * pKeyFrame = this->m_gKeyFrames[i];
		Eigen::Vector3d mNewTranslation_wc;
		Eigen::Matrix3d mNewRotation_wc;
		pKeyFrame->GetCameraPose(mNewTranslation_wc, mNewRotation_wc);
		Eigen::Affine3d mNewPose_wc;
		mNewPose_wc.linear() = mNewRotation_wc;
		mNewPose_wc.translation() = mNewTranslation_wc;
		Eigen::Affine3d mSubmapPose_wc;
		mSubmapPose_wc.linear() = this->m_mRefPose_wc.rotation_matrix();
		mSubmapPose_wc.translation() = this->m_mRefPose_wc.translation();

		Eigen::Affine3d mRelPose = mSubmapPose_wc.inverse() * mNewPose_wc;
		Sophus::SE3 mRelPoseSophus(mRelPose.rotation(), mRelPose.translation());
		this->m_gCompositePoses_wc[i] = mRelPoseSophus;

		// cout << "Rel pose is: " << endl << mRelPose.matrix() << endl;
		// cout << "Comp rotation is: " << endl << this->m_gCompositePoses_wc[i].rotation_matrix() << endl;
		// cout << "Comp translation is: " << endl << this->m_gCompositePoses_wc[i].translation() << endl;
		if (pKeyFrame->m_bFinalizeDepthMap == false){
			continue;
		}
        pKeyFrame->m_mDepthMutex.lock();
        cv::Mat mDepthMapDouble = 1.0 / pKeyFrame->m_mInvDepthMap;
        cv::Mat mColorImage = pKeyFrame->m_mRecoveredImage;


        

        pKeyFrame->m_mDepthMutex.unlock();
        cv::Mat mDepthMapFloat;
		mDepthMapDouble.convertTo(mDepthMapFloat, CV_32FC1);

		//Resize the image.
		cv::resize( mDepthMapFloat, mDepthMapFloat, cv::Size(this->m_nWidth, this->m_nHeight), 0.0 , 0.0, cv::INTER_NEAREST); 
        cv::resize( mColorImage, mColorImage, cv::Size(this->m_nWidth, this->m_nHeight), 0.0 , 0.0, cv::INTER_NEAREST); 
        
  //       stringstream ss;
		// int nIndex = i;
		// string aIndex = "";
		// ss << nIndex;
		// ss >> aIndex;
		// string aPath = "/home/kyrie/Documents/DataSet/Submap/";
		// string aTotalFilename = aPath + aIndex + "_img.jpg";

		// string aDepthFileName = aPath + aIndex + "_dep.jpg";
		// cv::imwrite(aTotalFilename, mColorImage);

		//  cv::Mat mDepthMapWrite = 255 * mDepthMapFloat.clone()/10.0;
		// cv::imwrite(aDepthFileName, mDepthMapWrite);

  //   //                     // cv::imwrite(aFilename, mDepthMapWrite);
		// cout << "Depth map is: " << endl << mDepthMapFloat << endl;



		this->Integrate(mDepthMapFloat, mColorImage, mNewPose_wc);

		mDepthMapDouble.release();
		mDepthMapFloat.release();
		mColorImage.release();


	}
	
}

	
	

//Recover depth maps to all keyframes.
void ServerSubMap::RecoverDepthMaps(){

	chisel::TruncatorPtr pTruncator;
	double nTruncationDistScale = 2.0;
	pTruncator.reset(new chisel::InverseTruncator(nTruncationDistScale));
	chisel::ChunkManager &iManagerRef = this->m_pChisel->GetMutableChunkManager(); 
	iManagerRef.UpdateOccupancy();

	cv::Mat mEmptyDepthMap = cv::Mat::ones( this->m_nHeight, this->m_nWidth, CV_32FC1);
	cv::Mat mEmptyColorMap = cv::Mat::zeros( this->m_nHeight, this->m_nWidth, CV_8UC3);
	ServerKeyFrame * pFirstKeyFrame = this->m_gKeyFrames[0];
	for (int i=0;i<this->m_gKeyFrames.size();i+=4){
		Sophus::SE3 mCompositePose_wc = this->m_gCompositePoses_wc[i];
		Eigen::Affine3d mAffinePose_wc;
		mAffinePose_wc.linear() = mCompositePose_wc.rotation_matrix();
		mAffinePose_wc.translation() = mCompositePose_wc.translation();
		ServerKeyFrame * pKeyFrame = this->m_gKeyFrames[i];
		Eigen::Matrix3d mDepthK = pKeyFrame->m_pDepthCamera->GetK();
		
		int nWidth = pKeyFrame->m_nWidth;
		int nHeight = pKeyFrame->m_nHeight;

		double nRatioX, nRatioY;
		nRatioX = (double) nWidth / (double) this->m_nWidth;
		nRatioY = (double) nHeight / (double) this->m_nHeight;
		mDepthK(0,0) = mDepthK(0,0) / nRatioX;
		mDepthK(0,2) = mDepthK(0,2) / nRatioX;
		mDepthK(1,1) = mDepthK(1,1) / nRatioY;
		mDepthK(1,2) = mDepthK(1,2) / nRatioY;

		auto start_time = std::chrono::system_clock::now(); 
        
		chisel::RecoverDepthAndColorMapFast(
			this->m_pChisel,
			mDepthK.cast<float>(),
			mAffinePose_wc.cast<float>(),
			this->m_nWidth, 
			this->m_nHeight,
			mEmptyDepthMap,
			mEmptyColorMap,
			pTruncator);

		std::chrono::duration<double> elapsed1 = std::chrono::system_clock::now() - start_time;
        printf("recover time 1 is: %f\n", elapsed1.count() * 1000);


		cv::Mat mInvDepthMapDouble;
		cv::Mat mInvDepth = 1.0 / mEmptyDepthMap;

		mInvDepth.convertTo(mInvDepthMapDouble, CV_64FC1);

        pKeyFrame->m_mDepthMutex.lock();
		cv::resize(mInvDepthMapDouble, pKeyFrame->m_mInvDepthMap, cv::Size(nWidth, nHeight), 0.0 , 0.0, cv::INTER_NEAREST);
		cv::resize(mEmptyColorMap, pKeyFrame->m_mRecoveredImage, cv::Size(nWidth, nHeight), 0.0 , 0.0, cv::INTER_NEAREST);

		// stringstream ss;
		// string aIndex = "";
		// ss << i;
		// ss >> aIndex;
		// string aPrefixIndex;
		// stringstream ss2;
		// ss2 << pFirstKeyFrame->m_nGlobalIndex;
		// ss2 >> aPrefixIndex;
		
		// string aPath = "/home/kyrie/Documents/DataSet/Submap/Color/";
		


		// cv::Mat mDepthColor,dst;
		// // double min;
		// // double max;
		// // cv::minMaxIdx(mEmptyDepthMap, &min, &max);
		// double min = 0.0;
		// double max = 10.0;
		// convertScaleAbs(mEmptyDepthMap, dst, 255 / max);
		// applyColorMap(dst, mDepthColor, cv::COLORMAP_RAINBOW);



		// cv::imwrite(aPath + aPrefixIndex + "_" + aIndex + "_color.jpg", mEmptyColorMap);
		// cv::imwrite(aPath + aPrefixIndex + "_" + aIndex + "_depth.png", mDepthColor);
		
        // pKeyFrame->m_mInvDepthMap = mInvDepthMapDouble.clone();
        // pKeyFrame->m_mRecoveredImage = mEmptyColorMap.clone();
        pKeyFrame->m_bFinalizeDepthMap = true;
        pKeyFrame->m_mDepthMutex.unlock();

		// pKeyFrame->m_mRecoveredDepthMap = mEmptyDepthMap.clone();

		
		mInvDepth.release();
		mInvDepthMapDouble.release();




	}
	
	mEmptyDepthMap.release();
	mEmptyColorMap.release();
	

	this->m_bRecoveredDepth = true;
}







void ServerSubMap::ConvertSubMapInfo(
    chisel_msg::TTUncompressedChiselMap & iMapMsg,
    Eigen::Affine3d & mRefPose_wc,
    chisel::ChiselPtr & pChisel){

	//Get poses
    Eigen::Vector3d mTranslation_wc;
    Eigen::Matrix3d mRotation_wc;
    mTranslation_wc(0) = iMapMsg.refPosition_wc.x;
    mTranslation_wc(1) = iMapMsg.refPosition_wc.y;
    mTranslation_wc(2) = iMapMsg.refPosition_wc.z;
     
    Eigen::Quaterniond mQuaternion_wc;
    mQuaternion_wc.w() = iMapMsg.refOrientation_wc.w;
    mQuaternion_wc.x() = iMapMsg.refOrientation_wc.x;
    mQuaternion_wc.y() = iMapMsg.refOrientation_wc.y;
    mQuaternion_wc.z() = iMapMsg.refOrientation_wc.z;
    mRotation_wc = mQuaternion_wc.toRotationMatrix();

    //Copy the pose
    mRefPose_wc.linear() = mRotation_wc;
    mRefPose_wc.translation() = mTranslation_wc;

    //Generate chisel map.
    TTChiselMapInfo * pChiselMapInfo = new TTChiselMapInfo();
    TTChunkManagerInfo * pManagerInfo = new TTChunkManagerInfo();
    pChiselMapInfo->m_pChunkManagerInfo = pManagerInfo;
    pManagerInfo->m_mChunkSize(0) = iMapMsg.chunkSize.x;
    pManagerInfo->m_mChunkSize(1) = iMapMsg.chunkSize.y;
    pManagerInfo->m_mChunkSize(2) = iMapMsg.chunkSize.z;
    pManagerInfo->m_nVoxelResolutionMeters = iMapMsg.voxelResolution;
    pManagerInfo->m_bUseColor = iMapMsg.useColor;
    pManagerInfo->m_gChunkValues.reserve(iMapMsg.lengthChunks);
    for (chisel_msg::TTChunk & iChunk : iMapMsg.chunkValues){
        Eigen::Vector3d mChunkID;
        TTChunkInfo * pChunkInfo = new TTChunkInfo();
        pChunkInfo->m_mChunkID(0) = iChunk.chunkID.x;
        pChunkInfo->m_mChunkID(1) = iChunk.chunkID.y;
        pChunkInfo->m_mChunkID(2) = iChunk.chunkID.z;

        pChunkInfo->m_mNumVoxels(0) = iChunk.numVoxels.x;
        pChunkInfo->m_mNumVoxels(1) = iChunk.numVoxels.y;
        pChunkInfo->m_mNumVoxels(2) = iChunk.numVoxels.z;
        pChunkInfo->m_nResolution = iChunk.resolution;
        pChunkInfo->m_mOrigin(0) = iChunk.origin.x;
        pChunkInfo->m_mOrigin(1) = iChunk.origin.y;
        pChunkInfo->m_mOrigin(2) = iChunk.origin.z;

       
        pManagerInfo->m_gChunkValues.push_back(pChunkInfo);
    }


    chisel::Chisel * pChiselPt = pChiselMapInfo->GenerateChiselMap();
    chisel::ChunkManager iManager = pChiselPt->GetMutableChunkManager();
    //Recover the compressed infos.

	vector<chisel_msg::Tensor> gAllTensors;

	chisel_msg::Tensor iTensorDist = iMapMsg.tensorDist;
	chisel_msg::Tensor iTensorR = iMapMsg.tensorR;
	chisel_msg::Tensor iTensorG = iMapMsg.tensorG;
	chisel_msg::Tensor iTensorB = iMapMsg.tensorB;

	gAllTensors.reserve(4);
	gAllTensors.push_back(iTensorDist);
	gAllTensors.push_back(iTensorR);
	gAllTensors.push_back(iTensorG);
	gAllTensors.push_back(iTensorB);

	int nTotalLength = 0;
	int nLengthX = 0;
	int nLengthY = 0;
	int nLengthZ = 0;
	vector<float *> gResults;
	for (int i=0;i<4;i++){
		chisel_msg::Tensor iTensor = gAllTensors[i];
		nLengthX = iTensor.shape[0];
		nLengthY = iTensor.shape[1];
		nLengthZ = iTensor.shape[2];
		nTotalLength = nLengthX*nLengthY*nLengthZ;
		float * pData = new float[nTotalLength];
		for (int j=0;j<nTotalLength;j++){
			pData[j] = iTensor.datas[j];
		}
		gResults.push_back(pData);
	}
	cout << "Start to convert!" << endl;
	ConvertArrayToChunksColor(
		gResults[0],  
		gResults[1],  
		gResults[2],  
		gResults[3],
		nLengthX, nLengthY, nLengthZ, &iManager);

	pChisel.reset(pChiselPt);


}


void ServerSubMap::ConvertSubMapInfo(
    chisel_msg::ChiselMap & iMapMsg,
    Eigen::Affine3d & mRefPose_wc,
    chisel::ChiselPtr & pChisel){
    
    //Get poses
    Eigen::Vector3d mTranslation_wc;
    Eigen::Matrix3d mRotation_wc;
    mTranslation_wc(0) = iMapMsg.refPosition_wc.x;
    mTranslation_wc(1) = iMapMsg.refPosition_wc.y;
    mTranslation_wc(2) = iMapMsg.refPosition_wc.z;
     
    Eigen::Quaterniond mQuaternion_wc;
    mQuaternion_wc.w() = iMapMsg.refOrientation_wc.w;
    mQuaternion_wc.x() = iMapMsg.refOrientation_wc.x;
    mQuaternion_wc.y() = iMapMsg.refOrientation_wc.y;
    mQuaternion_wc.z() = iMapMsg.refOrientation_wc.z;
    mRotation_wc = mQuaternion_wc.toRotationMatrix();

    //Copy the pose
    mRefPose_wc.linear() = mRotation_wc;
    mRefPose_wc.translation() = mTranslation_wc;

    //Generate chisel map.
    ChiselMapInfo * pChiselMapInfo = new ChiselMapInfo();
    ChunkManagerInfo * pManagerInfo = new ChunkManagerInfo();
    pChiselMapInfo->m_pChunkManagerInfo = pManagerInfo;
    pManagerInfo->m_mChunkSize(0) = iMapMsg.chunkSize.x;
    pManagerInfo->m_mChunkSize(1) = iMapMsg.chunkSize.y;
    pManagerInfo->m_mChunkSize(2) = iMapMsg.chunkSize.z;
    pManagerInfo->m_nVoxelResolutionMeters = iMapMsg.voxelResolution;
    pManagerInfo->m_bUseColor = iMapMsg.useColor;
    pManagerInfo->m_gChunkValues.reserve(iMapMsg.lengthChunks);
    for (chisel_msg::Chunk & iChunk : iMapMsg.chunkValues){
        Eigen::Vector3d mChunkID;
        ChunkInfo * pChunkInfo = new ChunkInfo();
        pChunkInfo->m_mChunkID(0) = iChunk.chunkID.x;
        pChunkInfo->m_mChunkID(1) = iChunk.chunkID.y;
        pChunkInfo->m_mChunkID(2) = iChunk.chunkID.z;

        pChunkInfo->m_mNumVoxels(0) = iChunk.numVoxels.x;
        pChunkInfo->m_mNumVoxels(1) = iChunk.numVoxels.y;
        pChunkInfo->m_mNumVoxels(2) = iChunk.numVoxels.z;
        pChunkInfo->m_nResolution = iChunk.resolution;
        pChunkInfo->m_mOrigin(0) = iChunk.origin.x;
        pChunkInfo->m_mOrigin(1) = iChunk.origin.y;
        pChunkInfo->m_mOrigin(2) = iChunk.origin.z;

        pChunkInfo->m_gVoxels.reserve(iChunk.lengthVoxels);
        pChunkInfo->m_gColors.reserve(iChunk.lengthColors);
        pChunkInfo->m_gIndices.reserve(iChunk.lengthVoxels);


        for (int nIndex : iChunk.indices){
            pChunkInfo->m_gIndices.push_back(nIndex);
        }

        for (chisel_msg::DistVoxel & iVoxel : iChunk.voxels){
            DistVoxelInfo * pInfo = new DistVoxelInfo();

            if (iVoxel.sdf == 20000){
			    pInfo->m_nSDF = 99999;
			    pInfo->m_nWeight = 0.0;	
			}else{
			    pInfo->m_nSDF = ((float)(iVoxel.sdf))/1000.0;
			    pInfo->m_nWeight = ((float)(iVoxel.weight))/1000.0;	
			}
            // pInfo->m_nSDF = (float)(iVoxel.sdf)/1000.0;
            // pInfo->m_nWeight = (float)(iVoxel.weight)/1000.0;
            pChunkInfo->m_gVoxels.push_back(*pInfo);
        }

        for (chisel_msg::ColorVoxel & iColor : iChunk.colors){
            ColorVoxelInfo * pInfo = new ColorVoxelInfo();
            pInfo->m_nRed = iColor.red;
            pInfo->m_nGreen = iColor.green;
            pInfo->m_nBlue = iColor.blue;
            
            pInfo->m_nWeight = iColor.weight;

            pChunkInfo->m_gColors.push_back(*pInfo);
        }

        pManagerInfo->m_gChunkValues.push_back(pChunkInfo);
    }
    chisel::Chisel * pChiselPt = pChiselMapInfo->GenerateChiselMap();
    pChisel.reset(pChiselPt);
}