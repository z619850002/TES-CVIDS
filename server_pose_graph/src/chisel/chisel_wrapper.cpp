
#include "../../include/chisel/open_chisel/chisel_wrapper.h"

using namespace std;

//Util functions.
bool CVToDepthMap(
		cv::Mat & mDepthMap,
		chisel::DepthImage<DepthData> * pDepthImage)
{
    size_t nDataSize = mDepthMap.step / mDepthMap.cols;
    assert(pDepthImage->GetHeight() == static_cast<int>(mDepthMap.rows) &&
     	   pDepthImage->GetWidth() == static_cast<int>(mDepthMap.cols));
    assert(nDataSize == sizeof(DepthData));

    const DepthData * pImageData = reinterpret_cast<const DepthData *>(mDepthMap.data);
    DepthData * pDepthImageData = pDepthImage->GetMutableData();
    int nTotalPixels = mDepthMap.rows * mDepthMap.cols;
    for (int i = 0; i < nTotalPixels; i++)
    {
        pDepthImageData[i] = pImageData[i];
    }
	return true;
}



bool CVToColorMap(
		cv::Mat & mColorMap,
		chisel::ColorImage<ColorData> * pColorImage){

	size_t nNumChannels = pColorImage->GetNumChannels();
    size_t nDataSize = mColorMap.step / mColorMap.cols;
    assert(	pColorImage->GetHeight() == static_cast<int>(mColorMap.rows) && 
    		pColorImage->GetWidth() == static_cast<int>(mColorMap.cols));

    if (nDataSize != nNumChannels * sizeof(ColorData))
    {
    	cout << "Inconsistent channel width!" << endl;
        // ROS_ERROR("Inconsistent channel width: %lu. Expected %lu\n", dataSize, numChannels * sizeof(DataType));
        return false;
    }

    const ColorData *pImageData = reinterpret_cast<const ColorData *>(mColorMap.data);
    ColorData * pColorImageData = pColorImage->GetMutableData();
    int nTotalPixels = mColorMap.rows * mColorMap.cols * nNumChannels;
    for (int i = 0; i < nTotalPixels; i++)
    {
        pColorImageData[i] = pImageData[i];
    }

	return true;
}



chisel::PinholeCamera GenerateCamera(
	float nFocalX, float nFocalY, 
	float nPrincipalX, float nPrincipalY, 
	int nWidth, int nHeight)
{
    chisel::PinholeCamera iCameraModel;
    chisel::Intrinsics iIntrinsics;
    iIntrinsics.SetFx(nFocalX);
    iIntrinsics.SetFy(nFocalY);
    iIntrinsics.SetCx(nPrincipalX);
    iIntrinsics.SetCy(nPrincipalY);
    iCameraModel.SetIntrinsics(iIntrinsics);
    iCameraModel.SetWidth(nWidth);
    iCameraModel.SetHeight(nHeight);
    return iCameraModel;
}


	
chisel::Transform GenerateTransform(Eigen::Affine3d mTransform)
{
    chisel::Transform iTransform;

	Eigen::Vector3d mTranslation = mTransform.translation();
	Eigen::Matrix3d mRotation = mTransform.rotation();
	Eigen::Quaterniond mRotationQuaternion(mRotation);

    iTransform.translation()(0) = mTranslation.x();
    iTransform.translation()(1) = mTranslation.y();
    iTransform.translation()(2) = mTranslation.z();

    chisel::Quaternion iQuat;
    iQuat.x() = mRotationQuaternion.x();
    iQuat.y() = mRotationQuaternion.y();
    iQuat.z() = mRotationQuaternion.z();
    iQuat.w() = mRotationQuaternion.w();
    iTransform.linear() = iQuat.toRotationMatrix();

    return iTransform;
}



namespace chisel{

float Norms(Vec3 mVec){
    return sqrt(mVec(0) * mVec(0) + mVec(1) * mVec(1) + mVec(2)* mVec(2));
}


bool InverseMergeChunks(
    chisel::ChiselPtr pChiselMapRef,
    chisel::ChiselPtr pChiselMapMerge,
    Eigen::Affine3f mPose_rm){


    ChunkManager &iManagerRef = pChiselMapRef->GetMutableChunkManager(); 
    ChunkManager &iManagerMerge = pChiselMapMerge->GetMutableChunkManager(); 

    //Get all chunks of the merge manager
    ChunkMap iMap = iManagerMerge.GetChunks();
    ChunkMap::iterator pIter;
    vector<ChunkPtr> gChunksToBeMerged;
    gChunksToBeMerged.reserve(iMap.size());
    for (pIter = iMap.begin(); pIter != iMap.end(); pIter++){
        ChunkPtr pChunk = pIter->second;
        gChunksToBeMerged.push_back(pChunk);
    }

    Vec3List gCentroids = iManagerMerge.GetCentroids();


    ChunkIDList gGarbageChunks;
    //Merge chunks one by one.
    for (ChunkPtr pChunk : gChunksToBeMerged){
        float nResolution = pChunk->GetVoxelResolutionMeters();
        Vec3 mOrigin = pChunk->GetOrigin();
        float nResolutionDiagonal = 2.0 * sqrt(3.0f) * nResolution;
        bool bUpdated = false;

        for (size_t i = 0; i < gCentroids.size(); i++)
        //parallel_for(indexes.begin(), indexes.end(), [&](const size_t& i)
        {
            DistVoxel &iVoxelMerged = pChunk->GetDistVoxelMutable(i);
            if (iVoxelMerged.GetWeight()  < 1e-5){
                continue;
            }
            ColorVoxel &iVoxelMergedColor = pChunk->GetColorVoxelMutable(i);

            //World coord of the voxel.
            Vec3 mVoxelCenter = gCentroids[i] + mOrigin;
            Vec3 mVoxelCenterRef = mPose_rm * mVoxelCenter;

            //Find the corresponding voxel in iManagerRef
            ChunkID iIDRef = iManagerRef.GetIDAt(mVoxelCenterRef);
            bool bChunkNew = false;
            // mutex.lock();
            if (!iManagerRef.HasChunk(iIDRef))
            {
                continue;
                // bChunkNew = true;
                // iManagerRef.CreateChunk(iIDRef);
                // cout << "Updated Chunk size: " << pChiselMapRef->GetMutableChunkManager().GetChunks().size() << endl;
                // cout << "Updated Chunk size2: " << iManagerRef.GetChunks().size() << endl;
            }
            ChunkPtr pChunkToBeUpdated = iManagerRef.GetChunk(iIDRef);
            // mutex.unlock();

            //Update pChunkToBeUpdated
            Vec3 mPos = mVoxelCenterRef - pChunkToBeUpdated->GetOrigin();
            Point3 mPoint = pChunkToBeUpdated->GetVoxelCoords(mPos);
            Point3 mPointLeft = mPoint - Point3(1 , 0 , 0);
            Point3 mPointRight = mPoint + Point3(1 , 0 , 0);
            Point3 mPointFront = mPoint + Point3(0 , 1 , 0);
            Point3 mPointBack = mPoint - Point3(0 , 1 , 0);
            Point3 mPointTop = mPoint + Point3(0 , 0 , 1);
            Point3 mPointBottom = mPoint - Point3(0 , 0 , 1);

            Vec3 mPosCenter = pChunkToBeUpdated->GetVoxelPoints(mPoint);
            Vec3 mPosLeft = pChunkToBeUpdated->GetVoxelPoints(mPointLeft);
            Vec3 mPosRight = pChunkToBeUpdated->GetVoxelPoints(mPointRight);
            Vec3 mPosFront = pChunkToBeUpdated->GetVoxelPoints(mPointFront);
            Vec3 mPosBack = pChunkToBeUpdated->GetVoxelPoints(mPointBack);
            Vec3 mPosTop = pChunkToBeUpdated->GetVoxelPoints(mPointTop);
            Vec3 mPosBottom = pChunkToBeUpdated->GetVoxelPoints(mPointBottom);

            // float nDistCenter = Norms(mPos - mPosCenter);
            // float nDistLeft = Norms(mPos - mPosLeft);
            // float nDistRight = Norms(mPos - mPosRight);
            // float nDistFront = Norms(mPos - mPosFront);
            // float nDistBack = Norms(mPos - mPosBack);
            // float nDistTop = Norms(mPos - mPosTop);
            // float nDistBottom = Norms(mPos - mPosBottom);


            // float nTotalDist =  nDistCenter + nDistLeft + nDistRight + 
            //                     nDistFront + nDistBack + nDistTop + nDistBottom;

            float nWeight = iVoxelMerged.GetWeight();


            VoxelID nID = pChunkToBeUpdated->GetVoxelID(mPoint);
            VoxelID nLeftID = pChunkToBeUpdated->GetVoxelID(mPointLeft);
            VoxelID nRightID = pChunkToBeUpdated->GetVoxelID(mPointRight);
            VoxelID nFrontID = pChunkToBeUpdated->GetVoxelID(mPointFront);
            VoxelID nBackID = pChunkToBeUpdated->GetVoxelID(mPointBack);
            VoxelID nTopID = pChunkToBeUpdated->GetVoxelID(mPointTop);
            VoxelID nBottomID = pChunkToBeUpdated->GetVoxelID(mPointBottom);

            std::vector<VoxelID> gIDs {nID, nLeftID, nRightID, nFrontID, nBackID, nTopID, nBottomID};
           
            

            for (int ii=0;ii<gIDs.size();ii++){
                if (gIDs[ii] < 0 || gIDs[ii] >= pChunkToBeUpdated->GetTotalNumVoxels()){
                    continue;
                }

                DistVoxel & iVoxelToBeUpdated = pChunkToBeUpdated->GetDistVoxelMutable(gIDs[ii]);
                ColorVoxel & iVoxelToBeUpdatedColor = pChunkToBeUpdated->GetColorVoxelMutable(gIDs[ii]);


                //Update iVoxelToBeUpdated.

                iVoxelToBeUpdated.Integrate(iVoxelMerged.GetSDF(), -iVoxelMerged.GetWeight());                
                if (iVoxelToBeUpdated.GetWeight() <=1e-5){
                    iVoxelToBeUpdated.Reset();
                    iVoxelToBeUpdatedColor.Reset();
                }

                // if (iVoxelToBeUpdatedColor.GetWeight() >=1)
                // {    int nWeight = iVoxelMergedColor.GetWeight();
                //     iVoxelToBeUpdatedColor.InverseIntegrate(iVoxelMergedColor.GetRed(), iVoxelMergedColor.GetGreen(), iVoxelMergedColor.GetBlue(), -nWeight);
                //     if (iVoxelToBeUpdatedColor.GetWeight()== 0){
                //         iVoxelToBeUpdatedColor.Reset();
                //     }
                // }

            }


            bool bNeedUpdate = true;

            // mutex.lock();
            if (bNeedUpdate)
            {
                ChunkSet & iSet = pChiselMapRef->GetMutableMeshesToUpdate();
                for (int dx = -1; dx <= 1; dx++)
                {
                    for (int dy = -1; dy <= 1; dy++)
                    {
                        for (int dz = -1; dz <= 1; dz++)
                        {
                            iSet[iIDRef + ChunkID(dx, dy, dz)] = true;
                        }
                    }
                }
            }
            // else if(bChunkNew)
            // {
            //     gGarbageChunks.push_back(iIDRef);
            // }
            // mutex.unlock();

            //Remove empty chunks.
           
        }

    }
    // pChiselMapRef->GarbageCollect(gGarbageChunks);

    //Check which chunk should be removed.

    // ChunkMap iRefMap = pChiselMapRef->GetChunkManager().GetChunks();

    // // for(auto & iChunkPair : iRefMap){
    // //   // this->m_gChunkKeys.push_back(iChunkPair.first);
    // //   ChunkPtr chunk = iChunkPair.second;
    // //   ChunkID chunkID = iChunkPair.first;
    // //   std::vector<DistVoxel> gVoxels = chunk->GetVoxels();
 // //        int nValidNum = 0;
 // //        for (DistVoxel & iVoxel : gVoxels){
 // //            if (iVoxel.GetWeight() > 1e-5){
 // //                nValidNum++;
 // //            }
 // //        }
 // //        if (nValidNum <= 5){
 // //          ChunkManager iManager = pChiselMapRef->GetChunkManager();
 // //            iManager.RemoveChunk(chunkID);
 // //        }
    // // }


    return true;

}



Vec3 Trunc(Vec3 & mVec){
    if (abs(mVec(0))< 1e-5){
        mVec(0) = 0.0;
    }

    if (abs(mVec(1))< 1e-5){
        mVec(1) = 0.0;
    }

    if (abs(mVec(2))< 1e-5){
        mVec(2) = 0.0;
    }
    return mVec;
}

Eigen::Vector3i Trunc(Eigen::Vector3i & mVec, Eigen::Vector3i & mMin, Eigen::Vector3i & mMax){
    if (mVec(0) < mMin(0)){
        mVec(0) = mMin(0);
    }
    if (mVec(1) < mMin(1)){
        mVec(1) = mMin(1);
    }
    if (mVec(2) < mMin(2)){
        mVec(2) = mMin(2);
    }

    if (mVec(0) >= mMax(0)){
        mVec(0) = mMax(0)-1;
    }
    if (mVec(1) >= mMax(1)){
        mVec(1) = mMax(1)-1;
    }
    if (mVec(2) >= mMax(2)){
        mVec(2) = mMax(2)-1;
    }

    return mVec;
}


void CheckTSDF(
        chisel::ChiselPtr pChiselMapRef,
        Eigen::Matrix3f mDepthK,
        Eigen::Affine3f mPose_rc,
        int nDepthWidth, 
        int nDepthHeight,
        cv::Mat & mDepthMap){

        float nSearchStep = 0.03;
        float nMinDepth = 0.2;
        float nMaxDepth = 5;
        int nSearchTimes = (int)((nMaxDepth-nMinDepth) / nSearchStep);
        ChunkManager &iManagerRef = pChiselMapRef->GetMutableChunkManager(); 

        for (int u=0;u<nDepthWidth;u++){
            for (int v=0;v<nDepthHeight;v++){
                Vec3 mPosition(u , v , 1.0);
                Vec3 mPositionNorm = mDepthK.inverse() * mPosition;
                float nDepth = mDepthMap.at<float>(v , u);
                mPositionNorm = mPositionNorm * nDepth;
                Vec3 mPositionRef = mPose_rc * mPositionNorm;
                mPositionRef = Trunc(mPositionRef);
                ChunkID iIDRef = iManagerRef.GetIDAt(mPositionRef);

            
                    // mutex.lock();
                if (!iManagerRef.HasChunk(iIDRef))
                {
                    continue;            
                }


                ChunkPtr pSearchedChunk = iManagerRef.GetChunk(iIDRef);
                   
                Vec3 mPos = mPositionRef - pSearchedChunk->GetOrigin();

                Point3 mPoint = pSearchedChunk->GetVoxelCoords(mPos);
                VoxelID nID = pSearchedChunk->GetVoxelID(mPoint);

                DistVoxel & iVoxelToBeUpdated = pSearchedChunk->GetDistVoxelMutable(nID);
                float nSDF = iVoxelToBeUpdated.GetSDF();
                if (abs(nSDF) > 0.2){
                    mDepthMap.at<float>(v , u) = 1000.0;
                }

                if (u==200 && v==200){
                    cout << "SDF is: " << nSDF << endl;
                    cout << "Depth is: " << mDepthMap.at<float>(v , u) << endl;
                    cout << "mPositionRef is: " << endl << mPositionRef << endl;
                }
            }
        }
}

float SearchForChunkLength(
    Vec3 mPositionRef,
    Vec3 mDirection,
    Vec3 mStart, Vec3 mEnd, Vec3 mOrigin){

    //Update pChunkToBeUpdated
    Vec3 mPos = mPositionRef - mOrigin;
    // Vec3 mStart(0.0, 0.0, 0.0);
    // Vec3 mEnd = pSearchedChunk->GetNumVoxels().cast<float>() * pSearchedChunk->GetVoxelResolutionMeters();

    float nLeftLength = (mStart(0) - mPos(0)) / mDirection(0);
    float nRightLength = (mEnd(0) - mPos(0)) / mDirection(0);
    float nBackLength = (mStart(1) - mPos(1)) / mDirection(1);
    float nFrontLength = (mEnd(1) - mPos(1)) / mDirection(1);
    float nTopLength = (mStart(2) - mPos(2)) / mDirection(2);
    float nBottomLength = (mEnd(2) - mPos(2)) / mDirection(2);
    if (nLeftLength < 0){
        nLeftLength = 1000000;
    }
    if (nRightLength < 0){
        nRightLength = 1000000;
    }
    if (nBackLength < 0){
        nBackLength = 1000000;
    }
    if (nFrontLength < 0){
        nFrontLength = 1000000;
    }
    if (nTopLength < 0){
        nTopLength = 1000000;
    }
    if (nBottomLength < 0){
        nBottomLength = 1000000;
    }

    float nMinLength = nLeftLength;

    if (nMinLength > nRightLength){
        nMinLength = nRightLength;
    }

    if (nMinLength > nBackLength){
        nMinLength = nBackLength;
    }

    if (nMinLength > nFrontLength){
        nMinLength = nFrontLength;
    }

    if (nMinLength > nTopLength){
        nMinLength = nTopLength;
    }


    if (nMinLength > nBottomLength){
        nMinLength = nBottomLength;
    }
    float nFinalLength = nMinLength * mDirection.norm() +1e-4;
    return nFinalLength;

}


void RecoverDepthAndColorMapFast(
        chisel::ChiselPtr pChiselMapRef,
        Eigen::Matrix3f mDepthK,
        Eigen::Affine3f mPose_rc,
        int nDepthWidth, 
        int nDepthHeight,
        cv::Mat & mDepthMap,
        cv::Mat & mColorMap,
        chisel::TruncatorPtr pTruncator){
        // cout << "Enter depth map!" << endl;

        ChunkManager &iManagerRef = pChiselMapRef->GetMutableChunkManager(); 

        float nSearchStep = iManagerRef.GetResolution();
        float nMinDepth = 0.2;
        float nMaxDepth = 10;
        int nSearchTimes = (int)((nMaxDepth-nMinDepth) / nSearchStep);

        #pragma omp parallel for
        for (int u=0;u<nDepthWidth;u++){
            #pragma omp parallel for
            for (int v=0;v<nDepthHeight;v++){
                int nRed, nGreen, nBlue;

                float nCurrentDepth = nMinDepth;
                Vec3 mPosition(u , v , 1.0);
                Vec3 mPositionNorm = mDepthK.inverse() * mPosition;
                Vec3 mStartPosition = nMinDepth * mPositionNorm;
                mStartPosition = mPose_rc * mStartPosition;

                Vec3 mLine1 = mPositionNorm * 1.0;
                Vec3 mLine2 = mPositionNorm * 2.0;
                Vec3 mDirection = (mPose_rc * mLine2) - (mPose_rc * mLine1);
                mDirection = mDirection / mDirection.norm();
                //The length to skip one chunk.
                float nDefaultSeachLength = -1.0;
                bool bFirst = true;
                bool bGet = false;

                while (nCurrentDepth < nMaxDepth){

                    Vec3 mCurrentPosition = mPose_rc * (mPositionNorm * nCurrentDepth);

                    mCurrentPosition = Trunc(mCurrentPosition);
                    ChunkID iIDRef = iManagerRef.GetIDAt(mCurrentPosition);

                    if (!iManagerRef.HasChunk(iIDRef))
                    {
                        //Skip to next chunk.
                        if (bFirst){            
                            Vec3 mStart(0.0 , 0.0 , 0.0);
                            Vec3 mEnd = iManagerRef.GetChunkSize().cast<float>() * iManagerRef.GetResolution();
                            Vec3 mOrigin(iIDRef(0) * mEnd(0), iIDRef(1) * mEnd(1), iIDRef(2) * mEnd(2));
                            float nSeachLength = SearchForChunkLength( mCurrentPosition, mDirection, mStart, mEnd, mOrigin);    
                            nCurrentDepth += nSeachLength;
                        } else if (nDefaultSeachLength < 0){
                            Vec3 mStart(0.0 , 0.0 , 0.0);
                            Vec3 mEnd = iManagerRef.GetChunkSize().cast<float>() * iManagerRef.GetResolution();
                            Vec3 mOrigin(iIDRef(0) * mEnd(0), iIDRef(1) * mEnd(1), iIDRef(2) * mEnd(2));
                            float nSeachLength = SearchForChunkLength( mCurrentPosition, mDirection, mStart, mEnd, mOrigin);    
                            nDefaultSeachLength = nSeachLength;
                            nCurrentDepth += nSeachLength;
                        } else {
                            nCurrentDepth += nDefaultSeachLength;
                        }
                        continue;
                    }


                    //Seach by grids.

                    ChunkPtr pSearchedChunk = iManagerRef.GetChunk(iIDRef);

                    //Update pChunkToBeUpdated
                    Vec3 mPos = mCurrentPosition - pSearchedChunk->GetOrigin();



                    Point3 mPoint = pSearchedChunk->GetVoxelCoords(mPos);
                    Eigen::Vector3i mMin(0 , 0 , 0);
                    Eigen::Vector3i mMax = pSearchedChunk->GetNumVoxels();
                    mPoint = Trunc(mPoint, mMin, mMax);
                    VoxelID nID = pSearchedChunk->GetVoxelID(mPoint);
                    DistVoxel & iVoxelToBeUpdated = pSearchedChunk->GetDistVoxelMutable(nID);

                    ColorVoxel & iColorToBeUpdated = pSearchedChunk->GetColorVoxelMutable(nID);
                    bool bValid = iVoxelToBeUpdated.GetOccupied();
                    float nSDF = iVoxelToBeUpdated.GetSDF();
                    nRed = iColorToBeUpdated.GetRed();
                    nGreen = iColorToBeUpdated.GetGreen();
                    nBlue = iColorToBeUpdated.GetBlue();

                    if (!bValid){

                        //Get the search step
                        float nLocalSearchStep;
                        float nMaxStep = pTruncator->GetTruncationDistance(nCurrentDepth);
                        float nMinStep = nSearchStep;
                        if (nSDF > nMaxStep){
                            nLocalSearchStep = nMaxStep/4.0*3.0;
                        }else{
                            nLocalSearchStep = nSDF/4.0*3.0;
                        }
                        if (nLocalSearchStep < nMinStep){
                            nLocalSearchStep = nMinStep;
                        }

                        nCurrentDepth += nLocalSearchStep;
                        continue;
                    }
                    bGet = true;
                    nCurrentDepth = nCurrentDepth + nSDF;
                    
                    break;
                    

                }
                
                if (bGet){
                    mDepthMap.at<float>(v , u) = nCurrentDepth;
                    mColorMap.at<cv::Vec3b>(v , u)(2) = nRed;
                    mColorMap.at<cv::Vec3b>(v , u)(1) = nGreen;
                    mColorMap.at<cv::Vec3b>(v , u)(0) = nBlue;
                }else{
                    mDepthMap.at<float>(v , u) = 1000.0;
                    mColorMap.at<cv::Vec3b>(v , u)(2) = 0;
                    mColorMap.at<cv::Vec3b>(v , u)(1) = 0;
                    mColorMap.at<cv::Vec3b>(v , u)(0) = 0;
                }

            }
        }

}





void RecoverDepthMapFast(
        chisel::ChiselPtr pChiselMapRef,
        Eigen::Matrix3f mDepthK,
        Eigen::Affine3f mPose_rc,
        int nDepthWidth, 
        int nDepthHeight,
        cv::Mat & mDepthMap,
        chisel::TruncatorPtr pTruncator){
        // cout << "Enter depth map!" << endl;

        ChunkManager &iManagerRef = pChiselMapRef->GetMutableChunkManager(); 

        float nSearchStep = iManagerRef.GetResolution();
        float nMinDepth = 0.2;
        float nMaxDepth = 10;
        int nSearchTimes = (int)((nMaxDepth-nMinDepth) / nSearchStep);

        #pragma omp parallel for
        for (int u=0;u<nDepthWidth;u++){
            #pragma omp parallel for
            for (int v=0;v<nDepthHeight;v++){
                float nCurrentDepth = nMinDepth;
                Vec3 mPosition(u , v , 1.0);
                Vec3 mPositionNorm = mDepthK.inverse() * mPosition;
                Vec3 mStartPosition = nMinDepth * mPositionNorm;
                mStartPosition = mPose_rc * mStartPosition;

                Vec3 mLine1 = mPositionNorm * 1.0;
                Vec3 mLine2 = mPositionNorm * 2.0;
                Vec3 mDirection = (mPose_rc * mLine2) - (mPose_rc * mLine1);
                mDirection = mDirection / mDirection.norm();
                //The length to skip one chunk.
                float nDefaultSeachLength = -1.0;
                bool bFirst = true;
                bool bGet = false;

                while (nCurrentDepth < nMaxDepth){

                    Vec3 mCurrentPosition = mPose_rc * (mPositionNorm * nCurrentDepth);

                    mCurrentPosition = Trunc(mCurrentPosition);
                    ChunkID iIDRef = iManagerRef.GetIDAt(mCurrentPosition);

                    if (!iManagerRef.HasChunk(iIDRef))
                    {
                        //Skip to next chunk.
                        if (bFirst){            
                            Vec3 mStart(0.0 , 0.0 , 0.0);
                            Vec3 mEnd = iManagerRef.GetChunkSize().cast<float>() * iManagerRef.GetResolution();
                            Vec3 mOrigin(iIDRef(0) * mEnd(0), iIDRef(1) * mEnd(1), iIDRef(2) * mEnd(2));
                            float nSeachLength = SearchForChunkLength( mCurrentPosition, mDirection, mStart, mEnd, mOrigin);    
                            nCurrentDepth += nSeachLength;
                        } else if (nDefaultSeachLength < 0){
                            Vec3 mStart(0.0 , 0.0 , 0.0);
                            Vec3 mEnd = iManagerRef.GetChunkSize().cast<float>() * iManagerRef.GetResolution();
                            Vec3 mOrigin(iIDRef(0) * mEnd(0), iIDRef(1) * mEnd(1), iIDRef(2) * mEnd(2));
                            float nSeachLength = SearchForChunkLength( mCurrentPosition, mDirection, mStart, mEnd, mOrigin);    
                            nDefaultSeachLength = nSeachLength;
                            nCurrentDepth += nSeachLength;
                        } else {
                            nCurrentDepth += nDefaultSeachLength;
                        }
                        continue;
                    }


                    //Seach by grids.

                    ChunkPtr pSearchedChunk = iManagerRef.GetChunk(iIDRef);

                    //Update pChunkToBeUpdated
                    Vec3 mPos = mCurrentPosition - pSearchedChunk->GetOrigin();



                    Point3 mPoint = pSearchedChunk->GetVoxelCoords(mPos);
                    Eigen::Vector3i mMin(0 , 0 , 0);
                    Eigen::Vector3i mMax = pSearchedChunk->GetNumVoxels();
                    mPoint = Trunc(mPoint, mMin, mMax);
                    VoxelID nID = pSearchedChunk->GetVoxelID(mPoint);
                    DistVoxel & iVoxelToBeUpdated = pSearchedChunk->GetDistVoxelMutable(nID);
                    bool bValid = iVoxelToBeUpdated.GetOccupied();
                    float nSDF = iVoxelToBeUpdated.GetSDF();

                    // Point3 mPoint = pSearchedChunk->GetVoxelCoords(mPos);
           //          Point3 mPointLeft = mPoint - Point3(1 , 0 , 0);
           //          Point3 mPointRight = mPoint + Point3(1 , 0 , 0);
           //          Point3 mPointFront = mPoint + Point3(0 , 1 , 0);
           //          Point3 mPointBack = mPoint - Point3(0 , 1 , 0);
           //          Point3 mPointTop = mPoint + Point3(0 , 0 , 1);
           //          Point3 mPointBottom = mPoint - Point3(0 , 0 , 1);

           //          Eigen::Vector3i mMin(0 , 0 , 0);
           //          Eigen::Vector3i mMax = pSearchedChunk->GetNumVoxels();

           //          Point3 mUntrunc = mPointRight;


           //          mPoint = Trunc(mPoint, mMin, mMax);
           //          // mPointLeft = Trunc(mPointLeft, mMin, mMax);
           //          // mPointRight = Trunc(mPointRight, mMin, mMax);
           //          // mPointFront = Trunc(mPointFront, mMin, mMax);
           //          // mPointBack = Trunc(mPointBack, mMin, mMax);
           //          // mPointTop = Trunc(mPointTop, mMin, mMax);
           //          // mPointBottom = Trunc(mPointBottom, mMin, mMax);



           //          VoxelID nID = pSearchedChunk->GetVoxelID(mPoint); 
           //          VoxelID nLeftID = pSearchedChunk->GetVoxelID(mPointLeft);
           //          VoxelID nRightID = pSearchedChunk->GetVoxelID(mPointRight);
           //          VoxelID nFrontID = pSearchedChunk->GetVoxelID(mPointFront);
           //          VoxelID nBackID = pSearchedChunk->GetVoxelID(mPointBack);
           //          VoxelID nTopID = pSearchedChunk->GetVoxelID(mPointTop);
           //          VoxelID nBottomID = pSearchedChunk->GetVoxelID(mPointBottom);

           //          std::vector<VoxelID> gIDs {nID, nLeftID, nRightID, nFrontID, nBackID, nTopID, nBottomID};
           
            
           //          bool bValid = true;


           //          for (int ii=0;ii<gIDs.size();ii++){
           //              if (gIDs[ii] < 0 || gIDs[ii] >= pSearchedChunk->GetTotalNumVoxels()){
           //                  continue;
           //              }


           //              DistVoxel & iVoxelToBeUpdated = pSearchedChunk->GetDistVoxelMutable(gIDs[ii]);
           //              float nSDF = iVoxelToBeUpdated.GetSDF();
              //           float nWeight = iVoxelToBeUpdated.GetWeight();


              //           if (abs(nSDF) > 0.1 || nWeight < 0.5){
              //            bValid = false;
              //            break;
              //           }         

           //          }

           //          DistVoxel & iVoxelCenter = pSearchedChunk->GetDistVoxelMutable(gIDs[0]);

                    if (!bValid){

                        //Get the search step
                        float nLocalSearchStep;
                        float nMaxStep = pTruncator->GetTruncationDistance(nCurrentDepth);
                        float nMinStep = nSearchStep;
                        if (nSDF > nMaxStep){
                            nLocalSearchStep = nMaxStep/4.0*3.0;
                        }else{
                            nLocalSearchStep = nSDF/4.0*3.0;
                        }
                        if (nLocalSearchStep < nMinStep){
                            nLocalSearchStep = nMinStep;
                        }

                        nCurrentDepth += nLocalSearchStep;
                        continue;
                    }
                    bGet = true;
                    nCurrentDepth = nCurrentDepth + nSDF;
                    // if (!iVoxelCenter.GetOccupied()){
                    //  cout << "Wrong!" << endl;
                    //  cout << "ChunkID: " << endl << iIDRef << endl;
                    //  cout << "Voxel ID: " << nID << endl;
                    // }
                    break;
                    

                }
                
                if (bGet){
                    mDepthMap.at<float>(v , u) = nCurrentDepth;
                }else{
                    mDepthMap.at<float>(v , u) = 1000.0;
                }

            }
        }

}




void RecoverDepthMap(
        chisel::ChiselPtr pChiselMapRef,
        Eigen::Matrix3f mDepthK,
        Eigen::Affine3f mPose_rc,
        int nDepthWidth, 
        int nDepthHeight,
        cv::Mat & mDepthMap){
        // cout << "Enter depth map!" << endl;

        float nSearchStep = 0.03;
        float nMinDepth = 0.2;
        float nMaxDepth = 10;
        int nSearchTimes = (int)((nMaxDepth-nMinDepth) / nSearchStep);
        ChunkManager &iManagerRef = pChiselMapRef->GetMutableChunkManager(); 


        #pragma omp parallel for
        for (int u=0;u<nDepthWidth;u++){
            #pragma omp parallel for
            for (int v=0;v<nDepthHeight;v++){
                Vec3 mPosition(u , v , 1.0);
                int nGetSS = -1;
                for (int ss=0;ss < nSearchTimes; ss++){

                    Vec3 mPositionNorm = mDepthK.inverse() * mPosition;
                    float nEstimatedDepth = (ss*nSearchStep + nMinDepth);


                    mPositionNorm = mPositionNorm * (ss*nSearchStep + nMinDepth);
                    Vec3 mPositionRef = mPose_rc * mPositionNorm;
                    mPositionRef = Trunc(mPositionRef);

                    ChunkID iIDRef = iManagerRef.GetIDAt(mPositionRef);


                    if (!iManagerRef.HasChunk(iIDRef))
                    {
                        continue;            
                    }




                    ChunkPtr pSearchedChunk = iManagerRef.GetChunk(iIDRef);


                    //Update pChunkToBeUpdated
                    Vec3 mPos = mPositionRef - pSearchedChunk->GetOrigin();

                    mPos = Trunc(mPos);


                    Point3 mPoint = pSearchedChunk->GetVoxelCoords(mPos);
                    Point3 mPointLeft = mPoint - Point3(1 , 0 , 0);
                    Point3 mPointRight = mPoint + Point3(1 , 0 , 0);
                    Point3 mPointFront = mPoint + Point3(0 , 1 , 0);
                    Point3 mPointBack = mPoint - Point3(0 , 1 , 0);
                    Point3 mPointTop = mPoint + Point3(0 , 0 , 1);
                    Point3 mPointBottom = mPoint - Point3(0 , 0 , 1);

                    Eigen::Vector3i mMin(0 , 0 , 0);
                    Eigen::Vector3i mMax = pSearchedChunk->GetNumVoxels();


                    mPoint = Trunc(mPoint, mMin, mMax);
                    mPointLeft = Trunc(mPointLeft, mMin, mMax);
                    mPointRight = Trunc(mPointRight, mMin, mMax);
                    mPointFront = Trunc(mPointFront, mMin, mMax);
                    mPointBack = Trunc(mPointBack, mMin, mMax);
                    mPointTop = Trunc(mPointTop, mMin, mMax);
                    mPointBottom = Trunc(mPointBottom, mMin, mMax);


                    VoxelID nID = pSearchedChunk->GetVoxelID(mPoint); 
                    VoxelID nLeftID = pSearchedChunk->GetVoxelID(mPointLeft);
                    VoxelID nRightID = pSearchedChunk->GetVoxelID(mPointRight);
                    VoxelID nFrontID = pSearchedChunk->GetVoxelID(mPointFront);
                    VoxelID nBackID = pSearchedChunk->GetVoxelID(mPointBack);
                    VoxelID nTopID = pSearchedChunk->GetVoxelID(mPointTop);
                    VoxelID nBottomID = pSearchedChunk->GetVoxelID(mPointBottom);

                    std::vector<VoxelID> gIDs {nID, nLeftID, nRightID, nFrontID, nBackID, nTopID, nBottomID};
           
            
                    bool bValid = true;
                    for (int ii=0;ii<gIDs.size();ii++){
                        if (gIDs[ii] < 0 || gIDs[ii] >= pSearchedChunk->GetTotalNumVoxels()){
                            continue;
                        }

                        DistVoxel & iVoxelToBeUpdated = pSearchedChunk->GetDistVoxelMutable(gIDs[ii]);
                        float nSDF = iVoxelToBeUpdated.GetSDF();
                        float nWeight = iVoxelToBeUpdated.GetWeight();

                        if (abs(nSDF) > 0.1 || nWeight < 0.5){
                            bValid = false;
                            break;
                        }        

                    }
                    if (bValid){
                        nGetSS = ss;

                        break;
                    }

                }

                if (u==100 && v==100){
                    cout << "SS is: " << nGetSS << endl;
                    cout << "Depth is: " << nGetSS * nSearchStep + nMinDepth << endl;
                }

                if (nGetSS > 0){
                    mDepthMap.at<float>(v , u) = nGetSS * nSearchStep + nMinDepth;
                }else{
                    mDepthMap.at<float>(v , u) = 1000.0;
                }

            }
        }

}



bool MergeChunks(
    chisel::ChiselPtr pChiselMapRef,
    chisel::ChiselPtr pChiselMapMerge,
    Eigen::Affine3f mPose_rm){


    ChunkManager &iManagerRef = pChiselMapRef->GetMutableChunkManager(); 
    ChunkManager &iManagerMerge = pChiselMapMerge->GetMutableChunkManager(); 

    //Get all chunks of the merge manager
    ChunkMap iMap = iManagerMerge.GetChunks();
    ChunkMap::iterator pIter;
    vector<ChunkPtr> gChunksToBeMerged;
    gChunksToBeMerged.reserve(iMap.size());
    for (pIter = iMap.begin(); pIter != iMap.end(); pIter++){
        ChunkPtr pChunk = pIter->second;
        gChunksToBeMerged.push_back(pChunk);
    }

    Vec3List gCentroids = iManagerMerge.GetCentroids();


    ChunkIDList gGarbageChunks;
    //Merge chunks one by one.
    for (ChunkPtr pChunk : gChunksToBeMerged){
        float nResolution = pChunk->GetVoxelResolutionMeters();
        Vec3 mOrigin = pChunk->GetOrigin();
        float nResolutionDiagonal = 2.0 * sqrt(3.0f) * nResolution;
        bool bUpdated = false;

        for (size_t i = 0; i < gCentroids.size(); i++)
        //parallel_for(indexes.begin(), indexes.end(), [&](const size_t& i)
        {
            DistVoxel &iVoxelMerged = pChunk->GetDistVoxelMutable(i);
            if (iVoxelMerged.GetWeight()  < 0.001){
                continue;
            }
            ColorVoxel &iVoxelMergedColor = pChunk->GetColorVoxelMutable(i);

            //World coord of the voxel.
            Vec3 mVoxelCenter = gCentroids[i] + mOrigin;
            Vec3 mVoxelCenterRef = mPose_rm * mVoxelCenter;

            //Find the corresponding voxel in iManagerRef
            ChunkID iIDRef = iManagerRef.GetIDAt(mVoxelCenterRef);
            bool bChunkNew = false;
            // mutex.lock();
            if (!iManagerRef.HasChunk(iIDRef))
            {
                bChunkNew = true;
                iManagerRef.CreateChunk(iIDRef);
                // cout << "Updated Chunk size: " << pChiselMapRef->GetMutableChunkManager().GetChunks().size() << endl;
                // cout << "Updated Chunk size2: " << iManagerRef.GetChunks().size() << endl;
                
    
            }
            ChunkPtr pChunkToBeUpdated = iManagerRef.GetChunk(iIDRef);
            // mutex.unlock();

            //Update pChunkToBeUpdated
            Vec3 mPos = mVoxelCenterRef - pChunkToBeUpdated->GetOrigin();
            Point3 mPoint = pChunkToBeUpdated->GetVoxelCoords(mPos);
            Point3 mPointLeft = mPoint - Point3(1 , 0 , 0);
            Point3 mPointRight = mPoint + Point3(1 , 0 , 0);
            Point3 mPointFront = mPoint + Point3(0 , 1 , 0);
            Point3 mPointBack = mPoint - Point3(0 , 1 , 0);
            Point3 mPointTop = mPoint + Point3(0 , 0 , 1);
            Point3 mPointBottom = mPoint - Point3(0 , 0 , 1);

            Vec3 mPosCenter = pChunkToBeUpdated->GetVoxelPoints(mPoint);
            Vec3 mPosLeft = pChunkToBeUpdated->GetVoxelPoints(mPointLeft);
            Vec3 mPosRight = pChunkToBeUpdated->GetVoxelPoints(mPointRight);
            Vec3 mPosFront = pChunkToBeUpdated->GetVoxelPoints(mPointFront);
            Vec3 mPosBack = pChunkToBeUpdated->GetVoxelPoints(mPointBack);
            Vec3 mPosTop = pChunkToBeUpdated->GetVoxelPoints(mPointTop);
            Vec3 mPosBottom = pChunkToBeUpdated->GetVoxelPoints(mPointBottom);

            // float nDistCenter = Norms(mPos - mPosCenter);
            // float nDistLeft = Norms(mPos - mPosLeft);
            // float nDistRight = Norms(mPos - mPosRight);
            // float nDistFront = Norms(mPos - mPosFront);
            // float nDistBack = Norms(mPos - mPosBack);
            // float nDistTop = Norms(mPos - mPosTop);
            // float nDistBottom = Norms(mPos - mPosBottom);


            // float nTotalDist =  nDistCenter + nDistLeft + nDistRight + 
            //                     nDistFront + nDistBack + nDistTop + nDistBottom;

            float nWeight = iVoxelMerged.GetWeight();


            VoxelID nID = pChunkToBeUpdated->GetVoxelID(mPoint);
            VoxelID nLeftID = pChunkToBeUpdated->GetVoxelID(mPointLeft);
            VoxelID nRightID = pChunkToBeUpdated->GetVoxelID(mPointRight);
            VoxelID nFrontID = pChunkToBeUpdated->GetVoxelID(mPointFront);
            VoxelID nBackID = pChunkToBeUpdated->GetVoxelID(mPointBack);
            VoxelID nTopID = pChunkToBeUpdated->GetVoxelID(mPointTop);
            VoxelID nBottomID = pChunkToBeUpdated->GetVoxelID(mPointBottom);

            std::vector<VoxelID> gIDs {nID, nLeftID, nRightID, nFrontID, nBackID, nTopID, nBottomID};
           
            

            for (int ii=0;ii<gIDs.size();ii++){
                if (gIDs[ii] < 0 || gIDs[ii] >= pChunkToBeUpdated->GetTotalNumVoxels()){
                    continue;
                }

                DistVoxel & iVoxelToBeUpdated = pChunkToBeUpdated->GetDistVoxelMutable(gIDs[ii]);
                ColorVoxel & iVoxelToBeUpdatedColor = pChunkToBeUpdated->GetColorVoxelMutable(gIDs[ii]);

                //Update iVoxelToBeUpdated.
                iVoxelToBeUpdated.Integrate(iVoxelMerged.GetSDF(), iVoxelMerged.GetWeight());
                iVoxelToBeUpdatedColor.Integrate(iVoxelMergedColor.GetRed(), iVoxelMergedColor.GetGreen(), iVoxelMergedColor.GetBlue(), iVoxelMergedColor.GetWeight());
            }


            bool bNeedUpdate = true;

            // mutex.lock();
            if (bNeedUpdate)
            {
                ChunkSet & iSet = pChiselMapRef->GetMutableMeshesToUpdate();
                for (int dx = -1; dx <= 1; dx++)
                {
                    for (int dy = -1; dy <= 1; dy++)
                    {
                        for (int dz = -1; dz <= 1; dz++)
                        {
                            iSet[iIDRef + ChunkID(dx, dy, dz)] = true;
                        }
                    }
                }
            }
            // else if(bChunkNew)
            // {
            //     gGarbageChunks.push_back(iIDRef);
            // }
            // mutex.unlock();
           
        }

    }
    // pChiselMapRef->GarbageCollect(gGarbageChunks);


    return true;

}



}



ChiselWrapper::ChiselWrapper(	Eigen::Matrix3d mK, Eigen::Matrix3d mDepthK, int nWidth, int nHeight,
				Eigen::Affine3d mRefPose_wc, bool bColor, float nResolution, int nChunkSize,
				float nTruncationDistScale, bool bUseCarving, float nCarvingDist,
				float nWeight, float nNearDist, float nFarDist,
				float nMinWeight) :
	m_bIntegrateColor(bColor), m_nResolution(nResolution), m_nChunkSize(nChunkSize),
	m_nTruncationDistScale(nTruncationDistScale), m_bUseCarving(bUseCarving), m_nWeight(nWeight),
	m_nCarvingDist(nCarvingDist), m_nMinWeight(nMinWeight), m_nNearDist(nNearDist), m_nFarDist(nFarDist),
	m_mK(mK), m_mDepthK(mDepthK), m_nWidth(nWidth), m_nHeight(nHeight), m_mRefPose_wc(mRefPose_wc)
{
	this->m_pCurrentChiselMap.reset(new chisel::Chisel(Eigen::Vector3i(nChunkSize, nChunkSize, nChunkSize), nResolution, bColor));
	this->m_gChisels.push_back(this->m_pCurrentChiselMap);
	this->m_gPoses.push_back(mRefPose_wc);
	this->m_pTruncator.reset(new chisel::InverseTruncator(nTruncationDistScale));
	this->SetupIntegrator(this->m_pCurrentChiselMap, this->m_pTruncator, nWeight, bUseCarving, nCarvingDist);
	this->m_pDepthImage.reset(new chisel::DepthImage<DepthData>(nWidth, nHeight));
    this->m_pColorImage.reset(new chisel::ColorImage<ColorData>(nWidth, nHeight, 3));

    //Generate camera model.
    float nFocalX = mK(0 , 0);
    float nFocalY = mK(1 , 1);
    float nPrincipalX = mK(0 , 2);
    float nPrincipalY = mK(1 , 2);

    float nDepthFocalX = mDepthK(0 , 0);
    float nDepthFocalY = mDepthK(1 , 1);
    float nDepthPrincipalX = mDepthK(0 , 2);
    float nDepthPrincipalY = mDepthK(1 , 2);

    this->m_iColorCamera = GenerateCamera(
		nFocalX, nFocalY, 
		nPrincipalX, nPrincipalY,
		nWidth, nHeight);

	this->m_iDepthCamera = GenerateCamera(
		nDepthFocalX, nDepthFocalY, 
		nDepthPrincipalX, nDepthPrincipalY,
		nWidth, nHeight);


    this->m_iColorCamera.SetNearPlane(nNearDist);
    this->m_iColorCamera.SetFarPlane(nFarDist);
    this->m_iDepthCamera.SetNearPlane(nNearDist);
    this->m_iDepthCamera.SetFarPlane(nFarDist);

    this->m_bNewSubMap = false;

}

cv::Size ChiselWrapper::GetSize(){
	return cv::Size(this->m_nWidth, this->m_nHeight);
}

//FC32  
bool ChiselWrapper::Integrate(cv::Mat & mDepthImage, cv::Mat mColorImage, Eigen::Affine3d & mPose_wc, int nPublishIndex){
	this->m_pDepthImage.reset(new chisel::DepthImage<DepthData>(this->m_nWidth, this->m_nHeight));
	this->m_pColorImage.reset(new chisel::ColorImage<ColorData>(this->m_nWidth, this->m_nHeight, 3));

	// cv::Size iSize(this->m_nWidth, this->m_nHeight);

	// cv::resize(mDepthImage, mDepthImage, iSize);
	// cv::resize(mColorImage, mColorImage, iSize);
	

	CVToDepthMap(mDepthImage, this->m_pDepthImage.get());
	CVToColorMap(mColorImage, this->m_pColorImage.get());

	Eigen::Affine3d mRelPose_wc = this->m_mRefPose_wc.inverse() * mPose_wc;

	chisel::Transform iTransform_wc = GenerateTransform(mRelPose_wc);


	this->m_pCurrentChiselMap->IntegrateDepthScanColor<DepthData, ColorData>(
					*(this->m_pProjectionIntegrator), 
					this->m_pDepthImage, iTransform_wc, this->m_iDepthCamera, 
					this->m_pColorImage, iTransform_wc, this->m_iColorCamera);

    this->m_gPublishIndices.push_back(nPublishIndex);
}


chisel::ProjectionIntegrator * ChiselWrapper::SetupIntegrator(
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


void ChiselWrapper::SaveMesh(string aFilename){
	this->m_pCurrentChiselMap->UpdateMeshesInstantly();
	this->m_pCurrentChiselMap->SaveAllMeshesToPLY(aFilename);
}

void ChiselWrapper::SaveRefMesh(string aFilename){
	if (this->m_gPoses.size()<1){
		return;
	}
	chisel::ChiselPtr pRefChisel = this->m_gChisels[0];	
	pRefChisel->UpdateMeshesInstantly();
	pRefChisel->SaveAllMeshesToPLY(aFilename);
}

void ChiselWrapper::GenerateNewSubMap(	Eigen::Affine3d mRefPose_wc,
						bool bColor, float nResolution, int nChunkSize,
						float nTruncationDistScale, bool bUseCarving, float nCarvingDist,
						float nWeight, float nNearDist, float nFarDist,
						float nMinWeight){
	int nWidth = this->m_nWidth;
	int nHeight = this->m_nHeight;
	this->m_mRefPose_wc = mRefPose_wc;
	this->m_gPoses.push_back(mRefPose_wc);

	this->m_pCurrentChiselMap.reset(new chisel::Chisel(Eigen::Vector3i(nChunkSize, nChunkSize, nChunkSize), nResolution, bColor));
	this->m_gChisels.push_back(this->m_pCurrentChiselMap);
	this->m_pTruncator.reset(new chisel::InverseTruncator(nTruncationDistScale));
	this->SetupIntegrator(this->m_pCurrentChiselMap, this->m_pTruncator, nWeight, bUseCarving, nCarvingDist);
	this->m_pDepthImage.reset(new chisel::DepthImage<DepthData>(nWidth, nHeight));
    this->m_pColorImage.reset(new chisel::ColorImage<ColorData>(nWidth, nHeight, 3));
    this->m_gPublishIndices.clear();
}


void ChiselWrapper::MergeChunks(){
	if (this->m_gPoses.size()<2){
		return;
	}
	chisel::ChiselPtr pRefChisel = this->m_gChisels[0];	
	Eigen::Affine3d mRefPose_wc = this->m_gPoses[0];
	for (int i=1;i<this->m_gPoses.size();i++){
		chisel::ChiselPtr pMergeChisel = this->m_gChisels[i];	
		Eigen::Affine3d mMergePose_wc = this->m_gPoses[i];
		Eigen::Affine3d mPose_rm = mRefPose_wc.inverse() * mMergePose_wc;
		Eigen::Affine3f mFloatPose_rm = mPose_rm.cast<float>();
		cout << "Start to merge chunk: " << i << endl;
		chisel::MergeChunks(pRefChisel, pMergeChisel, mFloatPose_rm);

	}
}
