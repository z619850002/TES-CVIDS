
#include "../include/server_keyframe.h"
using namespace std;

template <typename Derived>
static void reduceVector(vector<Derived> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


//Constructor without image.
ServerKeyFrame::ServerKeyFrame(	double nTimeStamp, int nClient, int nLocalIndex, 
								Sophus::SE3 mVIOPose,
								Eigen::Matrix3d mRotation_ic,
    							Eigen::Vector3d mTranslation_ic, 
			 					vector<cv::Point3f> & gWindowPoints3D, 
			 					vector<cv::Point2f> & gWindowPoints2D, 
			 					vector<int> & gWindowPointsID,
			 					vector<DVision::BRIEF::bitset> & gWindowDescriptors,
			 					vector<cv::Point2f> & gProjectedPoints2D, 
			 					vector<DVision::BRIEF::bitset> & gProjectedDescriptors,
			 					bool bFromVisionSLAM,
			 					ServerCamera * pCamera){


	this->m_bHasLoop = false;
	this->m_bFreeSpace = false;

	this->m_pServerCamera = pCamera;

	//Set the identifier.
	this->m_nTimeStamp = nTimeStamp;
	this->m_nClientID = nClient;
	this->m_nLocalIndex = nLocalIndex;
	//FIXME: The index here should be modified in the future.
	this->m_nGlobalIndex = nLocalIndex;

	//Set the pose.
	this->m_mLocalPose_iw = mVIOPose;
	this->m_mLocalTranslation_wi = mVIOPose.inverse().translation();
	this->m_mLocalRotation_wi = mVIOPose.inverse().rotation_matrix();
	//In the begining the global pose should be set to the same to the local pose.
	this->m_mGlobalPose_iw = this->m_mLocalPose_iw;
	this->m_mGlobalTranslation_wi = this->m_mLocalTranslation_wi;
	this->m_mGlobalRotation_wi = this->m_mLocalRotation_wi;


	this->m_mLocalR_Backup_wi = this->m_mLocalRotation_wi;
	this->m_mLocalT_Backup_wi = this->m_mLocalTranslation_wi;
	


	//Set the extrinsics.
	this->m_mRotation_ic = mRotation_ic;
	this->m_mTranslation_ic = mTranslation_ic;

	
	//If mappoints can be obtained.
	this->m_bFromVisionSLAM = bFromVisionSLAM;


	//Copy those map points.
	if (bFromVisionSLAM){
		this->m_gPoints3D = gWindowPoints3D;
		this->m_gPoints2D = gWindowPoints2D;
		this->m_gWindowBriefDescriptors = gWindowDescriptors;
		//Compute the normalized point.
		this->m_gNormalizedWindowPoints.reserve(gWindowPoints2D.size());
		//Add normalized points.
		for (auto iPoint2D : gWindowPoints2D){
			Eigen::Vector2d mPointPixel(iPoint2D.x, iPoint2D.y);
			Eigen::Vector3d mTempPoint = this->m_pServerCamera->LiftProject(mPointPixel);
			this->m_gNormalizedWindowPoints.push_back(cv::Point2f(mTempPoint.x()/mTempPoint.z(), mTempPoint.y()/mTempPoint.z()));
		}
		//ID of map points.
		this->m_gPointsID = gWindowPointsID;	
	}

	
	//Copy of those points to be projected.
	this->m_gProjectedPoints = gProjectedPoints2D;
	this->m_gBriefDescriptors = gProjectedDescriptors;
	//Add normalized points.
	for (auto iPoint2D : gProjectedPoints2D){
		Eigen::Vector2d mPointPixel(iPoint2D.x, iPoint2D.y);
		Eigen::Vector3d mTempPoint = this->m_pServerCamera->LiftProject(mPointPixel);
		this->m_gNormalizedProjectedPoints.push_back(cv::Point2f(mTempPoint.x()/mTempPoint.z(), mTempPoint.y()/mTempPoint.z()));
	}



	//Info about the loop closure.
	this->m_nLoopIndex = -1;

	this->m_mLoopInfo << 0, 0, 0, 0, 0, 0, 0, 0;


	this->m_pPreviousKeyFrame = NULL;
	this->m_pNextKeyFrame = NULL;

	// cout << "Create Debug!" << endl;	
	
	// vector<cv::Point2f> gCurrentPoints2D, gOldPoints2D;
	// vector<cv::Point2f> gCurrentNorm2D, gOldNorm2D;
	// vector<cv::Point3f> gMatchedPoints3D;
	// vector<int> gMatchedID;
	// vector<uchar> gStatus;

	// gMatchedPoints3D = this->m_gPoints3D;
	// gCurrentPoints2D = this->m_gPoints2D;
	// gCurrentNorm2D = this->m_gNormalizedWindowPoints;
	// gMatchedID = this->m_gPointsID;



	// Eigen::Matrix3d mRotationDebug;
	// Eigen::Vector3d mTranslationDebug;


	// PnPRANSAC(gCurrentNorm2D, gMatchedPoints3D, gStatus, mTranslationDebug, mRotationDebug);
	// cout << "Debug rotation is: " << endl << mRotationDebug << endl;
	// cout << "Debug translation is: " << endl << mTranslationDebug << endl;
	// cout << "GT rotation is: " << endl << m_mLocalRotation_wi << endl;
	// cout << "GT translation is: " << endl << m_mLocalTranslation_wi << endl;


	//Initialize the depth estimator.
	//Firstly compute the pose wc
	this->m_bFinalizeDepthMap = false;


}




//Find one matched point from the old descriptors.
bool ServerKeyFrame::SearchInArea(		const DVision::BRIEF::bitset iWindowDescriptor,
                            			const vector<DVision::BRIEF::bitset> & gOldDescriptors,
                            			const vector<cv::Point2f> & gOldPoints,
                            			const vector<cv::Point2f> & gNormOldPoints,
                            			cv::Point2f & iBestMatchedPoint,
                            			cv::Point2f & iBestMatchedNormPoint){
    int nBestDistance = 128;
    int nSecondBestDistance = 128;
    bool bFirstBest = true;
    int nBestIndex = -1;
    for(int i = 0; i < (int)gOldDescriptors.size(); i++)
    {
        int nDistance = this->HammingDistance(iWindowDescriptor, gOldDescriptors[i]);
        if(nDistance < nBestDistance)
        {
        	if (bFirstBest){
        		bFirstBest = false;
        	}else{
        		nSecondBestDistance = nBestDistance;
        	}
            nBestDistance = nDistance;
            nBestIndex = i;
        }
    }
    //printf("best dist %d", bestDist);
    if (nBestIndex != -1 && nBestDistance < 80 && nBestDistance < 0.7 * nSecondBestDistance)
    {
      iBestMatchedPoint = gOldPoints[nBestIndex];
      iBestMatchedNormPoint = gNormOldPoints[nBestIndex];
      return true;
    }
    else
      return false;
}



int ServerKeyFrame::HammingDistance(	const DVision::BRIEF::bitset & iDescriptorA, 
										const DVision::BRIEF::bitset & iDescriptorB)
{
    DVision::BRIEF::bitset iOrDescript  = iDescriptorA ^ iDescriptorB;
    int nDistance = iOrDescript.count();
    return nDistance;
}



//Find correspondences of points in this frame and another frame.
void ServerKeyFrame::SearchByBRIEFDes(	const vector<DVision::BRIEF::bitset> & gOldDescriptors,
                                		const vector<cv::Point2f> & gOldPoints,
                                		const vector<cv::Point2f> & gOldNormPoints,
                                		vector<uchar> & gStatus,
										vector<cv::Point2f> & gMatchedPoints2D,
										vector<cv::Point2f> & gNormMatchedPoints2D)
{
	//Reserve
	gStatus.reserve(m_gWindowBriefDescriptors.size());
	gMatchedPoints2D.reserve(m_gWindowBriefDescriptors.size());
	gNormMatchedPoints2D.reserve(m_gWindowBriefDescriptors.size());

    for(int i = 0; i < (int)this->m_gWindowBriefDescriptors.size(); i++)
    {
        cv::Point2f iPoint2D(0.f, 0.f);
        cv::Point2f iNormPoint2D(0.f, 0.f);
        if (this->SearchInArea(this->m_gWindowBriefDescriptors[i], gOldDescriptors, gOldPoints, gOldNormPoints, iPoint2D, iNormPoint2D)){
          gStatus.push_back(1);
        }
        else{
          gStatus.push_back(0);
        }
        gMatchedPoints2D.push_back(iPoint2D);
        gNormMatchedPoints2D.push_back(iNormPoint2D);
    }

}


//Not used yet.
void ServerKeyFrame::FundmantalMatrixRANSAC(const vector<cv::Point2f> & gCurrentNormPoints,
                                    		const vector<cv::Point2f> & gOldNormPoints,
                                    		vector<uchar> & gStatus)
{
	int nPointsNumber = (int) gCurrentNormPoints.size();
	gStatus.reserve(nPointsNumber);
	for (int i = 0; i < nPointsNumber; i++){
		gStatus.push_back(0);
	}
	//Only with more than 8 points the algorithm can be activated.
    if (nPointsNumber >= 8)
    {
        vector<cv::Point2f> gTempCurrent(nPointsNumber), gTempOld(nPointsNumber);
        for (int i = 0; i < (int)nPointsNumber; i++)
        {
        	//TODO: The focal length has been modified!
        	// Eigen::Matrix3d mK = this->m_pServerCamera->GetK();

            //Convert to pixel coordinate!!!
            Eigen::Vector3d mCurrentNormPoint(gCurrentNormPoints[i].x, gCurrentNormPoints[i].y, 1.0);
            Eigen::Vector3d mOldNormPoint(gOldNormPoints[i].x, gOldNormPoints[i].y, 1.0);

            Eigen::Vector2d mCurrentPixel = this->m_pServerCamera->Project(mCurrentNormPoint);
            Eigen::Vector2d mOldPixel = this->m_pServerCamera->Project(mOldNormPoint);

            gTempCurrent[i] = cv::Point2f(mCurrentPixel.x(), mCurrentPixel.y());
            gTempOld[i] = cv::Point2f(mOldPixel.x(), mOldPixel.y());
            
        }
        cv::findFundamentalMat(gTempCurrent, gTempOld, cv::FM_RANSAC, 3.0, 0.9, gStatus);
    }
}


//Maybe something wrong in this function??
//The guessed keyframe should be old keyframe's
void ServerKeyFrame::PnPRANSAC(		const vector<cv::Point2f> & gOldNormPoints2D,
                        			const vector<cv::Point3f> & gMatchedPoints3D,
                        			vector<uchar> & gStatus,
                        			Eigen::Vector3d & mOldTranslation_wi, 
                        			Eigen::Matrix3d & mOldRotation_wi)
{
    cv::Mat mRotationCV, mRotationVecCV, mTranslationCV, mDistortionCV, mTempRotationCV;
    //The 2d points are normalized, thus the intrinsic matrix should be set to identity matrix.
    cv::Mat mK = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
    mDistortionCV = (cv::Mat_<double>(4 , 1) << 0.0, 0.0, 0.0, 0.0);
    
    //Extrinsics
    Eigen::Matrix3d mRotation_ic = this->m_mRotation_ic;
    Eigen::Vector3d mTranslation_ic = this->m_mTranslation_ic;

    //Get the pose of the camera.
    //Current Frame
    Eigen::Matrix3d mRotation_wc = this->m_mGlobalRotation_wi * mRotation_ic;
    Eigen::Vector3d mTranslation_wc = this->m_mGlobalTranslation_wi + this->m_mGlobalRotation_wi * mTranslation_ic;

    //Now we obtain the pose of the camera.
    //The pose of current frame
    Eigen::Matrix3d mInitialRotation_cw = mRotation_wc.inverse();
    Eigen::Vector3d mInitialTranslation_cw = -(mInitialRotation_cw * mTranslation_wc);
    //Convert them to cv type.
    cv::eigen2cv(mInitialRotation_cw, mTempRotationCV);
    cv::Rodrigues(mTempRotationCV, mRotationVecCV);
    cv::eigen2cv(mInitialTranslation_cw, mTranslationCV);

    cv::Mat mInliers;

    //FIXME:Change the initial value of the pose.
    //Use PnP to find outliers.
    if (CV_MAJOR_VERSION < 3){
        cv::solvePnPRansac(gMatchedPoints3D, gOldNormPoints2D, mK, mDistortionCV, mRotationVecCV, mTranslationCV, false, 100, 10.0 / 460.0, 100, mInliers);
    }
    else
    {
        if (CV_MINOR_VERSION < 2){
            cv::solvePnPRansac(gMatchedPoints3D, gOldNormPoints2D, mK, mDistortionCV, mRotationVecCV, mTranslationCV, false, 100, sqrt(10.0 / 460.0), 0.99, mInliers);
		    
        }
        else{
            cv::solvePnPRansac(gMatchedPoints3D, gOldNormPoints2D, mK, mDistortionCV, mRotationVecCV, mTranslationCV, false, 100, 10.0 / 460.0, 0.99, mInliers);
        }
    }


    //The pose has also be updated.
    cv::Rodrigues(mRotationVecCV, mRotationCV);

    gStatus.reserve(gOldNormPoints2D.size());
    for (int i = 0; i < (int)gOldNormPoints2D.size(); i++){
        gStatus.push_back(0);
    }

    for( int i = 0; i < mInliers.rows; i++)
    {
        int n = mInliers.at<int>(i);
        gStatus[n] = 1;
    }

    // 4.655261,-1.613830,0.669651
    // 4.594507,-1.643386,0.653808


    Eigen::Matrix3d mRotationPnP_cw, mRotationPnP_wc;
    cv::cv2eigen(mRotationCV, mRotationPnP_cw);


    mRotationPnP_wc = mRotationPnP_cw.transpose();

    Eigen::Vector3d mTranslationPnP_cw, mTranslationPnP_wc;
    cv::cv2eigen(mTranslationCV, mTranslationPnP_cw);
    mTranslationPnP_wc = mRotationPnP_wc * (-mTranslationPnP_cw);

    //We have solved the inverse pose of the old keyframe.
    mOldRotation_wi = mRotationPnP_wc * mRotation_ic.transpose();
    mOldTranslation_wi = mTranslationPnP_wc - mOldRotation_wi * mTranslation_ic;
}


//Find the loop info, T i_old i_current
bool ServerKeyFrame::FindConnection(ServerKeyFrame* pOldKeyFrame, bool bAlign)
{
	int nMinLoop = BIG_LOOP_NUM;
	if (bAlign){
		nMinLoop = MIN_LOOP_NUM;
	}
	
	vector<cv::Point2f> gCurrentPoints2D, gOldPoints2D;
	vector<cv::Point2f> gCurrentNorm2D, gOldNorm2D;
	vector<cv::Point3f> gMatchedPoints3D;
	vector<int> gMatchedID;
	vector<uchar> gStatus;

	gMatchedPoints3D = this->m_gPoints3D;
	gCurrentPoints2D = this->m_gPoints2D;
	gCurrentNorm2D = this->m_gNormalizedWindowPoints;
	gMatchedID = this->m_gPointsID;


	//Find matches.
	this->SearchByBRIEFDes( pOldKeyFrame->m_gBriefDescriptors, 
							pOldKeyFrame->m_gProjectedPoints, 
							pOldKeyFrame->m_gNormalizedProjectedPoints, 
							gStatus,
							gOldPoints2D, gOldNorm2D);
	
	//Remove unmatched points.
	reduceVector(gCurrentPoints2D, gStatus);
	reduceVector(gOldPoints2D, gStatus);
	reduceVector(gCurrentNorm2D, gStatus);
	reduceVector(gOldNorm2D, gStatus);
	reduceVector(gMatchedPoints3D, gStatus);
	reduceVector(gMatchedID, gStatus);
	gStatus.clear();


	this->FundmantalMatrixRANSAC(gCurrentNorm2D, gOldNorm2D, gStatus);
	//Remove unmatched points.
	reduceVector(gCurrentPoints2D, gStatus);
	reduceVector(gOldPoints2D, gStatus);
	reduceVector(gCurrentNorm2D, gStatus);
	reduceVector(gOldNorm2D, gStatus);
	reduceVector(gMatchedPoints3D, gStatus);
	reduceVector(gMatchedID, gStatus);
	gStatus.clear();


	Eigen::Vector3d mOldTranslation_wi;
	Eigen::Matrix3d mOldRotation_wi;
	Eigen::Vector3d mRelativeTranslation;
	Eigen::Quaterniond mRelativeQuanternion;


	double nRelativeYaw;

	//Debug
	Eigen::Matrix3d mCurrentRotationDebug;
	Eigen::Vector3d mCurrentTranslationDebug;



	//Firstly use PnP to remove outliers.
	if ((int)gCurrentPoints2D.size() > nMinLoop)
	{
		gStatus.clear();
	    PnPRANSAC(gOldNorm2D, gMatchedPoints3D, gStatus, mOldTranslation_wi, mOldRotation_wi);
	 	
	 	// PnPRANSAC(gCurrentNorm2D, gMatchedPoints3D, gStatus, mOldTranslation_wi, mOldRotation_wi);

	    reduceVector(gCurrentPoints2D, gStatus);
		reduceVector(gOldPoints2D, gStatus);
		reduceVector(gCurrentNorm2D, gStatus);
		reduceVector(gOldNorm2D, gStatus);
		reduceVector(gMatchedPoints3D, gStatus);
		reduceVector(gMatchedID, gStatus);
		
		//Remove the plotting module.
	}

	//Plot the matching result.
   //Now we can draw the matching correspondence.
    vector<cv::KeyPoint> gCurrentKeyPoints, gOldKeyPoints;
    vector<cv::DMatch> gMatches;
    gCurrentKeyPoints.reserve(gCurrentPoints2D.size());
    gOldKeyPoints.reserve(gCurrentPoints2D.size());
    gMatches.reserve(gCurrentPoints2D.size());

    for (int ii=0;ii<gCurrentPoints2D.size();ii++){
    	cv::KeyPoint iCurrentKeyPoint, iOldKeyPoint;
    	iCurrentKeyPoint.pt = gCurrentPoints2D[ii];
    	iOldKeyPoint.pt = gOldPoints2D[ii];
    	gCurrentKeyPoints.push_back(iCurrentKeyPoint);
    	gOldKeyPoints.push_back(iOldKeyPoint);
    	cv::DMatch iMatch;
    	iMatch.trainIdx = ii;
    	iMatch.queryIdx = ii;
    	gMatches.push_back(iMatch);
    }
    // cv::Mat mMatchedImage;

    // cv::drawMatches(this->m_mImage, gCurrentKeyPoints, pOldKeyFrame->m_mImage, gOldKeyPoints, gMatches, mMatchedImage);
    // this->m_mMatchedImage = mMatchedImage.clone();
    // cv::imwrite("/home/kyrie/Documents/DataSet/CoVins/loop_closure.jpg", mMatchedImage);
    // // cv::waitKey(5);

    
	//Remove outliers with the epipolar constraints.
	cout << "Matched size: " << gCurrentPoints2D.size() << endl;
	if ((int)gCurrentPoints2D.size() >= nMinLoop)
	{
	    
	    mRelativeTranslation = mOldRotation_wi.transpose() * (this->m_mGlobalTranslation_wi - mOldTranslation_wi);
	    mRelativeQuanternion = mOldRotation_wi.transpose() * this->m_mGlobalRotation_wi;
	    // mRelativeQuanternion.toRotationMatrix();
	    nRelativeYaw = (ServerUtility::R2ypr(this->m_mGlobalRotation_wi).x() - ServerUtility::R2ypr(mOldRotation_wi).x());
	    // for (int i=0;i<100;i++){
	    // 	cout << endl;
	    // 	cout << fixed << setprecision(6);
	    // 	cout << "Old Timestamp: " << pOldKeyFrame->m_nTimeStamp << endl;
	    // 	cout << "Current Timestamp: " << this->m_nTimeStamp << endl;
	    // 	cout << "RelativeYaw is: " << nRelativeYaw << endl;
	    // 	cout << "RelativeTranslation is: " << mRelativeTranslation << endl;
	    // 	cout << "Local Rotation" << endl << this->m_mLocalRotation_wi << endl;
	    // 	cout << "Local Translation" << endl << this->m_mLocalT_Backup_wi << endl;
	    // 	cout << "Rotation ic is: " << endl << m_mRotation_ic << endl;
	    // 	cout << "Translation ic is: " << endl << m_mTranslation_ic << endl;
	    // 	cout << "points are: " << endl;
	    // 	for (int i=0;i<gMatchedPoints3D.size();i++){
	    // 		cout << "Point3d: " << gMatchedPoints3D[i] << endl;
	    // 		// cout << "Norm2d: " << gOldNorm2D[i] << endl; 
	    // 	}

	    // 	for (int i=0;i<gMatchedPoints3D.size();i++){
	    // 		cout << "Point2d: " << gCurrentPoints2D[i] << endl;
	    // 		// cout << "Norm2d: " << gOldNorm2D[i] << endl; 
	    // 	}


	    // 	for (int i=0;i<gMatchedPoints3D.size();i++){
	    // 		cout << "OldPoint2d" << gOldPoints2D[i] << endl;
	    // 		// cout << "Norm2d: " << gOldNorm2D[i] << endl; 
	    // 	}


	    // 	for (int i=0;i<gMatchedPoints3D.size();i++){
	    // 		cout << "OldNorm" << gOldNorm2D[i] << endl;
	    // 		// cout << "Norm2d: " << gOldNorm2D[i] << endl; 
	    // 	}


	    // 	for (int i=0;i<gMatchedPoints3D.size();i++){
	    // 		cout << "CurrentNorm" << gCurrentNorm2D[i] << endl;
	    // 		// cout << "Norm2d: " << gOldNorm2D[i] << endl; 
	    // 	}

	    // 	cout << endl;
	    // }
	 
	    if (abs(nRelativeYaw) < 10.0 && mRelativeTranslation.norm() < 20.0)
	    {

	    	this->m_bHasLoop = true;
	    	this->m_nLoopIndex = pOldKeyFrame->m_nGlobalIndex;
	    	this->m_mLoopInfo << 	mRelativeTranslation.x(), mRelativeTranslation.y(), mRelativeTranslation.z(),
	    	             			mRelativeQuanternion.w(), mRelativeQuanternion.x(), mRelativeQuanternion.y(), mRelativeQuanternion.z(),
	    	             			nRelativeYaw;


	    	this->m_gLoopIndices.push_back(this->m_nLoopIndex);
	    	this->m_gLoopInfos.push_back(this->m_mLoopInfo);
	    	cout << "Finish loop closure" << endl;
	    	//Fast relocalization is not required!!!
	        return true;
	    }
	}
	return false;
}

//Get Local Pose.
void ServerKeyFrame::GetVIOPose(Eigen::Vector3d & mTranslation_wi , Eigen::Matrix3d & mRotation_wi)
{

	this->m_mPoseMutex.lock();
    mTranslation_wi = this->m_mLocalTranslation_wi;
    mRotation_wi = this->m_mLocalRotation_wi;

	this->m_mPoseMutex.unlock();
}

//Get Global Pose.
void ServerKeyFrame::GetPose(Eigen::Vector3d & mTranslation_wi, Eigen::Matrix3d & mRotation_wi)
{

	this->m_mPoseMutex.lock();
    mTranslation_wi = this->m_mGlobalTranslation_wi;
    mRotation_wi = this->m_mGlobalRotation_wi;

	this->m_mPoseMutex.unlock();
}

void ServerKeyFrame::GetCameraPose(Eigen::Vector3d & mTranslation_wc, Eigen::Matrix3d & mRotation_wc){

	this->m_mPoseMutex.lock();
	Sophus::SE3 mPose_wi = Sophus::SE3(
		this->m_mGlobalRotation_wi,
		this->m_mGlobalTranslation_wi); 
	
	Sophus::SE3 mPose_ic(this->m_mRotation_ic, this->m_mTranslation_ic);
	Sophus::SE3 mPose_wc = mPose_wi * mPose_ic;
	mTranslation_wc = mPose_wc.translation();
	mRotation_wc = mPose_wc.rotation_matrix();
	this->m_mPoseMutex.unlock();
}

void ServerKeyFrame::GetInitialCameraPose(Eigen::Vector3d & mTranslation_wc, Eigen::Matrix3d & mRotation_wc){

	this->m_mPoseMutex.lock();
	Sophus::SE3 mPose_wi = Sophus::SE3(
		this->m_mLocalR_Backup_wi,
		this->m_mLocalT_Backup_wi); 
	
	Sophus::SE3 mPose_ic(this->m_mRotation_ic, this->m_mTranslation_ic);
	Sophus::SE3 mPose_wc = mPose_wi * mPose_ic;
	mTranslation_wc = mPose_wc.translation();
	mRotation_wc = mPose_wc.rotation_matrix();
	this->m_mPoseMutex.unlock();
}


void ServerKeyFrame::UpdatePose(const Eigen::Vector3d & mTranslation_wi, const Eigen::Matrix3d & mRotation_wi)
{
	this->m_mPoseMutex.lock();
    this->m_mGlobalTranslation_wi = mTranslation_wi;
    this->m_mGlobalRotation_wi = mRotation_wi;

    this->m_mGlobalPose_iw = Sophus::SE3(mRotation_wi, mTranslation_wi).inverse();
	this->m_mPoseMutex.unlock();

}

//The global pose should also be modified.
void ServerKeyFrame::UpdateVIOPose(const Eigen::Vector3d & mTranslation_wi, const Eigen::Matrix3d & mRotation_wi)
{

	this->m_mPoseMutex.lock();
	//Compute the shift.
	Sophus::SE3 mPose_ni(mRotation_wi, mTranslation_wi);
	Sophus::SE3 mPose_wi(this->m_mLocalRotation_wi, this->m_mLocalTranslation_wi);

	Sophus::SE3 mPose_nw = mPose_ni * mPose_wi.inverse();

    for (int i=0;i<this->m_gPoints3D.size();i++){
        cv::Point3f iPoint3D = this->m_gPoints3D[i];
        Eigen::Vector3d mPoint3D(iPoint3D.x, iPoint3D.y, iPoint3D.z);
        mPoint3D = mPose_nw * mPoint3D;
        iPoint3D.x = mPoint3D(0);
        iPoint3D.y = mPoint3D(1);
        iPoint3D.z = mPoint3D(2);
        this->m_gPoints3D[i] = iPoint3D;
    }


	this->m_mLocalTranslation_wi = mTranslation_wi;
    this->m_mLocalRotation_wi = mRotation_wi;

    this->m_mLocalPose_iw = Sophus::SE3(mRotation_wi, mTranslation_wi).inverse();

    //The global pose should also be set.
    this->m_mGlobalTranslation_wi = this->m_mLocalTranslation_wi;
    this->m_mGlobalRotation_wi = this->m_mLocalRotation_wi;

    this->m_mGlobalPose_iw = this->m_mLocalPose_iw;

	this->m_mPoseMutex.unlock();
}


Eigen::Vector3d ServerKeyFrame::GetLoopRelativeT()
{
    return Eigen::Vector3d(m_mLoopInfo(0), m_mLoopInfo(1), m_mLoopInfo(2));
}

//Old Local
Eigen::Quaterniond ServerKeyFrame::GetLoopRelativeQ()
{
    return Eigen::Quaterniond(m_mLoopInfo(3), m_mLoopInfo(4), m_mLoopInfo(5), m_mLoopInfo(6));
}



Eigen::Vector3d ServerKeyFrame::GetLoopRelativeT(int nIndex)
{
	Eigen::Matrix<double, 8, 1 > mLoopInfo = this->m_gLoopInfos[nIndex];
    return Eigen::Vector3d(mLoopInfo(0), mLoopInfo(1), mLoopInfo(2));
}

//Old Local
Eigen::Quaterniond ServerKeyFrame::GetLoopRelativeQ(int nIndex)
{
	Eigen::Matrix<double, 8, 1 > mLoopInfo = this->m_gLoopInfos[nIndex];
    return Eigen::Quaterniond(mLoopInfo(3), mLoopInfo(4), mLoopInfo(5), mLoopInfo(6));
}


double ServerKeyFrame::GetLoopRelativeYaw(int nIndex)
{
    Eigen::Matrix<double, 8, 1 > mLoopInfo = this->m_gLoopInfos[nIndex];
    return mLoopInfo(7);
}


double ServerKeyFrame::GetLoopRelativeYaw()
{
    return this->m_mLoopInfo(7);
}

void ServerKeyFrame::UpdateLoop(Eigen::Matrix<double, 8, 1 > & mLoopInfo)
{
	if (abs(mLoopInfo(7)) < 30.0 && Eigen::Vector3d(mLoopInfo(0), mLoopInfo(1), mLoopInfo(2)).norm() < 20.0)
	{
		//printf("update loop info\n");
		this->m_mLoopInfo = mLoopInfo;
	}
}






// //The info published to open chisel.
// void ServerKeyFrame::LoadRefInfo(	Sophus::SE3 & mRefPose_wc,
//                     				std_msgs::Header & iHeader,
// 			                        cv::Mat & mDepthMap,
// 			                        cv::Mat & mColorMap,
// 			                        cv::Mat & mK,
// 			                        int & nWidth, 
// 			                        int & nHeight){
// 	Eigen::Matrix3d mRefRotation_wc;
// 	Eigen::Vector3d mRefTranslation_wc;
// 	this->GetCameraPose(mRefTranslation_wc, mRefRotation_wc);
// 	mRefPose_wc = Sophus::SE3(mRefRotation_wc, mRefTranslation_wc);
// 	iHeader = this->m_iHeader;
// 	mDepthMap = 1.0/this->m_mInvDepthMap;
// 	mColorMap = this->m_mUndistortedImage;
// 	Eigen::Matrix3d mK_eigen = this->m_pDepthCamera->GetK();
// 	cv::eigen2cv(mK_eigen, mK);
// 	nWidth = mColorMap.cols;
// 	nHeight = mColorMap.rows;
// }

bool ServerKeyFrame::FreeSpace(){
	this->m_mDepthMutex.lock();

	this->m_mPoseMutex.lock();
	if (!this->m_bFreeSpace){
		
			this->m_mInvDepthMap.release();
			this->m_mRecoveredImage.release();
			// // this->m_mImage.release();
			// this->m_mColoredImage.release();

			// this->m_mUndistortedImage.release();
			
			vector<cv::Point3d>().swap(this->m_gMapPoints);
			vector<cv::Point3d>().swap(this->m_gMapPointsColor);


			this->m_bFreeSpace = true;
		
	}

	this->m_mDepthMutex.unlock();

	this->m_mPoseMutex.unlock();
	return true;
	
}