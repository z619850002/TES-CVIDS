#ifndef SERVER_KEYFRAME_H_
#define SERVER_KEYFRAME_H_

#include <vector>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "parameters.h"
#include "server_brief_extractor.h"
#include "server_camera.h"
#include "./utility/server_utility.h"
//Sophus

#include "sophus/so3.h"
#include "sophus/se3.h"

#include <mutex>

#include <std_msgs/Header.h>

#define MIN_LOOP_NUM 20


#define BIG_LOOP_NUM 20

using namespace std;



class ServerKeyFrame
{
public:


	//TODO: Construct without image.
	ServerKeyFrame(	double nTimeStamp, int nClient, int nLocalIndex, 
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
			 		ServerCamera * pCamera);


	

	//Find matches.
	bool SearchInArea(		const DVision::BRIEF::bitset iWindowDescriptor,
                            const vector<DVision::BRIEF::bitset> & gOldDescriptors,
                            const vector<cv::Point2f> & gOldPoints,
                            const vector<cv::Point2f> & gNormOldPoints,
                            cv::Point2f & iBestMatchedPoint,
                            cv::Point2f & iBestMatchedNormPoint);

	//Compute the hamming distance between 2 descriptors.
	int HammingDistance(const DVision::BRIEF::bitset & iDescriptorA, const DVision::BRIEF::bitset & iDescriptorB);

	//Find matches with the gKeyPointsOld with the brief descriptors.
	//Called in findConnection.
	void SearchByBRIEFDes(		const vector<DVision::BRIEF::bitset> & gOldDescriptors,
                                const vector<cv::Point2f> & gOldPoints,
                                const vector<cv::Point2f> & gOldNormPoints,
                                vector<uchar> & gStatus,
								vector<cv::Point2f> & gMatchedPoints2D,
								vector<cv::Point2f> & gNormMatchedPoints2D);


	//Remove outliers by epipolar constraints.
	void FundmantalMatrixRANSAC(	const vector<cv::Point2f> & gCurrentNormPoints,
                                    const vector<cv::Point2f> & gOldNormPoints,
                                    vector<uchar> & gStatus);

	//Remove outliers by PnP
	void PnPRANSAC(		const vector<cv::Point2f> & gOldNormPoints2D,
                        const vector<cv::Point3f> & gMatchedPoints3D,
                        vector<uchar> & gStatus,
                        Eigen::Vector3d & mOldTranslation_wi, 
                        Eigen::Matrix3d & mOldRotation_wi);

	bool FindConnection(ServerKeyFrame* pOldKeyFrame, bool bAlign = false);

	void GetVIOPose(Eigen::Vector3d & mTranslation_wi , Eigen::Matrix3d & mRotation_wi);
	void GetPose(Eigen::Vector3d & mTranslation_wi, Eigen::Matrix3d & mRotation_wi);
	void GetCameraPose(Eigen::Vector3d & mTranslation_wc, Eigen::Matrix3d & mRotation_wc);
	void GetInitialCameraPose(Eigen::Vector3d & mTranslation_wc, Eigen::Matrix3d & mRotation_wc);


	void UpdatePose(const Eigen::Vector3d & mTranslation_wi, const Eigen::Matrix3d & mRotation_wi);
	void UpdateVIOPose(const Eigen::Vector3d & mTranslation_wi, const Eigen::Matrix3d & mRotation_wi);

	Eigen::Vector3d GetLoopRelativeT();
	Eigen::Quaterniond GetLoopRelativeQ();


	Eigen::Vector3d GetLoopRelativeT(int nIndex);
	Eigen::Quaterniond GetLoopRelativeQ(int nIndex);

	double GetLoopRelativeYaw();
	double GetLoopRelativeYaw(int nIndex);

	void UpdateLoop(Eigen::Matrix<double, 8, 1 > & mLoopInfo);


	void LoadDepthMap(cv::Mat & mDepthMap){
		this->m_mDepthMutex.lock();
		mDepthMap = 1/this->m_mInvDepthMap;
		this->m_mDepthMutex.unlock();
	}



	void DeleteDepthMaps(){

		this->m_mDepthMutex.lock();
		this->m_mInvDepthMap.release();
		this->m_mRecoveredImage.release();

		this->m_mDepthMutex.unlock();
	}


	// //The info published to open chisel.
	// void LoadRefInfo(		Sophus::SE3 & mRefPose_wc,
 //                            std_msgs::Header & iHeader,
 //                            cv::Mat & mDepthMap,
 //                            cv::Mat & mColorMap,
 //                            cv::Mat & mK,
 //                            int & nWidth, 
 //                            int & nHeight);

	bool FreeSpace();



public:
	bool m_bFreeSpace;
	//Is this keyframe from a vision slam system.
	bool m_bFromVisionSLAM;

	//Identifier of this keyframe.
	double m_nTimeStamp;                            //double time_stamp; 
	int m_nLocalIndex;								//int local_index;
	int m_nClientID;
	int m_nGlobalIndex;

	//Extrinsics
	Eigen::Matrix3d m_mRotation_ic;
	Eigen::Vector3d m_mTranslation_ic;


	
	//Pose offered by the VIO of the client.
	//Translation and rotation here is as the inverse of the pose.
	Sophus::SE3 m_mLocalPose_iw;
	//Translation.
	Eigen::Vector3d m_mLocalTranslation_wi;  			//Eigen::Vector3d vio_T_w_i; 
	//Rotation.
	Eigen::Matrix3d m_mLocalRotation_wi;				//Eigen::Matrix3d vio_R_w_i; 
	
	//Global Pose.
	//Translation and rotation here is as the inverse of the pose.
	Sophus::SE3 m_mGlobalPose_iw;
	Eigen::Vector3d m_mGlobalTranslation_wi;			//Eigen::Vector3d T_w_i;
	Eigen::Matrix3d m_mGlobalRotation_wi;				//Eigen::Matrix3d R_w_i;
	
	//Used for backup?
	//Translation and rotation here is as the inverse of the pose.
	Eigen::Vector3d m_mLocalT_Backup_wi;				//Eigen::Vector3d origin_vio_T;		
	Eigen::Matrix3d m_mLocalR_Backup_wi;				//Eigen::Matrix3d origin_vio_R;

	//Image of this frame.
	// cv::Mat m_mImage;								//cv::Mat image;
	// cv::Mat m_mUndistortedImage;
	// cv::Mat m_mColoredImage;
	cv::Mat m_mRecoveredImage;
 	//Compressed image.
	// cv::Mat m_mThumbnailImage;						//cv::Mat thumbnail;

	//Points from the vio
	//These are all mappoints offered by VIO.
	vector<cv::Point3f> m_gPoints3D;				//vector<cv::Point3f> point_3d; 
	vector<cv::Point2f> m_gPoints2D;				//vector<cv::Point2f> point_2d_uv; vector<cv::Point2f> point_2d_norm;
	vector<cv::Point2f> m_gNormalizedWindowPoints;
	//The ID of the mappoints.
	vector<int> m_gPointsID;						//vector<double> point_id;
	vector<DVision::BRIEF::bitset> m_gWindowBriefDescriptors;	// vector<BRIEF::bitset> window_brief_descriptors;



	//These are all points extracted additionally.
	//All Points.
	//Used to be matched by others.
	vector<cv::Point2f> m_gProjectedPoints;				// vector<cv::KeyPoint> keypoints;
	vector<cv::Point2f> m_gNormalizedProjectedPoints;
	//Points from the vio.
	vector<DVision::BRIEF::bitset> m_gBriefDescriptors;			// vector<BRIEF::bitset> brief_descriptors;
	

	// vector<cv::KeyPoint> keypoints_norm;
	
	// bool has_fast_point;
	// int sequence;
	bool m_bHasLoop;								//bool has_loop;
	bool m_bHasBeenAligned;


	int m_nLoopIndex;								//int loop_index;
	// cv::Mat m_mMatchedImage;	
	Eigen::Matrix<double, 8 , 1> m_mLoopInfo;		// Eigen::Matrix<double, 8, 1 > loop_info;


	vector<int> m_gLoopIndices;
	vector<Eigen::Matrix<double, 8 , 1>> m_gLoopInfos;


	ServerCamera * m_pServerCamera;
	ServerCamera * m_pDepthCamera;

	ServerKeyFrame * m_pPreviousKeyFrame;
	ServerKeyFrame * m_pNextKeyFrame;


	//Used in dense mapping

	cv::Mat m_mRecoveredDepthMap;
	cv::Mat m_mInvDepthMap;
	cv::Mat m_mInvCovMap;
	cv::Mat m_mConvergeMap;
	bool m_bFinalizeDepthMap;

	int m_nWidth, m_nHeight;

	//The map points are of the camera coordinates.
	vector<cv::Point3d> m_gMapPoints;
	//The color of these map points.
	//BGR
	vector<cv::Point3d> m_gMapPointsColor;


	std_msgs::Header m_iHeader;
	
	std::mutex m_mDepthMutex;
	std::mutex m_mPoseMutex;
	int m_nPublishIndex;				
};

#endif

