#ifndef CHISEL_WRAPPER_H_
#define CHISEL_WRAPPER_H_

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include "./Chisel.h"
#include "./truncation/QuadraticTruncator.h"
#include "./truncation/InverseTruncator.h"
#include "./weighting/ConstantWeighter.h"
#include "./Chunk.h"
#include "../chisel_wrapper/chisel_map_info.h"

#include <iostream>

#include <string>
#include <vector>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>



#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/core/eigen.hpp>

#include <chisel_msg/ChiselMap.h>


using namespace std;



typedef float DepthData;
typedef uint8_t ColorData;

bool CVToDepthMap(
		cv::Mat & mDepthMap,
		chisel::DepthImage<DepthData> * pDepthImage);



bool CVToColorMap(
		cv::Mat & mColorMap,
		chisel::ColorImage<ColorData> * pColorImage);


chisel::PinholeCamera GenerateCamera(
	float nFocalX, float nFocalY, 
	float nPrincipalX, float nPrincipalY, 
	int nWidth, int nHeight);


	
chisel::Transform GenerateTransform(Eigen::Affine3d mTransform);

namespace chisel{

float Norms(Vec3 mVec);


bool InverseMergeChunks(
	chisel::ChiselPtr pChiselMapRef,
	chisel::ChiselPtr pChiselMapMerge,
	Eigen::Affine3f mPose_rm);

Vec3 Trunc(Vec3 & mVec);

Eigen::Vector3i Trunc(Eigen::Vector3i & mVec, Eigen::Vector3i & mMin, Eigen::Vector3i & mMax);


void CheckTSDF(
		chisel::ChiselPtr pChiselMapRef,
		Eigen::Matrix3f mDepthK,
		Eigen::Affine3f mPose_rc,
		int nDepthWidth, 
		int nDepthHeight,
		cv::Mat & mDepthMap);

float SearchForChunkLength(
	Vec3 mPositionRef,
	Vec3 mDirection,
	Vec3 mStart, Vec3 mEnd, Vec3 mOrigin);


void RecoverDepthAndColorMapFast(
        chisel::ChiselPtr pChiselMapRef,
        Eigen::Matrix3f mDepthK,
        Eigen::Affine3f mPose_rc,
        int nDepthWidth, 
        int nDepthHeight,
        cv::Mat & mDepthMap,
        cv::Mat & mColorMap,
        chisel::TruncatorPtr pTruncator);


void RecoverDepthMapFast(
		chisel::ChiselPtr pChiselMapRef,
		Eigen::Matrix3f mDepthK,
		Eigen::Affine3f mPose_rc,
		int nDepthWidth, 
		int nDepthHeight,
		cv::Mat & mDepthMap,
		chisel::TruncatorPtr pTruncator);


void RecoverDepthMap(
		chisel::ChiselPtr pChiselMapRef,
		Eigen::Matrix3f mDepthK,
		Eigen::Affine3f mPose_rc,
		int nDepthWidth, 
		int nDepthHeight,
		cv::Mat & mDepthMap);


bool MergeChunks(
	chisel::ChiselPtr pChiselMapRef,
	chisel::ChiselPtr pChiselMapMerge,
	Eigen::Affine3f mPose_rm);
}


//Wrapper class.
class ChiselWrapper
{
public:
	ChiselWrapper(	Eigen::Matrix3d mK, Eigen::Matrix3d mDepthK, int nWidth, int nHeight,
					Eigen::Affine3d mRefPose_wc,
					bool bColor = true, float nResolution = 0.03, int nChunkSize = 8,
					float nTruncationDistScale = 1.0, bool bUseCarving = true, float nCarvingDist = 0.0,
					float nWeight = 1.0, float nNearDist = 0.3, float nFarDist = 3.0,
					float nMinWeight = 0.0);

	cv::Size GetSize();

	//FC32  
	bool Integrate(cv::Mat & mDepthImage, cv::Mat mColorImage, Eigen::Affine3d & mPose_wc, int nPublishIndex);


	chisel::ProjectionIntegrator * SetupIntegrator(
			chisel::ChiselPtr pChiselMap, 
			chisel::TruncatorPtr pTruncator, 
			uint16_t nWeight, bool bUseCarving, float nCarvingDist);
	

	void SaveMesh(string aFilename);

	void SaveRefMesh(string aFilename);

	void GenerateNewSubMap(	Eigen::Affine3d mRefPose_wc,
							bool bColor = true, float nResolution = 0.03, int nChunkSize = 8,
							float nTruncationDistScale = 1.0, bool bUseCarving = true, float nCarvingDist = 0.0,
							float nWeight = 1.0, float nNearDist = 0.3, float nFarDist = 3.0,
							float nMinWeight = 0.0);

	void MergeChunks();

	ChiselMapInfo * CompressedToInfo(){
		ChiselMapInfo * pInfo = new ChiselMapInfo(this->m_pCurrentChiselMap.get());
		return pInfo;
	}

	void ConvertSubMapInfo(
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

			for (chisel_msg::DistVoxel & iVoxel : iChunk.voxels){
				DistVoxelInfo * pInfo = new DistVoxelInfo();
				pInfo->m_nSDF = iVoxel.sdf;
				pInfo->m_nWeight = iVoxel.weight;
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


	void PublishSubMapInfo(int nAgentNum, chisel_msg::ChiselMap & iMapMsg){
		ChiselMapInfo * pInfo = new ChiselMapInfo(this->m_pCurrentChiselMap.get());
		ChunkManagerInfo * pManagerInfo = pInfo->m_pChunkManagerInfo;

		iMapMsg.header.stamp = ros::Time::now();

		for (int i=0;i< this->m_gPublishIndices.size();i++){
			iMapMsg.publishIndices.push_back(this->m_gPublishIndices[i]);
		}

		Eigen::Vector3d mTranslation_wc =  m_mRefPose_wc.translation();
		Eigen::Matrix3d mRotation_wc = m_mRefPose_wc.rotation();

		iMapMsg.refPosition_wc.x = mTranslation_wc(0);
		iMapMsg.refPosition_wc.y = mTranslation_wc(1);
		iMapMsg.refPosition_wc.z = mTranslation_wc(2);

		Eigen::Quaterniond mQuaternion_wc(mRotation_wc);
		iMapMsg.refOrientation_wc.w = mQuaternion_wc.w();
		iMapMsg.refOrientation_wc.x = mQuaternion_wc.x();
		iMapMsg.refOrientation_wc.y = mQuaternion_wc.y();
		iMapMsg.refOrientation_wc.z = mQuaternion_wc.z();

		iMapMsg.agentNum = nAgentNum;

		//TODO: Header
		iMapMsg.chunkSize.x = pManagerInfo->m_mChunkSize(0);
		iMapMsg.chunkSize.y = pManagerInfo->m_mChunkSize(1);
		iMapMsg.chunkSize.z = pManagerInfo->m_mChunkSize(2);
		iMapMsg.voxelResolution = pManagerInfo->m_nVoxelResolutionMeters;
		iMapMsg.useColor = pManagerInfo->m_bUseColor;
		iMapMsg.lengthChunks = pManagerInfo->m_gChunkValues.size();
		for (ChunkInfo * pChunkInfo : pManagerInfo->m_gChunkValues){
			chisel_msg::Chunk iChunk;
			iChunk.chunkID.x = pChunkInfo->m_mChunkID(0);
			iChunk.chunkID.y = pChunkInfo->m_mChunkID(1);
			iChunk.chunkID.z = pChunkInfo->m_mChunkID(2);

			iChunk.numVoxels.x = pChunkInfo->m_mNumVoxels(0);
			iChunk.numVoxels.y = pChunkInfo->m_mNumVoxels(1);
			iChunk.numVoxels.z = pChunkInfo->m_mNumVoxels(2);
			iChunk.resolution = pChunkInfo->m_nResolution;
			iChunk.origin.x = pChunkInfo->m_mOrigin(0);
			iChunk.origin.y = pChunkInfo->m_mOrigin(1);
			iChunk.origin.z = pChunkInfo->m_mOrigin(2);
			iChunk.lengthVoxels = pChunkInfo->m_gVoxels.size();
			iChunk.lengthColors = pChunkInfo->m_gColors.size();

			for (DistVoxelInfo & iVoxelInfo : pChunkInfo->m_gVoxels){
				chisel_msg::DistVoxel iVoxel;
				iVoxel.sdf = iVoxelInfo.m_nSDF;
				iVoxel.weight = iVoxelInfo.m_nWeight;
				iChunk.voxels.push_back(iVoxel);
			}


			for (ColorVoxelInfo & iColorInfo : pChunkInfo->m_gColors){
				chisel_msg::ColorVoxel iColor;
				iColor.red = iColorInfo.m_nRed;
				iColor.green = iColorInfo.m_nGreen;
				iColor.blue = iColorInfo.m_nBlue;
				iColor.weight = iColorInfo.m_nWeight;
				iChunk.colors.push_back(iColor);
			}
			iMapMsg.chunkValues.push_back(iChunk);

		}



	}


	

private:
	bool m_bNewSubMap;
	int m_nWidth, m_nHeight;
	//Intrinsics
	Eigen::Matrix3d m_mK, m_mDepthK;
	Eigen::Affine3d m_mRefPose_wc;


	chisel::ChiselPtr m_pCurrentChiselMap;
	vector<int> m_gPublishIndices;

	vector<chisel::ChiselPtr> m_gChisels;
	vector<Eigen::Affine3d> m_gPoses;
	chisel::TruncatorPtr m_pTruncator;
	chisel::ProjectionIntegrator * m_pProjectionIntegrator;
	std::shared_ptr<chisel::DepthImage<DepthData>> m_pDepthImage;
	std::shared_ptr<chisel::ColorImage<ColorData>> m_pColorImage;


	bool m_bIntegrateColor;
	float m_nWeight;
	float m_nMinWeight;
	//Resolution of the tsdf.
	float m_nResolution; 
	int m_nChunkSize;
	float m_nTruncationDistScale; 
	bool m_bUseCarving;
	float m_nCarvingDist;
	float m_nNearDist; 
	float m_nFarDist;

	//Camera models.
	chisel::PinholeCamera m_iColorCamera, m_iDepthCamera;



	
};

#endif