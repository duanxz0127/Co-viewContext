#include <cstdlib>
#include <ctime>
#include <iostream>
#include <time.h>
#include <vector>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>

#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/impl/pcl_base.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/registration/gicp.h>
#include <pcl/recognition/ransac_based/auxiliary.h>
#include <pcl/recognition/ransac_based/trimmed_icp.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <Eigen/Dense>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "CSF/CSF.h"
#include "CSF/point_cloud.h"
#include "ScanContext/Scancontext.h"

#include <omp.h>

#pragma once
class CoviewContext
{
	using PointT = pcl::PointXYZ;

public:
	// reference scan grid size
	float gridSize;
	// radius search scope
	float searchRadius;
	// dowmsample size
	float downSampleSize;
    // trimmed ICP threshold
	double icp_thr;

	// reference scan path
	std::string referncePCDPath;
	// local scan path
	std::string localPCDPath;
    // local trajectory path
    std::string localTrajectoryPath;
	// save path
	std::string saveBasePath;

private:
	// reference scan point cloud
	pcl::PointCloud<PointT>::Ptr referenceCloud;
	// downsampled reference scan point cloud
	pcl::PointCloud<PointT>::Ptr referenceDSCloud;
	// ground points of reference scan
	pcl::PointCloud<PointT>::Ptr referenceGroundCloud;
	// virtual stations of reference scan
	pcl::PointCloud<PointT>::Ptr referenceVirtualCloud;
	
    // local scan point cloud
    pcl::PointCloud<PointT>::Ptr localCloud;
    // downsampled local scan point cloud
    pcl::PointCloud<PointT>::Ptr localDSCloud;

	// vector of reference scan point cloud
	std::vector<pcl::PointCloud<PointT>> v_ref_pc;
	// vector of local scan point cloud
	std::vector<pcl::PointCloud<PointT>> v_loc_pc;

    // trajectory of local scan
	std::vector<Eigen::Vector3d> v_traj;
	// reference Co-view Context descriptor vector
	std::vector <MatrixXd> v_ref_coc;	    
	// local Co-view Context descriptor vector
    std::vector <MatrixXd> v_loc_coc;
    // pose of local scans
	std::vector<Eigen::Matrix4f> v_loc_pose;
    
	SCManager sc;
    
	double icp_score;

public:

	CoviewContext() 
	{
		gridSize = 10.0;
		searchRadius = 25.0;
		downSampleSize = 0.75;
        icp_thr = 0.6;
		sc.PC_MAX_RADIUS = searchRadius;

		referenceCloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
		referenceDSCloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
		referenceGroundCloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
		referenceVirtualCloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
		localCloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
		localDSCloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

	}
    
	~CoviewContext()
	{

	}
    
	void processReferenceCloud();

	void processLocalCloud();

private:

	bool create_directories_if_not_exist(const std::string& path);

    void readPCDFile(const std::string path, pcl::PointCloud<PointT>::Ptr cloud);

	void readTrajectoryFile(const std::string path);

	void voxelFilter(pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointCloud<PointT>::Ptr cloud_out, double size);

    void outlierRemoval(pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointCloud<PointT>::Ptr cloud_out);

	void segGroundPoints(pcl::PointCloud<PointT>::Ptr cloud_in);

	void groupReferenceCloud(float radius);

    void groupLocalCloud(float dis_thr, float radius);

    void makeCoViewContextDescriptors(pcl::PointCloud<PointT>::Ptr cloud_in, MatrixXd &v_coc);

    void rotateYawByDescriptor(pcl::PointCloud<PointT>& cloud_in, float rotAngle);

	Eigen::Matrix4f doTrimmedICP(pcl::PointCloud<PointT>::Ptr cloud_src, pcl::PointCloud<PointT>::Ptr cloud_tar, pcl::PointCloud<PointT>::Ptr cloud_trans, double ratio);

    Eigen::Matrix4f doGeneralizedICP(pcl::PointCloud<PointT>::Ptr cloud_src, pcl::PointCloud<PointT>::Ptr cloud_tar, pcl::PointCloud<PointT>::Ptr cloud_trans);

	

};