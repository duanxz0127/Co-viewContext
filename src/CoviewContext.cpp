#include "CoviewContext.h"

using namespace std;

void CoviewContext::processReferenceCloud()
{
    readPCDFile(referncePCDPath, referenceCloud);    
	voxelFilter(referenceCloud, referenceDSCloud, downSampleSize);
    outlierRemoval(referenceDSCloud, referenceDSCloud);
    segGroundPoints(referenceDSCloud);
	groupReferenceCloud(searchRadius);
}

void CoviewContext::processLocalCloud()
{
    readPCDFile(localPCDPath, localCloud);
    readTrajectoryFile(localTrajectoryPath);
    voxelFilter(localCloud, localDSCloud, downSampleSize);
    outlierRemoval(localDSCloud, localDSCloud);
    groupLocalCloud(gridSize, searchRadius);
}

bool CoviewContext::create_directories_if_not_exist(const std::string& path) {
   struct stat info;

    if (stat(path.c_str(), &info) != 0) {
        if (errno == ENOENT) {
            std::string command = "mkdir -p " + path;
            int result = system(command.c_str());
            if (result == 0) {
                std::cout << "Directories created: " << path << std::endl;
                return true;
            } else {
                std::cerr << "Failed to create directories: " << path << std::endl;
                return false;
            }
        } else {
            return false;
        }
    } else if (info.st_mode & S_IFDIR) {
        return true;
    } else {
        return false;
    }
}

void CoviewContext::readPCDFile(const string path, pcl::PointCloud<PointT>::Ptr cloud)
{
    if (pcl::io::loadPCDFile<PointT>(path, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file %s \n", path.c_str());
    }
}

void CoviewContext::readTrajectoryFile(const std::string path)
{
	v_traj.clear();

	ifstream file(path);
	if (!file.is_open()) {
		cerr << "Could not open file " << path << endl;
		return;
	}

	string line;
	while (getline(file, line)) {
		istringstream iss(line);
		string token;
		vector<double> values;

		while (getline(iss, token, ' ')) {
			values.push_back(stod(token));
		}

		if (values.size() != 8) {
			cerr << "Invalid line format: " << line << endl;
			continue;
		}

		double x = values[1];
		double y = values[2];
		double z = values[3];

		Eigen::Vector3d pos(x, y, z);
		v_traj.push_back(pos);
	}

	file.close();
}

void CoviewContext::voxelFilter(pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointCloud<PointT>::Ptr cloud_out, double size)
{
	std::cout << "->Downsampling pointcloud..." << std::endl;
	pcl::VoxelGrid<PointT> vox;
	vox.setInputCloud(cloud_in);
	vox.setLeafSize(size, size, size);
	vox.filter(*cloud_out);
}

void CoviewContext::outlierRemoval(pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointCloud<PointT>::Ptr cloud_out)
{
	std::cout << "->Removing outliers..." << std::endl;
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(cloud_in);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_out);
}

void CoviewContext::segGroundPoints(pcl::PointCloud<PointT>::Ptr cloud_in)
{
	std::cout << "->Splitting ground pointcloud..." << std::endl;

	pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr roof_cloud(new pcl::PointCloud<pcl::PointNormal>);

	CSF csfilter;
	csfilter.params.bSloopSmooth = true;
	csfilter.params.cloth_resolution = 1.0;
	csfilter.params.rigidness = 3;
	csfilter.params.time_step = 0.65;
	csfilter.params.class_threshold = 0.5;
	csfilter.params.interations = 500;

	std::vector<csf::Point> csfPoints;
	for (int i = 0; i < referenceDSCloud->points.size(); i++)
	{
		csf::Point a;
		a.x = referenceDSCloud->points[i].x;
		a.y = referenceDSCloud->points[i].y;
		a.z = referenceDSCloud->points[i].z;
		csfPoints.push_back(a);
	}
	csfilter.setPointCloud(csfPoints);

	std::vector<int> ground_index;
	std::vector<int> non_ground_index;
	csfilter.do_filtering(ground_index, non_ground_index);

	float dimian_z = 999.0;
	for (int i = 0; i < ground_index.size(); i++)
	{
		PointT a;
		a.x = csfPoints[ground_index[i]].x;
		a.y = csfPoints[ground_index[i]].y;
		a.z = csfPoints[ground_index[i]].z;
		referenceGroundCloud->push_back(a);
	}

}

void CoviewContext::groupReferenceCloud(float radius)
{
	// Grouping pointcloud
	std::cout << "->Grouping reference pointcloud..." << std::endl;
    
	voxelFilter(referenceGroundCloud, referenceVirtualCloud, gridSize);

	pcl::PointCloud<PointT>::Ptr candidate_cloud(new pcl::PointCloud<PointT>);
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	PointT stationCandidate;
	size_t groupSize = referenceVirtualCloud->size();
    
	cout << "VRS count: " << groupSize << endl;
	for (size_t i = 0; i < groupSize; ++i) {
		if (i % 50 == 0)
		{
			cout << "Processing No." << i << " Reference PointCloud..." << endl;
		}
		// VRS position
		stationCandidate.x = referenceVirtualCloud->points[i].x;
		stationCandidate.y = referenceVirtualCloud->points[i].y;
		stationCandidate.z = referenceVirtualCloud->points[i].z;
		// stationCandidate.intensity = referenceVirtualCloud->points[i].intensity;

		// k-d tree radius search
		pcl::KdTreeFLANN<PointT> refKdTree;
		refKdTree.setInputCloud(referenceDSCloud);
		refKdTree.radiusSearch(stationCandidate, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

		for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
			PointT tmpPoint = referenceDSCloud->points[pointIdxRadiusSearch[j]];
			tmpPoint.x = tmpPoint.x - stationCandidate.x;
			tmpPoint.y = tmpPoint.y - stationCandidate.y;
			tmpPoint.z = tmpPoint.z - stationCandidate.z;
			candidate_cloud->push_back(tmpPoint);
		}
        MatrixXd coc;
        makeCoViewContextDescriptors(candidate_cloud, coc);
		v_ref_coc.push_back(coc);
		// v_ref_pc.push_back(*candidate_cloud);

		// save candidate_cloud
		std::string save_dir = saveBasePath + "VRS/";
		create_directories_if_not_exist(save_dir);
		std::string save_path = save_dir + "VRS_" + std::to_string(i) + ".pcd";
		pcl::io::savePCDFileASCII(save_path, *candidate_cloud);

		candidate_cloud->clear();
		pointIdxRadiusSearch.clear();
		pointRadiusSquaredDistance.clear();

	}
}

void CoviewContext::groupLocalCloud(float dis_thr, float radius)
{    
	double score_max = -1.0;
	int score_max_index = 0;
	std::vector<std::tuple<double, int, int, int>> coc_scores;
	pcl::PointCloud<PointT>::Ptr localRadiusCloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr localTransformedCloud(new pcl::PointCloud<PointT>);

    std::vector<pcl::PointCloud<PointT>> v_icp_pc;
	std::vector<float> v_icp_scores;

	std::vector<Eigen::Vector3d> v_key_trajectory;
	v_key_trajectory.push_back(v_traj[0]);

    // select key pose frame
    for (int i = 1; i < v_traj.size(); i++)
	{
		Eigen::Vector3d p1 = v_key_trajectory.back();
		Eigen::Vector3d p2 = v_traj[i];

		float traj_distance = (p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]) + (p1[2] - p2[2]) * (p1[2] - p2[2]);
		// if (traj_distance > dis_thr * dis_thr)
		if (traj_distance > 4)
		{
			v_key_trajectory.push_back(v_traj[i]);		
		}
	}

    cout << "key pose frame size: " << v_key_trajectory.size() << endl;
	for (int i = 0; i < v_key_trajectory.size(); i++)
	{
        cout << "No. " << i << " key pose frame..." << endl;
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
        
		PointT ctPoint;
		ctPoint.x = v_key_trajectory[i](0);  ctPoint.y = v_key_trajectory[i](1); ctPoint.z = v_key_trajectory[i](2);

		pointIdxRadiusSearch.clear();
		pointRadiusSquaredDistance.clear();
		pcl::KdTreeFLANN<PointT> tlsKdTree;
		tlsKdTree.setInputCloud(localDSCloud);
		tlsKdTree.radiusSearch(ctPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

		for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {

			PointT tmp_point;
			tmp_point.x = localDSCloud->points[pointIdxRadiusSearch[j]].x - ctPoint.x;
			tmp_point.y = localDSCloud->points[pointIdxRadiusSearch[j]].y - ctPoint.y;
			tmp_point.z = localDSCloud->points[pointIdxRadiusSearch[j]].z - ctPoint.z;
			// tmp_point.intensity = localDSCloud->points[pointIdxRadiusSearch[j]].intensity;
			localRadiusCloud->push_back(tmp_point);
		}

        MatrixXd coc;
        makeCoViewContextDescriptors(localRadiusCloud, coc);
		v_loc_coc.push_back(coc);
		v_loc_pc.push_back(*localRadiusCloud);

		localRadiusCloud->clear();
	}

	std::cout << "->Descriptor matching..." << std::endl;
	
	omp_set_num_threads(8);
	#pragma omp parallel for
    for (int i = 0; i < v_loc_coc.size(); ++i)
	{
		MatrixXd loc_coc = v_loc_coc[i];
		for (int j = 0; j < v_ref_coc.size(); ++j)
		{
			MatrixXd ref_coc = v_ref_coc[j];
			std::pair<double, int> dis;
			dis = sc.distanceBtnScanContext(loc_coc, ref_coc);
			std::tuple<double, int, int, int> score(dis.first, dis.second, i, j);
            #pragma omp critical
            {
                coc_scores.push_back(score);
            }
		}
	}

	std::sort(coc_scores.begin(), coc_scores.end());

    #pragma omp parallel for
	for(int k = 0; k < 1; ++k)
	{
		std::tuple<double, int, int, int> coc_score_max = coc_scores[k];
		double coc_score = std::get<0>(coc_score_max);
		int coc_shift = std::get<1>(coc_score_max);
		int loc_id = std::get<2>(coc_score_max);
		int ref_id = std::get<3>(coc_score_max);
		cout << "ref_id: " << ref_id << " loc_id: " << loc_id << " score: " << coc_score << " shift: " << coc_shift << endl;

		// // debug: save matched descriptor
		// MatrixXd loc_coc = v_loc_coc[16];
		// MatrixXd ref_coc = v_ref_coc[472];		
		// std::pair<double, double> kRangeColorAxis = std::pair<double, double>{ 0, 30.0 };
		// cv::Mat cvMatrix(loc_coc.rows(), loc_coc.cols(), CV_32FC1);
		// cv::eigen2cv(loc_coc, cvMatrix);
		// cv::Mat colorImg1 = convertColorMappedImg(cvMatrix, kRangeColorAxis);
		// string filename1 = saveBasePath + "/loc_coc_" + std::to_string(loc_id) + ".bmp";
		// cv::imwrite(filename1, colorImg1);		
		// cv::eigen2cv(ref_coc, cvMatrix);
		// cv::Mat colorImg2 = convertColorMappedImg(cvMatrix, kRangeColorAxis);
		// string filename2 = saveBasePath + "/ref_coc_" + std::to_string(ref_id) + ".bmp";
		// cv::imwrite(filename2, colorImg2);

		pcl::PointCloud<PointT>::Ptr loc_coc_matched(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr ref_coc_matched(new pcl::PointCloud<PointT>);
		*loc_coc_matched = v_loc_pc[loc_id];

		// load reference point cloud
		std::string ref_path = saveBasePath + "/VRS/VRS_" + std::to_string(ref_id) + ".pcd";
		pcl::io::loadPCDFile(ref_path, *ref_coc_matched);

		float rot_angle = 360.0 - coc_shift * (360 / sc.PC_NUM_SECTOR);
		rotateYawByDescriptor((*loc_coc_matched), rot_angle);

		// rot_angle
		Eigen::Matrix4f transformation_matrix_1 = Eigen::Matrix4f::Identity();
		transformation_matrix_1(0, 0) = std::cos(rot_angle * M_PI / 180.0);
		transformation_matrix_1(0, 1) = -std::sin(rot_angle * M_PI / 180.0);
		transformation_matrix_1(1, 0) = std::sin(rot_angle * M_PI / 180.0);
		transformation_matrix_1(1, 1) = std::cos(rot_angle * M_PI / 180.0);

		// trimmed ICP
		pcl::PointCloud<PointT>::Ptr transformedTICP(new pcl::PointCloud<PointT>);
		// Eigen::Matrix4f transformation_matrix_2 = doTrimmedICP(loc_coc_matched, ref_coc_matched, transformedTICP, icp_thr);
		Eigen::Matrix4f transformation_matrix_2 = doGeneralizedICP(loc_coc_matched, ref_coc_matched, transformedTICP);

		PointT ref_center_point;
		ref_center_point = (*referenceVirtualCloud)[ref_id];
		Vector3d loc_center_point;
		loc_center_point = v_key_trajectory[loc_id];

		cout << "als_center_point: " << ref_center_point.x << " " << ref_center_point.y << " " << ref_center_point.z << endl;
		cout << "loc_center_point: " << loc_center_point[0] << " " << loc_center_point[1] << " " << loc_center_point[2] << endl;

		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.translation() << -loc_center_point[0], -loc_center_point[1], -loc_center_point[2];
		Eigen::Matrix4f transformation_matrix_0 = transform.matrix();

		transform.translation() << ref_center_point.x, ref_center_point.y, ref_center_point.z;

		pcl::PointCloud<PointT>::Ptr transformedCloud(new pcl::PointCloud<PointT>);
		pcl::transformPointCloud(*transformedTICP, *transformedCloud, transform);
		Eigen::Matrix4f transformation_matrix_3 = transform.matrix();

		Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
		transformation_matrix = transformation_matrix_3 * (transformation_matrix_2 * (transformation_matrix_1 * transformation_matrix_0));

		pcl::transformPointCloud(*localDSCloud, *localTransformedCloud, transformation_matrix);
		cout << "-------------------------------" << endl;
		cout << transformation_matrix << endl;

        #pragma omp critical
        {
            v_icp_pc.push_back(*localTransformedCloud);
            v_icp_scores.push_back(coc_score);
        }
	}

   	// get the mini v_icp_scores index
	int min_index = 0;
	float min_score = v_icp_scores[0];
	cout << "v_icp_scores: " << v_icp_scores[0] << endl;
	for (int i = 1; i < v_icp_scores.size(); i++)
	{
		cout << "v_icp_scores: " << v_icp_scores[i] << endl;
		if (v_icp_scores[i] < min_score)
		{
			min_score = v_icp_scores[i];
			min_index = i;
		}
	}

	for (int i = 0; i < v_icp_scores.size(); i++)
	{
		std::string save_dir = saveBasePath + "result/";
		create_directories_if_not_exist(save_dir);
		std::string save_path = save_dir + "global_" + std::to_string(i) + ".pcd";
		pcl::io::savePCDFileASCII(save_path, v_icp_pc[i]);
	}
}

void CoviewContext::rotateYawByDescriptor(pcl::PointCloud<PointT>& cloud_in, float rotAngle)
{
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	float angle_radians = rotAngle * M_PI / 180.0;
	transform.rotate(Eigen::AngleAxisf(angle_radians, Eigen::Vector3f::UnitZ()));

	pcl::transformPointCloud(cloud_in, cloud_in, transform);
}

Eigen::Matrix4f CoviewContext::doTrimmedICP(pcl::PointCloud<PointT>::Ptr cloud_src, pcl::PointCloud<PointT>::Ptr cloud_tar, pcl::PointCloud<PointT>::Ptr cloud_trans, double ratio)
{
	pcl::recognition::TrimmedICP<PointT, double> tricp;
	tricp.init(cloud_tar);//target
    
	Eigen::Matrix4d transformation_matrixd = Eigen::Matrix4d::Identity();
	tricp.align(*cloud_src, (int)cloud_src->size() * ratio, transformation_matrixd);
	pcl::transformPointCloud(*cloud_src, *cloud_trans, transformation_matrixd);

	Eigen::Matrix4f transformation_matrix = transformation_matrixd.cast<float>();
	return transformation_matrix;
}

Eigen::Matrix4f CoviewContext::doGeneralizedICP(pcl::PointCloud<PointT>::Ptr cloud_src, pcl::PointCloud<PointT>::Ptr cloud_tar, pcl::PointCloud<PointT>::Ptr cloud_trans)
{
	pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
	gicp.setInputSource(cloud_src);
	gicp.setInputTarget(cloud_tar);
	gicp.align(*cloud_trans);
	icp_score = gicp.getFitnessScore();
	Eigen::Matrix4f transformation_matrix = gicp.getFinalTransformation();
	return transformation_matrix;
}

void CoviewContext::makeCoViewContextDescriptors(pcl::PointCloud<PointT>::Ptr cloud_in, MatrixXd &coc)
{
    MatrixXd desc = sc.makeScancontext((*cloud_in));
    // cv::Mat
    cv::Mat cvMatrix(desc.rows(), desc.cols(), CV_32FC1);
    cv::eigen2cv(desc, cvMatrix);

    cv::Mat cvHorizontal = cvMatrix.clone();
    for (int n = 0; n < cvMatrix.cols; n++) {
        double max_height = cvMatrix.at<double>(1, n);
        int max_height_row = 1;
        for (int m = 1; m < cvMatrix.rows; m++) {
            double cur_height = cvMatrix.at<double>(m, n);
			double occlusion_height = m / max_height_row * max_height;
            if (cur_height < 0.5f * occlusion_height) {
                cvHorizontal.at<double>(m, n) = 0.0;
            }
            else {
                max_height = cur_height;
                max_height_row = m;
                cvHorizontal.at<double>(m, n) = cur_height;
            }
        }
    }

    int cloud_size = cloud_in->size();
    int point_per_bin_avr = cloud_size / (sc.PC_NUM_SECTOR * sc.PC_NUM_RING);

    MatrixXd desc_num = sc.makeNumbercontext((*cloud_in));
    cv::Mat cvNum(desc_num.rows(), desc_num.cols(), CV_32FC1);
    cv::eigen2cv(desc_num, cvNum);

    cv::Mat cvVertival = cvNum.clone();
    for (int n = 0; n < cvNum.cols; n++) {
        for (int m = 0; m < cvNum.rows; m++) {
            if (cvVertival.at<float>(m, n) < point_per_bin_avr) {
                cvVertival.at<float>(m, n) = 0.0;
            }
            else {
                cvVertival.at<float>(m, n) = 256.0;
            }
        }
    }

    MatrixXd hor_sc, ver_sc;
    cv::cv2eigen(cvHorizontal, hor_sc);
    cv::cv2eigen(cvVertival, ver_sc);
    MatrixXd ave_height = sc.makeAvrHeightcontext((*cloud_in), hor_sc, ver_sc);

    cv::Mat cvAvrHeight(ave_height.rows(), ave_height.cols(), CV_32FC1);
    cv::eigen2cv(ave_height, cvAvrHeight);

    cv::Mat cvCoViewContext = cvHorizontal.clone();
    for (int n = 0; n < cvHorizontal.cols; n++) {
        for (int m = 0; m < cvHorizontal.rows; m++) {
            if (cvHorizontal.at<float>(m, n) != 0.0)
            {
                if (cvVertival.at<float>(m, n) == 256.0)
                {
                    cvCoViewContext.at<float>(m, n) = cvAvrHeight.at<float>(m, n);
                }
            }
        }
    }

    cv::cv2eigen(cvHorizontal, coc);
}



