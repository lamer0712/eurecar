#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ia_ransac.h>

// matrix and vector for storing final transformation
Eigen::Matrix4f initialTransformation, finalTransformation; 
vector<float> matrixTransformation;

extern "C" {

_declspec (dllexport) int ICP_preALIGNMENT(int	  cloudCols_source, 
										   int	  cloudRows_source,
										   float  *cloudX_source, 
										   float  *cloudY_source, 
										   float  *cloudZ_source,
										   int    cloudCols_target, 
										   int    cloudRows_target, 
										   float  *cloudX_target, 
										   float  *cloudY_target, 
										   float  *cloudZ_target,
										   float  sac_ia_min_sample_distance,
										   float  sac_ia_max_correspondence_distance,
										   int    sac_ia_maximum_iterations,
										   double icp_RANSAC_threshold, 
										   int    icp_maxICP_iterations,
										   double icp_maxCorrespondence_distance,
										   double icp_EuclideanFitness_epsilon,
										   double icp_Transformation_epsilon,
										   int    *icp_converged, 
										   double *icp_score, 
										   float  *final_transformation_matrix,
										   double  normals_radius,
										   double  fpfh_radius,
										   int	   full_registration); 
}

_declspec (dllexport) int ICP_preALIGNMENT(int	  cloudCols_source, 
										   int	  cloudRows_source,
										   float  *cloudX_source, 
										   float  *cloudY_source, 
										   float  *cloudZ_source,
										   int    cloudCols_target, 
										   int    cloudRows_target, 
										   float  *cloudX_target, 
										   float  *cloudY_target, 
										   float  *cloudZ_target,
										   float  sac_ia_min_sample_distance,
										   float  sac_ia_max_correspondence_distance,
										   int    sac_ia_maximum_iterations,
										   double icp_RANSAC_threshold, 
										   int    icp_maxICP_iterations,
										   double icp_maxCorrespondence_distance,
										   double icp_EuclideanFitness_epsilon,
										   double icp_Transformation_epsilon,
										   int    *icp_converged, 
										   double *icp_score, 
										   float  *final_transformation_matrix,
										   double  normals_radius,
										   double  fpfh_radius,
										   int	   full_registration)
{
	// create two PCL boost shared pointers and initialize them
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
	
	// set cloud size source
	cloud_source->width = cloudCols_source;
	cloud_source->height = cloudRows_source;
	cloud_source->points.resize((cloud_source->width)*(cloud_source->height));
	cloud_source->is_dense = true;	
	// set cloud size target
	cloud_target->width = cloudCols_target;
	cloud_target->height = cloudRows_target;
	cloud_target->points.resize(cloud_target->width*cloud_target->height);
	cloud_target->is_dense = true;
	
	
	// fill cloud data target
	float *pX_ref = &cloudX_target[0];
	float *pY_ref = &cloudY_target[0];
	float *pZ_ref = &cloudZ_target[0];
	for(int i=0;i<cloud_target->points.size();i++,pX_ref++,pY_ref++,pZ_ref++) {
		cloud_target->points[i].x = (*pX_ref);
		cloud_target->points[i].y = (*pY_ref);
		cloud_target->points[i].z = (*pZ_ref);
	}
	// fill cloud data new
	float *pX_new = &cloudX_source[0];
	float *pY_new = &cloudY_source[0];
	float *pZ_new = &cloudZ_source[0];
		for(int i=0;i<cloud_source->points.size();i++,pX_new++,pY_new++,pZ_new++) {
		cloud_source->points[i].x = (*pX_new);
		cloud_source->points[i].y = (*pY_new);
		cloud_source->points[i].z = (*pZ_new);
	}	
	
	// SAC-IA + ICP
	if(full_registration == 0)
	{
		// 1. STEP ->  rough initial alignment using the feature descriptors (fast point feature histogram descriptors)

		// TARGET CLOUD FIRST
		// create the normal estimation class (multi-thread compatible, OpenMP), and pass the input dataset to it
		pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::Normal> n_target;
		n_target.setInputCloud(cloud_target);
		// create an empty kdtree representation, and pass it to the normal estimation object.
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_target_normals (new pcl::search::KdTree<pcl::PointXYZ>);
		n_target.setSearchMethod(tree_target_normals);
		// output datasets
		pcl::PointCloud<pcl::Normal>::Ptr normals_target (new pcl::PointCloud<pcl::Normal>);
		// use all neighbors in a sphere of radius
		n_target.setRadiusSearch(normals_radius);
		// compute the normals
		n_target.compute(*normals_target);
	
		// create the FPFH estimation class, and pass the input dataset + normals to it
		pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> f_target;
		f_target.setInputCloud(cloud_target);
		f_target.setInputNormals(normals_target);
		// create an empty kdtree representation, and pass it to the FPFH estimation object.
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_target_fpfh (new pcl::search::KdTree<pcl::PointXYZ>);
		f_target.setSearchMethod(tree_target_fpfh);
		// output datasets
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_target (new pcl::PointCloud<pcl::FPFHSignature33>);
		// use all neighbors in a sphere of radius (has to be larger than normal estimation radius!)
		f_target.setRadiusSearch(fpfh_radius);
		// compute the normals
		f_target.compute(*fpfh_target);

		// SOURCE CLOUD SECOND
		// create the normal estimation class (multi-thread compatible, OpenMP), and pass the input dataset to it
		pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::Normal> n_source;
		n_source.setInputCloud(cloud_source);
		// create an empty kdtree representation, and pass it to the normal estimation object.
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_source_normals (new pcl::search::KdTree<pcl::PointXYZ>);
		n_source.setSearchMethod(tree_source_normals);
		// output datasets
		pcl::PointCloud<pcl::Normal>::Ptr normals_source (new pcl::PointCloud<pcl::Normal>);
		// use all neighbors in a sphere of radius
		n_source.setRadiusSearch(normals_radius);
		// compute the normals
		n_source.compute(*normals_source);
	
		// create the FPFH estimation class, and pass the input dataset + normals to it
		pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> f_source;
		f_source.setInputCloud(cloud_source);
		f_source.setInputNormals(normals_source);
		// create an empty kdtree representation, and pass it to the FPFH estimation object.
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_source_fpfh (new pcl::search::KdTree<pcl::PointXYZ>);
		f_source.setSearchMethod(tree_source_fpfh);
		// output datasets
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_source (new pcl::PointCloud<pcl::FPFHSignature33>);
		// use all neighbors in a sphere of radius (has to be larger than normal estimation radius!)
		f_source.setRadiusSearch(fpfh_radius);
		// compute the normals
		f_source.compute(*fpfh_source);

		// sample consensus initial alignment registration
		pcl::SampleConsensusInitialAlignment<pcl::PointXYZ,pcl::PointXYZ,pcl::FPFHSignature33> sac_ia;
		sac_ia.setMinSampleDistance(sac_ia_min_sample_distance);
		sac_ia.setMaxCorrespondenceDistance(sac_ia_max_correspondence_distance);
		sac_ia.setMaximumIterations(sac_ia_maximum_iterations);

		// input the datasets 
		// target cloud
		sac_ia.setInputTarget(cloud_target);
		sac_ia.setTargetFeatures(fpfh_target);
		// source cloud
		sac_ia.setInputCloud(cloud_source);
		sac_ia.setSourceFeatures(fpfh_source);
		// align the point clouds
		pcl::PointCloud<pcl::PointXYZ>::Ptr sac_ia_registration (new pcl::PointCloud<pcl::PointXYZ>);
		sac_ia.align(*sac_ia_registration);
		// get the transformation
		initialTransformation = sac_ia.getFinalTransformation();

		// 2. STEP -> ICP refined alignment

		// create instance of ICP and set the clouds
		pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
		icp.setInputTarget(cloud_target);
		icp.setInputCloud(sac_ia_registration);
		// set the parameters
		icp.setRANSACOutlierRejectionThreshold(icp_RANSAC_threshold);
		icp.setMaximumIterations(icp_maxICP_iterations);
		icp.setMaxCorrespondenceDistance(icp_maxCorrespondence_distance);
		icp.setEuclideanFitnessEpsilon(icp_EuclideanFitness_epsilon);
		icp.setTransformationEpsilon(icp_Transformation_epsilon);
		// set resultant cloud
		pcl::PointCloud<pcl::PointXYZ> icp_registration;
		// align
		icp.align(icp_registration);	
		// check if converged
		(*icp_converged) = icp.hasConverged();
		// check score
		(*icp_score) = icp.getFitnessScore();	
		// get final transformation (consider sac_ia transformation)
		finalTransformation = icp.getFinalTransformation() * initialTransformation;
	}
	
	// ONLY ICP
	else if (full_registration < 0) {
		// create instance of ICP and set the clouds
		pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
		icp.setInputTarget(cloud_target);
		icp.setInputCloud(cloud_source);
		// set the parameters
		icp.setRANSACOutlierRejectionThreshold(icp_RANSAC_threshold);
		icp.setMaximumIterations(icp_maxICP_iterations);
		icp.setMaxCorrespondenceDistance(icp_maxCorrespondence_distance);
		icp.setEuclideanFitnessEpsilon(icp_EuclideanFitness_epsilon);
		icp.setTransformationEpsilon(icp_Transformation_epsilon);
		// set resultant cloud
		pcl::PointCloud<pcl::PointXYZ> icp_registration;
		// align
		icp.align(icp_registration);	
		// check if converged
		(*icp_converged) = icp.hasConverged();
		// check score
		(*icp_score) = icp.getFitnessScore();	
		// get final transformation (consider sac_ia transformation)
		finalTransformation = icp.getFinalTransformation();
	}

	// ONLY SAC-IA
	else {
		// 1. STEP ->  rough initial alignment using the feature descriptors (fast point feature histogram descriptors)

		// TARGET CLOUD FIRST
		// create the normal estimation class (multi-thread compatible, OpenMP), and pass the input dataset to it
		pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::Normal> n_target;
		n_target.setInputCloud(cloud_target);
		// create an empty kdtree representation, and pass it to the normal estimation object.
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_target_normals (new pcl::search::KdTree<pcl::PointXYZ>);
		n_target.setSearchMethod(tree_target_normals);
		// output datasets
		pcl::PointCloud<pcl::Normal>::Ptr normals_target (new pcl::PointCloud<pcl::Normal>);
		// use all neighbors in a sphere of radius
		n_target.setRadiusSearch(normals_radius);
		// compute the normals
		n_target.compute(*normals_target);
	
		// create the FPFH estimation class, and pass the input dataset + normals to it
		pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> f_target;
		f_target.setInputCloud(cloud_target);
		f_target.setInputNormals(normals_target);
		// create an empty kdtree representation, and pass it to the FPFH estimation object.
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_target_fpfh (new pcl::search::KdTree<pcl::PointXYZ>);
		f_target.setSearchMethod(tree_target_fpfh);
		// output datasets
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_target (new pcl::PointCloud<pcl::FPFHSignature33>);
		// use all neighbors in a sphere of radius (has to be larger than normal estimation radius!)
		f_target.setRadiusSearch(fpfh_radius);
		// compute the normals
		f_target.compute(*fpfh_target);

		// SOURCE CLOUD SECOND
		// create the normal estimation class (multi-thread compatible, OpenMP), and pass the input dataset to it
		pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::Normal> n_source;
		n_source.setInputCloud(cloud_source);
		// create an empty kdtree representation, and pass it to the normal estimation object.
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_source_normals (new pcl::search::KdTree<pcl::PointXYZ>);
		n_source.setSearchMethod(tree_source_normals);
		// output datasets
		pcl::PointCloud<pcl::Normal>::Ptr normals_source (new pcl::PointCloud<pcl::Normal>);
		// use all neighbors in a sphere of radius
		n_source.setRadiusSearch(normals_radius);
		// compute the normals
		n_source.compute(*normals_source);
	
		// create the FPFH estimation class, and pass the input dataset + normals to it
		pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> f_source;
		f_source.setInputCloud(cloud_source);
		f_source.setInputNormals(normals_source);
		// create an empty kdtree representation, and pass it to the FPFH estimation object.
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_source_fpfh (new pcl::search::KdTree<pcl::PointXYZ>);
		f_source.setSearchMethod(tree_source_fpfh);
		// output datasets
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_source (new pcl::PointCloud<pcl::FPFHSignature33>);
		// use all neighbors in a sphere of radius (has to be larger than normal estimation radius!)
		f_source.setRadiusSearch(fpfh_radius);
		// compute the normals
		f_source.compute(*fpfh_source);

		// sample consensus initial alignment registration
		pcl::SampleConsensusInitialAlignment<pcl::PointXYZ,pcl::PointXYZ,pcl::FPFHSignature33> sac_ia;
		sac_ia.setMinSampleDistance(sac_ia_min_sample_distance);
		sac_ia.setMaxCorrespondenceDistance(sac_ia_max_correspondence_distance);
		sac_ia.setMaximumIterations(sac_ia_maximum_iterations);

		// input the datasets 
		// target cloud
		sac_ia.setInputTarget(cloud_target);
		sac_ia.setTargetFeatures(fpfh_target);
		// source cloud
		sac_ia.setInputCloud(cloud_source);
		sac_ia.setSourceFeatures(fpfh_source);
		// align the point clouds
		pcl::PointCloud<pcl::PointXYZ>::Ptr sac_ia_registration (new pcl::PointCloud<pcl::PointXYZ>);
		sac_ia.align(*sac_ia_registration);
		// get the transformation
		initialTransformation = sac_ia.getFinalTransformation();
		finalTransformation = initialTransformation;
	}

	// resize vetor to fit
	matrixTransformation.resize(finalTransformation.size());
	// pointer to vector
	float *p_matrixTransformation = &matrixTransformation[0];
	for(int j=0;j<finalTransformation.rows();j++) {
		for(int i=0;i<finalTransformation.cols();i++,p_matrixTransformation++)
			(*p_matrixTransformation) = finalTransformation(i,j);			
	}
	// copy transformation matrix
	memcpy(final_transformation_matrix,&matrixTransformation[0],matrixTransformation.size()*sizeof(float));
		
	return 0;	
}