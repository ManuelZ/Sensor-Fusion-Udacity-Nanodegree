/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
							   int maxIterations,
							   float distanceTol) {

	auto startTime = std::chrono::steady_clock::now();

	std::cout << "Starting Ransac\n";
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	
	for(int i = 0; i < maxIterations; i++) {
		std::cout << "START: Iteration " << i << "\n";
		
		std::unordered_set<int> inliers;
		
		while (inliers.size() < 3) {
			int random_point = (static_cast<double>(std::rand()) / RAND_MAX) * cloud->points.size();
			inliers.insert(random_point);
		}

		auto iterator = inliers.begin();
		
		pcl::PointXYZ p1 = cloud->points[*iterator];
		float x1 = p1.x;
		float y1 = p1.y;
		float z1 = p1.z;
		++iterator;

		pcl::PointXYZ p2 = cloud->points[*iterator];
		float x2 = p2.x;
		float y2 = p2.y;
		float z2 = p2.z;
		++iterator;
		
		pcl::PointXYZ p3 = cloud->points[*iterator];
		float x3 = p3.x;
		float y3 = p3.y;
		float z3 = p3.z;

		std::tuple<float, float, float> v1(x2 - x1, y2 - y1, z2 - z1);
		std::tuple<float, float, float> v2(x3 - x1, y3 - y1, z3 - z1);

		std::tuple<float, float, float> normal((y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1),
											   (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1),
											   (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1));

		float A = std::get<0>(normal);
		float B = std::get<1>(normal);
		float C = std::get<2>(normal);
		float D = -(A * x1 + B * y1 + C * z1);

		// std::cout << "The line's equation is: " << A << "x" << " + " << B << "y" << " + " << C << "\n";
		
		for(int i = 0; i < cloud->points.size(); i++)  {
			pcl::PointXYZ p0 = cloud->points[i];

			if (inliers.count(i) > 0)
				continue;

			float d = std::fabs(A * p0.x + B * p0.y + C * p0.z + D) / std::sqrt(A * A + B * B + C * C);
			// std::cout << "Point " << "(" << p0.x << "," << p0.y << "), " << "distance to line: " << d << "\n";
			
			if (d <= distanceTol) {
				inliers.insert(i);
			}
		}
		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}
		std::cout << "END: Iteration " << i << ", with " << inliers.size() << " inlier points.\n\n";
	}

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC for planes took " << elapsedTime.count() << " milliseconds" << std::endl;


	
	// Randomly sample a two-point-subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int getDistanceToLine(pcl::PointXYZ p) {
	std::cout << p.x;
	return 1;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
