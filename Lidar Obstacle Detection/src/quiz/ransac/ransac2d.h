#include <unordered_set>
#include <pcl/common/common.h>

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
							   int maxIterations,
							   float distanceTol);