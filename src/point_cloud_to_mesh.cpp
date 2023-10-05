#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/common/common.h>
#include <visualization_msgs/Marker.h>

// For unorganized approach
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>


  typedef pcl::PointXYZ PCLPoint;
  typedef pcl::PointCloud<PCLPoint> PCLPointCloud;
  

class PointCloudToMesh
{
private:
  ros::Subscriber pointcloud_sub_;
  ros::NodeHandle nh_;
  ros::Publisher visualization_pub_;
  
  
  
  void unorganized(PCLPointCloud::Ptr& cloud, std::vector<pcl::Vertices>& triangles)
  {
    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures
    
    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals
    
    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);
    
    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh polymesh;
    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.025);
    
    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);
    
    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (polymesh);
    
    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    
    std::swap(polymesh.polygons, triangles);
    
  }
  
  void organized(PCLPointCloud::Ptr& cloud, std::vector<pcl::Vertices>& triangles)
  {
    
    pcl::OrganizedFastMesh<PCLPoint> ofm;
    
    ROS_DEBUG_STREAM("Number of points: " << cloud->size());
    
    // Set parameters
    ofm.setInputCloud (cloud);
    ofm.setMaxEdgeLength (1.5);
    ofm.setTrianglePixelSize (1);
    ofm.setTriangulationType (pcl::OrganizedFastMesh<PCLPoint>::TRIANGLE_ADAPTIVE_CUT);
    //ofm.storeShadowedFaces(true);
    
    // Reconstruct
    ofm.reconstruct (triangles);
    ROS_DEBUG_STREAM("reconstruction complete! " << triangles.size() << " vertices");
  }

  void pointCloudCb(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
  {
    ROS_DEBUG_STREAM("point cloud callback!");
    
    PCLPointCloud::Ptr cloud (new PCLPointCloud);    
    pcl::fromROSMsg(*point_cloud_msg, *cloud);
    
    // Init objects
    std::vector<pcl::Vertices> triangles;
    
    int finite=0;
    for(PCLPoint point : *cloud)
    {
      if(pcl::isFinite (point))
        finite++;
    }
    ROS_DEBUG_STREAM("Num finite pnts: " << finite ); //TODO: only calculate this if debug logging enabled
    
    
    organized(cloud,triangles);
    
    
    visualization_msgs::Marker marker;
    marker.header = point_cloud_msg->header;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "surface";
    marker.color.a = .25;
    marker.color.b = 1;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    
    
    for(pcl::Vertices vertices : triangles)
    {
      for(auto i = vertices.vertices.rbegin(); i != vertices.vertices.rend(); ++i )
      //for(int index : vertices.vertices)
      {
        PCLPoint point = (*cloud)[*i];
        geometry_msgs::Point point_msg;
        point_msg.x = point.x;
        point_msg.y = point.y;
        point_msg.z = point.z;
        
        marker.points.push_back(point_msg);
      }
    }
    
    visualization_pub_.publish(marker);
  }

public:
  
  void init()
  {
    pointcloud_sub_ = nh_.subscribe("points", 5, &PointCloudToMesh::pointCloudCb, this);
    visualization_pub_ = nh_.advertise<visualization_msgs::Marker>("mesh",5);
          
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_to_mesh");
  
  PointCloudToMesh p;
  p.init();
  ros::spin();
}
