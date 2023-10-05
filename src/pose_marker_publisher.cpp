#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <pips/HallucinatedRobotModelConfig.h>



struct ModelParams
{
  double radius, height, floor_tolerance, safety_expansion; 
  
  int model_type;
};

class PoseMarkerPublisher
{
private:
  ros::Subscriber pose_array_sub_;
  ros::NodeHandle nh_, pnh_, cc_nh_;
  ros::Publisher visualization_pub_;
  
  
  bool fillParameters(ModelParams& params)
  {
    if(!cc_nh_.getParamCached("robot_radius", params.radius))
      return false;
    if(!cc_nh_.getParamCached("robot_height", params.height))
      return false;
    if(!cc_nh_.getParamCached("floor_tolerance", params.floor_tolerance))
      return false;
    if(!cc_nh_.getParamCached("safety_expansion", params.safety_expansion))
      return false;
    if(!cc_nh_.getParamCached("model_type", params.model_type))
      return false;

    return true;
  }
  
  bool checkParameters()
  {
    ModelParams params;
    return fillParameters(params);
  }
  
  visualization_msgs::Marker getMarker(geometry_msgs::Pose pose, const ModelParams& params)
  {
    visualization_msgs::Marker marker;
    
    
    
    
    
    
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = .25;
    marker.color.r = 1;
    marker.scale.x = (params.radius+params.safety_expansion)*2;
    marker.scale.y = (params.radius+params.safety_expansion)*2;
    marker.scale.z = params.height-params.floor_tolerance;
    
    pose.position.z += params.height/2 + params.floor_tolerance;
    
    marker.pose.position = pose.position;
    
    if(params.model_type == pips::HallucinatedRobotModel_rectangular || params.model_type == pips::HallucinatedRobotModel_rectangular_ss)
    {
      marker.type = visualization_msgs::Marker::CUBE;
      marker.pose.orientation.z = 1;
    }
    else if(params.model_type == pips::HallucinatedRobotModel_cylindrical || params.model_type == pips::HallucinatedRobotModel_cylindrical_t_vect || params.model_type == pips::HallucinatedRobotModel_cylindrical_t)
    {
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.pose.orientation = pose.orientation;
    }
    
    return marker;
  }
  
  void addMarkers(std::vector<visualization_msgs::Marker>& markers, const geometry_msgs::PoseArray::ConstPtr poses, const ModelParams& params, std::string ns)
  {
    visualization_msgs::Marker marker;
    marker.ns = ns;
    marker.action = visualization_msgs::Marker::DELETEALL;
    
    markers.push_back(marker);
    
    int id=0;
    for(geometry_msgs::Pose pose : poses->poses)
    {
      visualization_msgs::Marker marker = getMarker(pose, params);
      marker.ns = ns;
      marker.id = id++;
      marker.header = poses->header;

      markers.push_back(marker);
    }
    
  }
  
  void addRawMarkers(std::vector<visualization_msgs::Marker>& markers, const geometry_msgs::PoseArray::ConstPtr poses, ModelParams params)
  {
    params.floor_tolerance = 0;
    params.safety_expansion = 0;
    
    addMarkers(markers, poses, params, "simplified_geometry");
  }
  
  void addAdjustedMarkers(std::vector<visualization_msgs::Marker>& markers, const geometry_msgs::PoseArray::ConstPtr poses, ModelParams params)
  {
    addMarkers(markers, poses, params, "adjusted_geometry");
  }
  
  
  void posesCB(const geometry_msgs::PoseArray::ConstPtr pose_array_msg)
  {
    ROS_DEBUG_STREAM("poses callback!");
    
    ModelParams params;
    
    if(fillParameters(params))
    {
      visualization_msgs::MarkerArray::Ptr markers = boost::make_shared<visualization_msgs::MarkerArray>();
      
      addRawMarkers(markers->markers, pose_array_msg, params);
      addAdjustedMarkers(markers->markers, pose_array_msg, params);
      
      visualization_pub_.publish(markers);
    }
  }

public:
  
  PoseMarkerPublisher() :
    nh_(),
    pnh_("~")
  {
    
  }
  
  void init()
  {
    std::string cc_ns;
    pnh_.getParam("cc_params_ns", cc_ns);
    
    cc_nh_ = ros::NodeHandle(cc_ns);
    
    checkParameters();
    
    
    pose_array_sub_ = nh_.subscribe("poses", 5, &PoseMarkerPublisher::posesCB, this);
    visualization_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("markers",5);
    
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "poses_to_markers");
  
  PoseMarkerPublisher p;
  p.init();
  ros::spin();
}
