
#include "path_array_display.h"

 
int main()
{
  rviz::PathArrayDisplay pathArray;
  //pathArray.onInitialize();
  pathArray.reset();
 
}
/*

#include <pluginlib/class_loader.h>
#include "path_display_temp.h"


int main(int argc, char** argv)
{
  std::string baseType = "rviz::Display";
  pluginlib::ClassLoader<rviz::Display> display_loader("rviz", baseType);

  try
  {
    std::string libName= "pips_visualization/Path";
    display_loader.refreshDeclaredClasses();
    std::vector<std::string> xmlPathsVec = display_loader.getPluginXmlPaths();
    
    std::string xmlPaths;
    for(std::vector<std::string>::const_iterator i = xmlPathsVec.begin(); i != xmlPathsVec.end(); ++i)
    {
      xmlPaths = xmlPaths + ' ' + *i;
    }
    ROS_INFO_STREAM("Paths to xml manifests for base type " << baseType << ": " << xmlPaths);
    
    std::vector<std::string> regLibsVec = display_loader.getRegisteredLibraries();
    
    std::string regLibs;
    for(std::vector<std::string>::const_iterator i = regLibsVec.begin(); i != regLibsVec.end(); ++i)
    {
      regLibs = regLibs + ' ' + *i;
    }
    ROS_INFO_STREAM("Registered libraries for base type " << baseType << "): " << regLibs);
    
    std::vector<std::string> decClassesVec = display_loader.getDeclaredClasses();
    
    std::string decClasses;
    for(std::vector<std::string>::const_iterator i = decClassesVec.begin(); i != decClassesVec.end(); ++i)
    {
      decClasses = decClasses + ' ' + *i;
    }
    ROS_INFO_STREAM("Declared classes for base type " << baseType << "): " << decClasses);
    
    std::string name = display_loader.getName(libName);
    std::string classType = display_loader.getClassType(libName);
    std::string classPackage = display_loader.getClassPackage(libName);
    std::string classDesc = display_loader.getClassDescription(libName);
    std::string libPath = display_loader.getClassLibraryPath(libName);
    std::string manifestPath = display_loader.getPluginManifestPath(libName);
    
    ROS_INFO_STREAM("Class name: " << name << ", class type: " << classType << ", class package: " << classPackage << ", class description: " << classDesc << ", manifest path: " << manifestPath);
    ROS_INFO_STREAM("Path to library: " << libPath);
    
    display_loader.loadLibraryForClass(libName);
    
    //boost::shared_ptr<rviz::Display> a = display_loader.createInstance(libName);
    //a.reset();


  //  ROS_INFO("Triangle area: %.2f", triangle->area());
  //  ROS_INFO("Square area: %.2f", square->area());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }
  

  return 0;
}
  
*/