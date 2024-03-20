#include <ros/ros.h>
#include "data_collection/data_collection_manager.h"

using namespace data_collection;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "data_collect_node");
  ros::NodeHandle nh("~");

  DataCollectionManager data_collect;
  data_collect.init(nh);


  // ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}

// 