#include "scan_map_icp/scan_map_icp.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_map_icp_node");
  scan_map_icp::ScanMapIcp icp;
  return 0;
}
