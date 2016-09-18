#include "machine.h"

void Machine::start()
{
    cout << "SSSSTTTTAAAARRRRTTTTEEEEDDD" << endl;
    ModeState::init(modeState, start_mode);
}

void Machine::setCloudRGBA(const pcl::PointCloud<pcl::PointXYZRGBA> &_input)
{
    pcl::copyPointCloud(_input, * cloudXYZRGBA);
    convertToXYZ(* cloudXYZRGBA, *cloudXYZ);
}

void Machine::convertToXYZ(const pcl::PointCloud<pcl::PointXYZRGBA> & input, pcl::PointCloud<pcl::PointXYZ> & output)
{
    pcl::copyPointCloud(input, output);
}
