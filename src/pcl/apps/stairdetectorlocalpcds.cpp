/*
 * Ming: Local Model Only
 *
 *      Takes in a folder of pcds and infinitly reads them and send them to the stair detection class
 */

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
//#include <pcl_visualization/pcl_visualizer.h>

#include "pcl/common/fhg_point_types.h"
#include "pcl/utils/pointcloud_utils.h"
#include "pcl/types/plane3d.h"
#include "pcl/common/color.h"

#include "pcl/common/fhg_point_types.h"

#include <pcl/features/normal_3d.h>

#include "pcl/opencv/cvutils.h"
#include "pcl/opencv/edgesdetector.h"

#include "pcl/models/localmodel.h"
#include "pcl/apps/stairdetectionlocal.h" // Ming model
#include <pcl/filters/random_sample.h>

#include "pcl/io/mo_io.h"
#include "pcl/surface/surface_utils.h"

//typedef pcl::PointXYZRGB PointIn;
//typedef pcl::PointXYZRGBNormal PointOut;

typedef pcl::PointMoXYZRGB PointIn;
typedef pcl::PointMoXYZRGBNormal PointOut;
pcl::PointCloud<PointIn> cloud;

pcl::StairDetectionLocal<PointIn, PointOut> demo;
int numIterations = 0;
void renderCloud (std::string path)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud_ (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<PointIn>::Ptr inCloudMo (new pcl::PointCloud<PointIn>);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (path.c_str (), *inCloud_) == -1)  //* load the file
  {
    char c[200];
    sprintf (c, "Couldn't read file %s\n", path.c_str ());
    throw std::runtime_error (c);
  }
  printf ("processing pcd file: %s\n", path.c_str ());



#if 1
// add voxel sample

// Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (inCloud_);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (*inCloud);


// random sample
  //pcl::RandomSample<pcl::PointXYZRGB> sor;
  //sor.setInputCloud (inCloud_);
  //sor.setSample(5000);
  //sor.filter (*inCloud);


std::cout << "Before: " << inCloud_->width * inCloud_->height << " After: " << inCloud->width * inCloud->height << "\n";
std::cout << "Before: W: " << inCloud_->width << " H: " << inCloud_->height << "\n";
std::cout << "After: W: " << inCloud->width  << " H: " << inCloud->height << "\n";


#endif


  pcl::copyPointCloud (*inCloud, *inCloudMo);
//  printf("pcds id=\n");
  for (size_t i = 0; i < inCloud->size (); i++)
  {
    (*inCloudMo)[i].id = (int) i;
  }

  size_t index = path.find_last_of ("/");
  std::string s = path.substr (index + 1, path.length () - index);
  std::string rawPath = s.substr (0, s.length () - 4);

//  demo.setInputCloud (inCloud);
  demo.setInputCloud (inCloudMo);
  demo.setCloudName (rawPath);
//  printf("rawPath=%s\n", rawPath.c_str());
printf("Height of inCloud: %d \n", inCloud->height);
  pcl::PointCloud<PointIn>::Ptr out = demo.compute ();

printf("### Computation Done. ###\n");

  std::string outPath = s;

  if (out->size () > 0)
  {
    outPath.replace (s.length () - 4, 4, "_out.vtk");
//    pcl::PointCloud<PointOut>::ConstPtr ptr2 = out;
//    pcl::io::savePolygonFileVTK (outPath.c_str (), *pcl::surface::getMesh<PointIn, PointOut>(out) );
    outPath.replace (outPath.length () - 4, 4, ".pcd");
    char f[50];
    sprintf (f, "out_%d.pcd", numIterations);
//    pcl::io::savePCDFileBinary (outPath.c_str (), *out);
    pcl::io::saveMoPcd (f, *out);
  }

}
int main (int argc, char **argv)
{
  //std::cout << "numinputs=" << argc - 1 << std::endl;
  for (size_t i = 1; i < (size_t) argc; i++)
  {
    std::string p (argv[i]);
    renderCloud (argv[i]);
  }
  return 0;
}

