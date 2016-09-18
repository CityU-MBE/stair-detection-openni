/**
 * @file   ground_only.cpp
 * @author Ming Liu [mingliu@cityu.edu.hk]
 * It can be the on-ground detection
 * It can be the end of stair-case top detection as well.
 */

/*#include "ground_only.h"*/

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

#include "pcl/io/mo_io.h"
#include "pcl/surface/surface_utils.h"

//typedef pcl::PointXYZRGB PointIn;
//typedef pcl::PointXYZRGBNormal PointOut;

// add io
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

//////////////
#include <iostream>
#include <ctime>

class Timer
{
public:
    Timer() { clock_gettime(CLOCK_REALTIME, &beg_); }

    double elapsed() {
        clock_gettime(CLOCK_REALTIME, &end_);
        return end_.tv_sec - beg_.tv_sec +
            (end_.tv_nsec - beg_.tv_nsec) / 1000000000.;
    }

    void reset() { clock_gettime(CLOCK_REALTIME, &beg_); }

private:
    timespec beg_, end_;
};
//////////////

//typedef pcl::PointMoXYZRGB PointIn;
//typedef pcl::PointMoXYZRGBNormal PointOut;
pcl::PointCloud<pcl::PointXYZ> cloud;

int numIterations = 0;

class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer"),frame_id(0),inCloud(new pcl::PointCloud<pcl::PointXYZ>),inCloud_(new pcl::PointCloud<pcl::PointXYZ>){tmr.reset();}
    int frame_id;

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_) 
        {
            //     tmr.reset();
                        
            if (!viewer.wasStopped())
                viewer.showCloud (cloud_);
            //     double t = tmr.elapsed();
            //     std::cout << "TIME: display " << t << std::endl;
            //     tmr.reset();

            pcl::copyPointCloud(*cloud_, *inCloud_);
            printf ("processing frame: %d\n", frame_id);

#if 1
// Create the filtering object
            pcl::VoxelGrid<pcl::PointXYZ> sor;
            sor.setInputCloud (inCloud_);
            sor.setLeafSize (0.02, 0.02, 0.02);
            sor.filter (*inCloud);

            std::cout << "Before: " << inCloud_->width * inCloud_->height << " After: " << inCloud->width * inCloud->height << "\n";

#endif

            //      t = tmr.elapsed();
            //      std::cout << "TIME: start " << t << std::endl;
            //      tmr.reset();

            tmr.reset();
            pcl::StairDetectionLocal<> stairdtect_;

            /*
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            // Optional
            seg.setOptimizeCoefficients (true);
            // Mandatory
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (0.01);

            seg.setInputCloud (inCloud);
            seg.segment (*inliers, *coefficients);

            if (inliers->indices.size () == 0)
            {
                PCL_ERROR ("Could not estimate a planar model for the given dataset.");
                return;
            }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

  Eigen::Vector3f M, N, axis, tmp; // M: current; N: reference; axis: rotation axis
  M << coefficients->values[0], coefficients->values[1], coefficients->values[2];
  N << 0.0, 1.0, 0.0;

  // sepration angle cos
  double costheta = M.dot(N) / (M.norm() * N.norm());

  cout << "angle cos:" << costheta << endl;

  if ( abs(costheta) > 0.8 ) std::cout << "\033[1;32mbold DETECT GROUND !! \033[0m\n" << std::endl;

  

  
                        
                        double t = tmr.elapsed();
                        std::cout << "TIME: process " << t << std::endl;
                        printf("### Computation Done. ###\n");
                        printf("============================================\n");


                        if ( 0)
                        {
                                //std::string outPath = "MingPt";
                                //outPath.replace (outPath.length()-4, 4, "_out.vtk");
                                //    pcl::PointCloud<PointOut>::ConstPtr ptr2 = out;
                                //    pcl::io::savePolygonFileVTK (outPath.c_str (), *pcl::surface::getMesh<PointIn, PointOut>(out) );
                                char f[50];
                                sprintf (f, "out_%d.pcd", frame_id);
                                //    pcl::io::savePCDFileBinary (outPath.c_str (), *out);
                               // pcl::io::saveMoPcd (f, *out);
                        }

                    */
                        frame_id ++;
                }

                void run ()
                {
                        pcl::Grabber* interface = new pcl::OpenNIGrabber();

                        boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
                                boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

                        interface->registerCallback (f);

                        interface->start ();

                        while (!viewer.wasStopped())
                        {
                                boost::this_thread::sleep (boost::posix_time::seconds (1));
                        }

                        interface->stop ();
                }

                pcl::visualization::CloudViewer viewer;
                pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, inCloud_;
                Timer tmr;
};




int main (int argc, char **argv)
{
        SimpleOpenNIViewer v;
        v.run ();
        return 0;
}
