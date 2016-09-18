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

typedef pcl::PointMoXYZRGB PointIn;
typedef pcl::PointMoXYZRGBNormal PointOut;
pcl::PointCloud<PointIn> cloud;

pcl::StairDetectionLocal<PointIn, PointOut> demo;
int numIterations = 0;

class SimpleOpenNIViewer
{
        public:
                SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer"),frame_id(0),inCloud(new pcl::PointCloud<pcl::PointXYZRGB>),inCloud_(new pcl::PointCloud<pcl::PointXYZRGB>){tmr.reset();}
                int frame_id;

                void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_) 
                {
                   //     tmr.reset();
                        
                        if (!viewer.wasStopped())
                                viewer.showCloud (cloud_);
                   //     double t = tmr.elapsed();
                   //     std::cout << "TIME: display " << t << std::endl;
                   //     tmr.reset();

                        pcl::copyPointCloud(*cloud_, *inCloud_);
                        pcl::PointCloud<PointIn>::Ptr inCloudMo (new pcl::PointCloud<PointIn>);
                        printf ("processing frame: %d\n", frame_id);

#if 1
// Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (inCloud_);
  sor.setLeafSize (0.02, 0.02, 0.02);
  sor.filter (*inCloud);

std::cout << "Before: " << inCloud_->width * inCloud_->height << " After: " << inCloud->width * inCloud->height << "\n";

#endif
                        pcl::copyPointCloud (*inCloud, *inCloudMo);
                        //  printf("pcds id=\n");
                        for (size_t i = 0; i < inCloud->size (); i++)
                        {
                                (*inCloudMo)[i].id = (int) i;
                        }

                  //      t = tmr.elapsed();
                  //      std::cout << "TIME: start " << t << std::endl;
                  //      tmr.reset();

                        // Ming: do stair stuff:
                        demo.setInputCloud (inCloudMo);
                        demo.setCloudName ("MingStair");

                        tmr.reset();
                        pcl::PointCloud<PointIn>::Ptr out = demo.compute ();
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
                                pcl::io::saveMoPcd (f, *out);
                        }

                        frame_id ++;
                }

                void run ()
                {
                        pcl::Grabber* interface = new pcl::OpenNIGrabber();

                        boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
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
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, inCloud_;
                Timer tmr;
};




int main (int argc, char **argv)
{
        SimpleOpenNIViewer v;
        v.run ();
        return 0;
}

