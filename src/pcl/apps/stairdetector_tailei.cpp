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

// add io
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

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

typedef pcl::PointMoXYZRGB PointIn;
typedef pcl::PointMoXYZRGBNormal PointOut;
pcl::PointCloud<pcl::PointXYZ> cloud;

int numIterations = 0;

class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer"),frame_id(0),inCloud(new pcl::PointCloud<pcl::PointXYZ>),inCloud_(new pcl::PointCloud<pcl::PointXYZ>){tmr.reset();}
    int frame_id;
    double stair_angle_threshold = 0.15;

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_) {
        if (!viewer.wasStopped())
            viewer.showCloud (cloud_);

        pcl::copyPointCloud(*cloud_, *inCloud_);
        printf ("processing frame: %d\n", frame_id);

#if 1
// Create the filtering object
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (inCloud_);
        sor.setLeafSize (0.02, 0.02, 0.02);
        sor.filter (*inCloud);
        //std::cout << "Before: " << inCloud_->width * inCloud_->height << " After: " << inCloud->width * inCloud->height << "\n";
#endif
        
        tmr.reset();
        pcl::PointCloud<PointIn>::Ptr inCloudMo (new pcl::PointCloud<PointIn>);
        pcl::StairDetectionLocal<pcl::PointMoXYZRGB, pcl::PointMoXYZRGBNormal>  stair_detector; 
        pcl::copyPointCloud (*inCloud, *inCloudMo);
        for (size_t i = 0; i < inCloud->size (); i++)
        {
            (*inCloudMo)[i].id = (int) i;
        }

        stair_detector.setInputCloud(inCloudMo);
        pcl::PointCloud<PointIn>::Ptr out = stair_detector.compute();
        
        bool stair_bool = stair_detector.stairdetection(); 

        if (stair_bool == false){
            std::cout << "\033[1;33m There is no stair  \033[0m\n" << std::endl;
        } 
        
        else{
			
            //std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
            //                                    << coefficients->values[1] << " "
            //                                    << coefficients->values[2] << " " 
            //                                    << coefficients->values[3] << std::endl;

            Eigen::Vector3f M; // M: current;
            M << 0.0,0.0,1.0;
            Eigen::Vector3f N  = stair_detector.getRiserNormal();
            
            // sepration angle cos
            double costheta = M.dot(N) / (M.norm() * N.norm());

            cout << "\033[1;33m angle cos : \033[0m\n" << costheta << endl;
            if ( abs(costheta) < stair_angle_threshold ) std::cout << "\033[1;32m Riser is Ready!!  \033[0m\n" << std::endl;
            else {
                std::cout << "\033[1;31m Riser is not Ready!!  \033[0m\n" << std::endl;
            }
        }
        double t = tmr.elapsed();
        std::cout << "TIME: process " << t << std::endl;
        printf("### Computation Done. ###\n");
        printf("============================================\n");


        if ( 0)
        {
            //std::string outPath = "MingPt";
            //outPath.replace (outPath.length()-4, 4, "_out.vtk");
            //pcl::PointCloud<PointOut>::ConstPtr ptr2 = out;
            //pcl::io::savePolygonFileVTK (outPath.c_str (), *pcl::surface::getMesh<PointIn, PointOut>(out) );
            char f[50];
            sprintf (f, "out_%d.pcd", frame_id);
            //pcl::io::savePCDFileBinary (outPath.c_str (), *out);
            // pcl::io::saveMoPcd (f, *out);
        }
        frame_id ++;
    }

    void run (){
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
