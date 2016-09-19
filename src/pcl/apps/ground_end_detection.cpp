/**
 * @file   ground_end_detection.cpp
 * @by Haoyang Ye [hyyezju@gmail.com]
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

//typedef pcl::PointMoXYZRGB PointIn;
//typedef pcl::PointMoXYZRGBNormal PointOut;
pcl::PointCloud<pcl::PointXYZ> cloud;

int numIterations = 0;

class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer"),frame_id(0),inCloud(new pcl::PointCloud<pcl::PointXYZ>),inCloud_(new pcl::PointCloud<pcl::PointXYZ>){tmr.reset();}
    int frame_id;

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_) {

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

            if (!viewer.wasStopped()) {
                viewer.showCloud (cloud_);
            }
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

            // if (inliers->indices.size () == 0)
            // {
            //     PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            //     return;
            // }

            /** ------ Create the filtering object - start ------
            * ref: http://pointclouds.org/documentation/tutorials/extract_indices.php
            */
            int i = 0, nr_points = (int) inCloud->points.size ();
            pcl::ExtractIndices<pcl::PointXYZ> extract;

            // While 30% of the original cloud is still there
            while (inCloud->points.size () > 0.1 * nr_points) {
                seg.setInputCloud (inCloud);
                seg.segment (*inliers, *coefficients);

                if (inliers->indices.size () == 0) {
                  std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
                  break;
                }

                Eigen::Vector3f M, N, axis, tmp; // M: current; N: reference; axis: rotation axis
                M << coefficients->values[0], coefficients->values[1], coefficients->values[2];
                N << 0.0, 1.0, 0.0;

                // sepration angle cos
                double costheta = M.dot(N) / (M.norm() * N.norm());

                // cout << "angle cos:" << costheta << endl;

                // Extract the inliers
                extract.setInputCloud (inCloud);
                extract.setIndices (inliers);
                extract.setNegative (false);
                extract.filter (*cloud_p);
                // std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

                // Create the filtering object
                extract.setNegative (true);
                extract.filter (*cloud_f);
                inCloud.swap (cloud_f); // inCloud

                i++;

                /** threshold of Ground's parallelity
                *@ {double}
                *@ threshold might be changed after abs(costheta)
                */
                if ( abs(costheta) > 0.85 && coefficients->values[3] < 0.30) {
                    std::cerr << "\033[1;32m PLANE DETECTED!! \033[0m" << std::endl;
                    if (!(coefficients->values[3] < 0.30 && coefficients->values[3] > 0)) {
                        // Ground should have a appropriate height.
                        std::cerr << "\033[1;32m ERROR: PLANE height is!! \033[0m" << -coefficients->values[3] << std::endl;
                        continue;
                    }

                    int pPointsNum = (int) cloud_p->points.size (); // number of filtered points -- the plane

                    double maxX = -10000, minX = 10000, meanX = 0;
                    // double maxY = -10000, minY = 10000, meanY = 0;
                    double maxZ = -10000, minZ = 10000, meanZ = 0;
                    for (int index = 0; index < pPointsNum; index++) {
                        pcl::PointXYZ tmpPoint;
                        tmpPoint = cloud_p->points[index];
                        if (tmpPoint.z > 2.5 || abs(tmpPoint.x) > 0.2) {
                            // Points with z bigger than 3.0
                            // should not be within a GROUND_END.
                            continue;
                        }
                        // std::cerr << "    " << tmpPoint.x << " "
                        // << tmpPoint.y << " "
                        // << tmpPoint.z << std::endl;
                        // if (index > 10) {
                        //     break;
                        // }
                        // if (maxX < tmpPoint.x) maxX = tmpPoint.x;
                        // if (minX > tmpPoint.x) minX = tmpPoint.x;
                        // if (maxY < tmpPoint.y) maxY = tmpPoint.y;
                        // if (minY > tmpPoint.y) minY = tmpPoint.y;
                        if (maxZ < tmpPoint.z) maxZ = tmpPoint.z;
                        if (minZ > tmpPoint.z) minZ = tmpPoint.z;
                        meanX += tmpPoint.x;
                        // meanY += tmpPoint.y;
                        meanZ += tmpPoint.z;
                    }
                    // meanX /= pPointsNum;
                    // meanY /= pPointsNum;
                    meanZ /= pPointsNum;
                    // std::cerr << "    minX: " << minX << "    maxX: " << maxX << "    meanX: " << meanX << std::endl;
                    // std::cerr << "    minY: " << minY << "    maxY: " << maxY << "    meanY: " << meanY << std::endl;
                    std::cerr << "    minZ: " << minZ << "    maxZ: " << maxZ << "    meanZ: " << meanZ << std::endl;


                    std::cerr << "    GROUND GROUND GROUND GROUND!!!!" << " i: "<< i << " costheta: " << costheta << std::endl;
                    std::cerr << "\033[1;32m    GROUND GROUND GROUND GROUND!!!! height of ground is \033[0m" <<  - coefficients->values[3] << "m." << std::endl;

                    if (maxZ - minZ > 0.5) {
                        continue;
                    }
                    std::cerr << "\033[1;31m    GROUND END GROUND END!!!!\033[0m" << std::endl;
                    return;
                }

            }




            double t = tmr.elapsed();
            std::cout << "TIME: process " << t << std::endl;
            printf("### Computation Done. ###\n");
            printf("============================================\n");


            if (0) {
                    //std::string outPath = "MingPt";
                    //outPath.replace (outPath.length()-4, 4, "_out.vtk");
                    //    pcl::PointCloud<PointOut>::ConstPtr ptr2 = out;
                    //    pcl::io::savePolygonFileVTK (outPath.c_str (), *pcl::surface::getMesh<PointIn, PointOut>(out) );
                    char f[50];
                    sprintf (f, "out_%d.pcd", frame_id);
                    //    pcl::io::savePCDFileBinary (outPath.c_str (), *out);
                   // pcl::io::saveMoPcd (f, *out);
            }

            frame_id ++;
    }

    // void oneFrame_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_) {

    //         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    //         if (!viewer.wasStopped()) {
    //             viewer.showCloud (cloud_);
    //         }
    //         //     double t = tmr.elapsed();
    //         //     std::cout << "TIME: display " << t << std::endl;
    //         //     tmr.reset();

    //         pcl::copyPointCloud(*cloud_, *inCloud_);
    //         printf ("processing frame: %d\n", frame_id);

    //         #if 1
    //         // Create the filtering object
    //                     pcl::VoxelGrid<pcl::PointXYZ> sor;
    //                     sor.setInputCloud (inCloud_);
    //                     sor.setLeafSize (0.02, 0.02, 0.02);
    //                     sor.filter (*inCloud);

    //                     std::cout << "Before: " << inCloud_->width * inCloud_->height << " After: " << inCloud->width * inCloud->height << "\n";

    //         #endif

    //         //      t = tmr.elapsed();
    //         //      std::cout << "TIME: start " << t << std::endl;
    //         //      tmr.reset();

    //         tmr.reset();

    //         pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    //         pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    //         // Create the segmentation object
    //         pcl::SACSegmentation<pcl::PointXYZ> seg;
    //         // Optional
    //         seg.setOptimizeCoefficients (true);
    //         // Mandatory
    //         seg.setModelType (pcl::SACMODEL_PLANE);
    //         seg.setMethodType (pcl::SAC_RANSAC);
    //         seg.setDistanceThreshold (0.01);

    //         // if (inliers->indices.size () == 0)
    //         // {
    //         //     PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    //         //     return;
    //         // }

    //         /** ------ Create the filtering object - start ------
    //         * ref: http://pointclouds.org/documentation/tutorials/extract_indices.php
    //         */
    //         int i = 0, nr_points = (int) inCloud->points.size ();
    //         pcl::ExtractIndices<pcl::PointXYZ> extract;

    //         // While 30% of the original cloud is still there
    //         while (inCloud->points.size () > 0.3 * nr_points) {
    //             seg.setInputCloud (inCloud);
    //             seg.segment (*inliers, *coefficients);

    //             if (inliers->indices.size () == 0) {
    //               std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    //               return;
    //             }

    //             std::cerr << "Model coefficients: " << coefficients->values[0] << " "
    //                             << coefficients->values[1] << " "
    //                             << coefficients->values[2] << " "
    //                             << coefficients->values[3] << std::endl;

    //             std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

    //             Eigen::Vector3f M, N, axis, tmp; // M: current; N: reference; axis: rotation axis
    //             M << coefficients->values[0], coefficients->values[1], coefficients->values[2];
    //             N << 0.0, 1.0, 0.0;

    //             // sepration angle cos
    //             double costheta = M.dot(N) / (M.norm() * N.norm());

    //             cout << "angle cos:" << costheta << endl;
    //             i++;

    //             if ( abs(costheta) > 0.8 ) {
    //                 if (!(coefficients->values[3] < -0.5 && coefficients->values[3] > -0.8)) continue;

    //                 std::cout << "\033[1;32mbold DETECT GROUND !! \033[0m\n" << std::endl;

    //                 // Extract the inliers
    //                 extract.setInputCloud (inCloud);
    //                 extract.setIndices (inliers);
    //                 extract.setNegative (false);
    //                 extract.filter (*cloud_p);
    //                 std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    //                 // Create the filtering object
    //                 extract.setNegative (true);
    //                 extract.filter (*cloud_f);
    //                 inCloud.swap (cloud_f); // inCloud

    //                 /** ------ Create the filtering object - end------ */

    //                 int pPointsNum = (int) cloud_p->points.size (); // number of filtered points -- the plane

    //                 // std::cerr << pPointsNum << std::endl;

    //                 double maxX = -10000, minX = 10000, meanX = 0;
    //                 double maxY = -10000, minY = 10000, meanY = 0;
    //                 double maxZ = -10000, minZ = 10000, meanZ = 0;
    //                 for (int index = 0; index < pPointsNum; index++) {
    //                     pcl::PointXYZ tmpPoint;
    //                     tmpPoint = cloud_p->points[index];
    //                     // std::cerr << "    " << tmpPoint.x << " "
    //                     // << tmpPoint.y << " "
    //                     // << tmpPoint.z << std::endl;
    //                     // if (index > 10) {
    //                     //     break;
    //                     // }
    //                     // if (maxX < tmpPoint.x) maxX = tmpPoint.x;
    //                     // if (minX > tmpPoint.x) minX = tmpPoint.x;
    //                     // if (maxY < tmpPoint.y) maxY = tmpPoint.y;
    //                     // if (minY > tmpPoint.y) minY = tmpPoint.y;
    //                     // if (maxZ < tmpPoint.z) maxZ = tmpPoint.z;
    //                     // if (minZ > tmpPoint.z) minZ = tmpPoint.z;
    //                     // meanX += tmpPoint.x;
    //                     // meanY += tmpPoint.y;
    //                     // meanZ += tmpPoint.z;
    //                 }
    //                 // meanX /= pPointsNum;
    //                 // meanY /= pPointsNum;
    //                 // meanZ /= pPointsNum;
    //                 // std::cerr << "    minX: " << minX << "    maxX: " << maxX << "    meanX: " << meanX << std::endl;
    //                 // std::cerr << "    minY: " << minY << "    maxY: " << maxY << "    meanY: " << meanY << std::endl;
    //                 // std::cerr << "    minZ: " << minZ << "    maxZ: " << maxZ << "    meanZ: " << meanZ << std::endl;

    //                 // std::cerr << "    TABLE TABLE TABLE TABLE!!!!" << " i: "<< i << " costheta: " << costheta << std::endl;
    //                 pcl::PCDWriter writer;
    //                 writer.write<pcl::PointXYZ> ("xxxxxx/pcd/T1_table.pcd", *cloud_p);

    //                 return;
    //             }
    //         }



    //         double t = tmr.elapsed();
    //         std::cout << "TIME: process " << t << std::endl;
    //         printf("### Computation Done. ###\n");
    //         printf("============================================\n");


    //         if (0) {
    //                 //std::string outPath = "MingPt";
    //                 //outPath.replace (outPath.length()-4, 4, "_out.vtk");
    //                 //    pcl::PointCloud<PointOut>::ConstPtr ptr2 = out;
    //                 //    pcl::io::savePolygonFileVTK (outPath.c_str (), *pcl::surface::getMesh<PointIn, PointOut>(out) );
    //                 char f[50];
    //                 sprintf (f, "out_%d.pcd", frame_id);
    //                 //    pcl::io::savePCDFileBinary (outPath.c_str (), *out);
    //                // pcl::io::saveMoPcd (f, *out);
    //         }

    //         frame_id ++;
    // }

    void run () {
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

    // void oneFrameTest() {
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //     pcl::PCDReader reader;
    //     reader.read<pcl::PointXYZ> ("xxxxxxs/pcd/T1.pcd", *cloud);
    //     this->oneFrame_cb_(cloud);
    // }

    pcl::visualization::CloudViewer viewer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, inCloud_;
    Timer tmr;
};




int main (int argc, char **argv)
{
        SimpleOpenNIViewer v;
        // v.oneFrameTest();
        v.run ();
        return 0;
}
