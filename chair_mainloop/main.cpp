#include "machine.h"

#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
// add io
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/filters/voxel_grid.h>
class SimpleOpenNIViewer
{
        public:
                SimpleOpenNIViewer (Machine * m) : viewer ("PCL OpenNI Viewer"),frame_id(0),inCloud(new pcl::PointCloud<pcl::PointXYZRGBA>),inCloud_(new pcl::PointCloud<pcl::PointXYZRGBA>), m(m)
                {
                    //m.start(); // blcoked
                    //m_thread = boost::thread(&Machine::start, m);
                }
                int frame_id;
                boost::thread m_thread;

                void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_)
                {
                        //     tmr.reset();

                        if (!viewer.wasStopped())
                                viewer.showCloud (cloud_);

                        // machine set new point-cloud
                        m->setCloudRGBA(*cloud_);

                        pcl::copyPointCloud(*cloud_, *inCloud_);
               //         printf ("processing frame: %d\n", frame_id++);

#if 1
                        // Create the filtering object
                        pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
                        sor.setInputCloud (inCloud_);
                        sor.setLeafSize (0.01, 0.01, 0.01);
                        sor.filter (*inCloud);

              //          std::cout << "Before: " << inCloud_->width * inCloud_->height << " After: " << inCloud->width * inCloud->height << "\n";

#endif
                }

                void run ()
                {
                        pcl::Grabber* interface = new pcl::OpenNIGrabber("#1");

                        boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
                                boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

                        interface->registerCallback (f);

                        interface->start ();

                       // viewer.setCameraPosition(0,0,-2,0,-1,0,0);

                       // viewer.setBackgroundColor (1.0, 0.5, 1.0);



                        while (!viewer.wasStopped())
                        {
                                boost::this_thread::sleep (boost::posix_time::seconds (1));
                        }

                        interface->stop ();
                }

                pcl::visualization::CloudViewer viewer;
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inCloud, inCloud_;
                Machine * m;
};

int main()
{
        Machine m;
        boost::thread* thr = new boost::thread(boost::bind(&Machine::start, &m));
        SimpleOpenNIViewer v(&m);
        v.run ();

        return 0;
}
