/*
 * stairdetectiondemo.h
 *
 *  Created on: Jul 15, 2012
 *      Author: elmasry
 */

#ifndef STAIRDETECTIONLOCAL_H_
#define STAIRDETECTIONLOCAL_H_

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include "pcl/utils/pointcloud_utils.h"
#include "pcl/types/plane3d.h"
#include "pcl/segmentation/planesegmentation.h"
#include "pcl/common/color.h"

#include "pcl/common/fhg_point_types.h"
#include "pcl/common/edges_common.h"
#include "pcl/opencv/cvutils.h"
#include "pcl/opencv/edgesdetector.h"

#include "pcl/utils/pointcloud_utils.h"
#include "pcl/models/localmodel.h"
#include "pcl/models/localmodelfactory.h"
#include "pcl/models/edges2planesfactory.h"
#include "pcl/models/model_utils.h"
#include "pcl/models/globalmodel.h"
#include "pcl/io/globfitwriter.h"
#include <vector>

namespace pcl
{
    template<typename PointIn, typename PointOut>
        class StairDetectionLocal
        {
            typedef Eigen::aligned_allocator<PointIn> Alloc;
            typedef Eigen::aligned_allocator<PointOut> AllocOut;
            typedef std::vector<pcl::Plane3D<PointOut>, AllocOut> Plane3DVector;
            typedef std::vector<pcl::LineSegment3D<PointOut>, Eigen::aligned_allocator<PointOut> > LineSegment3DVector;
            typedef std::vector<Step<PointOut>, Eigen::aligned_allocator<PointOut> > StairSteps;
            typename pcl::PointCloud<PointIn>::ConstPtr inCloud;
            pcl::PlaneSegmentation<PointIn, PointOut> p;
            float cameraAngle, cameraHeight;
            int numIterations;
            LocalModel<PointOut> model;
            Eigen::Vector3f  RiserNormal;
            YAML::Node yamlNode;
            std::string yamlName;
            public:
            pcl::GlobalModel<PointOut> globalModel;
            std::string cloudName;

            float checkHeight;
            float precision ;
            float lengthThresholdShort;
            float lengthThresholdLong;

            int maxNumberSteps;
            int minNumberSteps;

            //If there is a StairSteps
            bool stairdetection()
            {
                int stair_count = stepsNumberDetection();

                if (stair_count < minNumberSteps || stair_count > maxNumberSteps) {
                    std::cout<<"Number of steps is unsatisfied: " << stair_count <<std::endl;
                    return false;
                }else if (stepsParametersDetection()){
                    std::cout<<"Number of steps is correct: " << stair_count <<std::endl;
                    return true;
                }else {
                    std::cout<<"Heigh of Riser is unsatisfied: " << stair_count <<std::endl;
                    return false;
                }
            }

            Eigen::Vector3f getRiserNormal(){
                return RiserNormal;
            }

            // Detect the number of steps
            inline int stepsNumberDetection ()
            {
                StairSteps steps = model.getSteps ();
                return steps.size();
            }

            // Calculate the tread and rise information
            bool  stepsParametersDetection()
            { 
                StairSteps steps = model.getSteps ();
                int count = 0;
                for (size_t i = 0; i < steps.size (); i++)
                {
                    Step<PointOut>& step = steps[i];
                    if (step.hasTread ())
                    {
                        const Tread<PointOut>& tread = step.getTread ();
                        if ((tread.getLDEpth() < checkHeight + precision)  && (tread.getLDEpth() > checkHeight - precision) && 
                                (tread.getLength() > lengthThresholdShort) && (tread.getLength() < lengthThresholdLong)){
                            if ((tread.getRDepth() < checkHeight + precision)  && (tread.getRDepth() > checkHeight - precision)){
                                std::cout<<"##### Tread Rheight is : "<<tread.getRDepth()
                                    <<" Tread Lheight is: "<< tread.getLDEpth() 
                                    <<" Tread length is: "<< tread.getLength() 
                                    <<" #####"<< std::endl;
                                RiserNormal = tread.getNormal();
                                return true;
                            } 
                        }
                    }
                    if (step.hasRiser ())
                    {
                        const Riser<PointOut>& riser = step.getRiser();
                        if ((riser.getHeight() < checkHeight + precision)  && (riser.getHeight() > checkHeight - precision) && 
                                (riser.getLength() > lengthThresholdShort) && (riser.getLength() < lengthThresholdLong)) 
                        {
                            std::cout<<"##### Riser height is : "<< riser.getHeight() 
                                <<"Riser length is: "<< riser.getLength() 
                                <<" #####" << std::endl;
                            RiserNormal = riser.getNormal();
                            return true;
                        }
                    } 
                }  
                return false;
            }


            protected:
            void logModel (LocalModel<PointOut> model_)
            {
                StairSteps steps = model_.getSteps ();
                int count = 0;
                size_t maxNumberSteps = 3;
                if (steps.size () >= maxNumberSteps)
                {
                    printf ("Bigger than max %d : %d\n", (int) maxNumberSteps, steps.size());
                }
                else
                {
                    printf ("Smaller than max: %d\n", (int) steps.size ());
                }

                for (size_t i = 0; i < steps.size (); i++)
                {
                    if (++count > 3)
                        break;
                    Step<PointOut>& step = steps[i];
                    if (step.hasTread ())
                    {
                        const Tread<PointOut>& tread = step.getTread ();
                        printf ("LocalTread l: %f ld: %f rd: %f---\n", tread.getLength (), tread.getLDEpth (), tread.getRDepth ());
                    }
                    if (step.hasRiser ())
                    {
                        const Riser<PointOut>& riser = step.getRiser ();
                        printf ("LocalRiser l: %f h: %f ---\n", riser.getLength (), riser.getHeight ());
                    }
                    printf ("\n");
                }
            }

            void calculateCameraHeight (const LocalModel<PointOut>& model_)
            {
                if (model_.getSteps ().size () > 0)
                {
                    const Step<PointOut>& step = model_.getSteps ()[0];
                    if (model_.isLookingUpstairs ())
                    {     
                        printf("Lookup!\n");

                        if (step.hasRiser ())
                        {
                            printf("Riser!\n");
                            cameraHeight = step.getRiser ().getHesseDistance ();
                        }
                        else
                        {
                            printf("NoRiser!\n");
                            cameraHeight = step.getTread ().getHesseDistance ();
                        }
                    }
                    else if (model_.isLookingDownstairs ())
                    {
                        printf("LookDown!\n");
                        if (step.hasTread ())
                        {
                            printf("Tread!\n");
                            cameraHeight = step.getTread ().getHesseDistance ();
                        }
                        else
                        {
                            printf("NoTread!\n");
                            cameraHeight = step.getRiser ().getHesseDistance ();
                        }
                    }
                }
            }

            public:

            StairDetectionLocal(std::string yamlname_): yamlName(yamlname_) {
                numIterations = 0;
                cameraHeight = 0.0f;
                cameraAngle = 0.0f;
                yamlNode = YAML::LoadFile(yamlName);
                checkHeight = yamlNode["RiserHeight"].as<float>();
                precision = yamlNode["RiserPrecision"].as<float>();
                lengthThresholdShort = yamlNode["RiserLengthShort"].as<float>();
                lengthThresholdLong = yamlNode["RiserLengthLong"].as<float>();
                maxNumberSteps = yamlNode["maxNumberSteps"].as<int>();
                minNumberSteps = yamlNode["minNumberSteps"].as<int>();
            }

            StairDetectionLocal(float rHeight,
                    float rHeightPrecision,
                    float rLengthShort,
                    float rLengthLong,
                    int maxSteps,
                    int minSteps):
                checkHeight(rHeight),
                precision(rHeightPrecision),
                lengthThresholdShort(rLengthShort),
                lengthThresholdLong(rLengthLong),
                maxNumberSteps(maxSteps),
                minNumberSteps(minSteps){
                numIterations = 0;
                cameraHeight = 0.0f;
                cameraAngle = 0.0f;
            }

            void setInputCloud (const typename pcl::PointCloud<PointIn>::ConstPtr inCloud)
            {
                this->inCloud = inCloud;
            }

            /**
            */
            //typename pcl::PointCloud<PointIn>::Ptr compute ()
            int compute ()
            {
                typename pcl::PointCloud<PointIn>::Ptr cloud (new pcl::PointCloud<PointIn> ());
                pcl::copyPointCloud (*inCloud, *cloud);
                typename pcl::PointCloud<PointIn>::Ptr grey_input_cloud (new pcl::PointCloud<PointIn> ());
                pcl::copyPointCloud (*inCloud, *grey_input_cloud);
                pcl::io::savePCDFileASCII ("incloud_gray.pcd", *grey_input_cloud);

                cloud->width = inCloud->width;
                cloud->height = inCloud->height;
                //Quaternion<float> rotQuaternion;
                pcl::cameraToworld (*cloud);

                typename pcl::PointCloud<PointIn>::Ptr outCloud (new pcl::PointCloud<PointIn> ());

                //use Dirk's Region Growth

                p.setInputCloud (cloud);
                p.compute ();
                //cameraAngle = p.getRotationangle ();
                //rotQuaternion = p.rotQuaternion;
                Plane3DVector planes = p.getPlanes ();

                //        {
                //          //  DEBUG /////
                //          typename pcl::PointCloud<PointOut>::Ptr rotatedCloud (new pcl::PointCloud<PointOut>);
                //          pcl::copyPointCloud (*cloud, *rotatedCloud);
                //          pcl::rotatePointCloud (*rotatedCloud, rotQuaternion);
                //          pcl::io::savePCDFileASCII ("rotated_cloud.pcd", *rotatedCloud);
                //          float black = pcl::generateColor (0, 0, 0);
                //          pcl::colorCloud (*rotatedCloud, black);
                //          pcl::io::savePCDFileASCII ("rotated_cloud_black.pcd", *rotatedCloud);
                //        }

                LocalModelFactory<PointOut> fac;
                fac.addPlanes (planes);
                model = fac.createLocalModel (true);

                {
                    typename pcl::PointCloud<PointOut>::Ptr bordersCloud = model.getBoundingBoxesCloud();
                    pcl::colorCloud(*bordersCloud, pcl::generateColor(0,0,0));
                    char f[50];
                    sprintf(f, "local_borders_%d.pcd", numIterations);
                    pcl::io::saveMoPcd(f, *bordersCloud);
                    //printf("localmodel after creation and add missing planes\n");
                    //model.logSteps();
                }

                pcl::copyPointCloud (*cloud, *outCloud); //Ming: take input as output
                //printf("localmodel before edge detection:\n");
                //model.logSteps();

                //calculateCameraHeight (model);
                //printf("#Model: %d\n", model.getSteps().size() );
                //printf("Camera Height: %.4f\n", cameraHeight);
                //printf("Camera Angle: %.4f\n", cameraAngle);

                //printf("logModel: \n");
                //logModel(model);


                //printf ("----------------------------------------------------\n");

                numIterations++;
                return 0;
            }

            typename pcl::PointCloud<PointOut>::Ptr getPlanesCloud ()
            {
                return p.getOutputCloud ();
            }

            typename pcl::PointCloud<PointOut>::Ptr getRawPlanesCloud ()
            {
                return p.getRawPlanesCloud ();
            }

            GlobalModel<PointOut> getGlobalModel () const
            {
                return globalModel;
            }

            void setCloudName (std::string cloudName)
            {
                this->cloudName = cloudName;
            }

        };

}

#define PCL_INSTANTIATE_StairDetectionLocal(In,Out) template class PCL_EXPORTS pcl::StairDetectionLocal<In,Out>;
#endif /* STAIRDETECTIONLOCAL_H_ */
