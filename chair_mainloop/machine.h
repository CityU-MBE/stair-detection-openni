#ifndef MACHINE_H
#define MACHINE_H

#include <string>
#include <iostream>
#include <unistd.h>
#include "genericstate.h"

//#include "pcl/apps/stairdetectionlocal.h"


#include <stdio.h>//lq

// pcl:
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

// for table(plane) extraction -- YHY:
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/transforms.h>

using namespace std;
class Machine
{
public:
    Machine() :
        cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>),
        cloudXYZ_front(new pcl::PointCloud<pcl::PointXYZ>),
        cloudXYZ_back(new pcl::PointCloud<pcl::PointXYZ>),
        cloudXYZRGBA(new pcl::PointCloud<pcl::PointXYZRGBA>),
        table(*this, modeState),
        walk(*this, modeState),
        ugv(*this, modeState),
        stair(*this, modeState),
        start_mode(*this, modeState){}
    ~Machine() {}
    void start();

public:
   enum Object { //detectable objects
       TABLE_OBJ,
       PLANE, // !!! Object PLANE is not used in this file
       PLANE_END,
       STAIRS,
       GROUND_ONLY,
       BUMPS,
       NONE_OBJ
   };

   enum Transform { // serial port
       WALK_MODE,
       STAIRS_MODE,
       UGV_MODE,
       TABLE_MODE
   };


   enum Mode{ // finite state mode
       TABLE,
       WALK,
       UGV,
       STAIR // START is omitted
   };

   enum Cameras{
        FCAMERA,
        BCAMERA
   };
public:
   void changeMode(Mode mode) { modeState->changeMode(mode); }
    void setCloudRGBA(const pcl::PointCloud<pcl::PointXYZRGBA> &);


private:
    static void print(const std::string &str) { std::cout << str << std::endl; }
    static void unhandledEvent() { print("unhandled event"); }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ_front;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ_back;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBA;

    void convertToXYZ(const pcl::PointCloud<pcl::PointXYZRGBA> &, pcl::PointCloud<pcl::PointXYZ> &);

    void my_pause()
    {
        int x;
        cin >> x;
        cin.ignore();
    }
private:
    class ModeState : public GenericState<Machine, ModeState> {
    protected:
        explicit ModeState(Machine &m, ModeState *&state) :
            GenericState<Machine, ModeState>(m, state) {}
    public:

        void perform_transform(Transform trans)
        {
            cout << "Transform to : " << trans << " mode" << endl;
            // use Liang Qing's API
            switch(trans)
            {
            case WALK_MODE:
                system("./sendcmd g");
                break;
            case STAIRS_MODE:
                system("./sendcmd i");
                break;
            case UGV_MODE:
                system("./sendcmd j");
                break;
            case TABLE_MODE:
                system("./sendcmd h");
                break;
            default:
                break;
            }
            sleep(10);
        }

        Object detect_object_new(const std::vector<Object> & list, const Cameras & cam, const string logText)
        {
            cout << logText << endl;
            if (FCAMERA == cam)
            {
                return detect_object_new_front(list);
            }
            else if (BCAMERA == cam)
            {
                return detect_object_new_back(list);
            }
        }

        Object detect_object_new_front(const std::vector<Object> & list)
        {
            std::cerr << "List size: " << list.size() << std::endl;
            Object obj = NONE_OBJ;

            /** !!!IMPORTANT, FOLLOWING WHILE LOOP NEEDS TO BE CHECKED:
            *@ Change the structure for loop, otherwise it will not work
            *@ YHY
            */
            while (obj == NONE_OBJ){
                for (size_t i = 0; i < list.size(); i++)
                {
                    obj = detect_object_core( list[i], s.cloudXYZ_front );
                    if (obj != NONE_OBJ) {
                        return obj;
                    }
                    else
                        continue;
                }
            }
        }
        Object detect_object_new_back(const std::vector<Object> & list)
        {
            for (size_t i = 0; i<list.size(); i++)
            {
                Object obj = detect_object_core( list[i], s.cloudXYZ_back );
                if (obj != NONE_OBJ)
                    return obj;
                else
                    continue;
            }
        }
        Object detect_object_core(const Object & obj, const pcl::PointCloud<pcl::PointXYZ>::Ptr & ptc)
        {
            Object res;
            switch( obj )
            {
                case TABLE_OBJ:
                // case PLANE: !!! PLANE is not detected in the TABLE_OBJ now -- YHY
                    res = YHY_code_TABLE(ptc);
                    break;
                case PLANE_END:
                    res = YHY_code_PLANE_END(ptc);
                    break;
                case STAIRS:
                    res = TAILEI_code_STAIRS(ptc);
                    break;
                case GROUND_ONLY:
                    res = MING_code_GROUND_ONLY(ptc);
                    break;
                case BUMPS:
                    res = LQ_code_BUMPS(ptc);
                    break;
                case NONE_OBJ:
                    res = NONE_OBJ;
                    break;
                default:
                    res = NONE_OBJ;
                    break;
            }
            return res;
        }
        // individual detectors:
        Object YHY_code_TABLE(const pcl::PointCloud<pcl::PointXYZ>::Ptr & ptc)
        {

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>),
                                                inCloud (new pcl::PointCloud<pcl::PointXYZ>);

            pcl::copyPointCloud(*ptc, *inCloud);

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

            /** ------ Create the filtering object - start ------
            * ref: http://pointclouds.org/documentation/tutorials/extract_indices.php
            */
            int i = 0, nr_points = (int) inCloud->points.size ();
            pcl::ExtractIndices<pcl::PointXYZ> extract;

            std::cerr << "YHY_code_TABLE start!!!!" << std::endl;

            // While 30% of the original cloud is still there
            while (inCloud->points.size () > 0.3 * nr_points) {
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

                if ( abs(costheta) > 0.8 ) {
                    std::cerr << "\033[1;32m DETECT PLANE !! \033[0m\n" << std::endl;
                    if (!(coefficients->values[3] < -0.5 && coefficients->values[3] > -0.8)) {
                        continue;
                    }

                    /** ------ Create the filtering object - end------ */

                    int pPointsNum = (int) cloud_p->points.size (); // number of filtered points -- the plane


                    std::cerr << "    TABLE TABLE TABLE TABLE!!!!" << " i: "<< i << " costheta: " << costheta << std::endl;
                    std::cerr << "\033[1;31m    TABLE TABLE TABLE TABLE!!!! height of table is \033[0m" <<  - coefficients->values[3] << "m." << std::endl;

                    cerr << "try detect => NEXT_OBJ" << endl;
                    // s.my_pause();
                    cerr << "TABLE_OBJ detected" << endl;
                    return TABLE_OBJ;

                    // break;
                }
            }

            // printf("### Computation Done. ###\n");
            // printf("============================================\n");
            /* ------- table_detection end -------*/

            cerr << "try detect => TABLE_OBJ" << endl;
            // s.my_pause();
            cerr << "not detected" << endl;
            return NONE_OBJ;
        }

        Object YHY_code_PLANE_END(const pcl::PointCloud<pcl::PointXYZ>::Ptr & ptc)
        {
            //TODO: change to real code
            cout << "try detect => PLANE_END" << endl;
            s.my_pause();
            cout << "detected" << endl;
            return PLANE_END;
        }

        Object TAILEI_code_STAIRS(const pcl::PointCloud<pcl::PointXYZ>::Ptr & ptc)
        {
            //TODO: change to real code
            cout << "try detect => STAIRS" << endl;
            s.my_pause();
            cout << "detected" << endl;
            return STAIRS;

        }

        Object MING_code_GROUND_ONLY(const pcl::PointCloud<pcl::PointXYZ>::Ptr & ptc)
        {
            //TODO: change to real code
            cout << "try detect => GROUND_ONLY" << endl;
            s.my_pause();
            cout << "detected" << endl;
            return GROUND_ONLY;
        }

        Object LQ_code_BUMPS(const pcl::PointCloud<pcl::PointXYZ>::Ptr & ptc)
        {
            //TODO: change to real code
            cout << "try detect => BUMPS" << endl;
            s.my_pause();
            cout << "detected" << endl;
            return BUMPS;
        }

        virtual void changeMode(Mode mode) { unhandledEvent(); }
    } * modeState;

    struct StartMode : public ModeState {
        explicit StartMode(Machine &m, ModeState *&state): ModeState(m, state){}
        void entry() {
                print("[START MODE] entering StartMode");
                Object obj;
                while(obj != TABLE_OBJ){
                        std::vector<Object> which_objects;
                        which_objects.push_back(TABLE_OBJ);
                        obj  = detect_object_new(which_objects, FCAMERA, "[START MODE]");
                        usleep(300000);
                }
                changeMode(TABLE);
        }
        void changeMode(Mode mode) {
                print("[START MODE] change to Mode: TABLE");
                if (mode == TABLE)
                {
                        print("OK.\n");
                        // TODO:
                        perform_transform(TABLE_MODE);
                        change(s.table);
                }
                else
                        print("Not IMPLEMENTED.");
        }

        //void bringDown() { change<Low>(); }
        void exit() { print("[START MODE] leaving StartMode"); }
    } start_mode;

    struct TableMode : public ModeState {
        explicit TableMode(Machine &m, ModeState *&state): ModeState(m, state){}
        void entry() {
                print("[TABLE MODE] entering TableMode");
                Object obj;
                while(obj != GROUND_ONLY){
                        std::vector<Object> which_objects;
                        which_objects.push_back(GROUND_ONLY);
                        obj  = detect_object_new(which_objects, FCAMERA, "[TABLE MODE]");
                }
                perform_transform(WALK_MODE);
                changeMode(WALK);
        }
        void changeMode(Mode mode) {
                cout << "[TABLE MODE] change to Mode: " << mode << endl;
                if (mode == WALK)
                {
                        print("OK.\n");
                        change(s.walk);
                }
                else
                        print("Not IMPLEMENTED.");
        }
        //void bringDown() { change<Low>(); }
        void exit() { print("[TABLE MODE] leaving TableMode"); }
    } table;


    struct WalkMode : public ModeState {
        explicit WalkMode(Machine &m, ModeState *&state): initflag(0), ModeState(m, state){}
        void entry() {
                print("[WALK MODE] entering Walk Mode");
                Object obj;
                while(obj != BUMPS && obj != STAIRS && obj != GROUND_ONLY){
                        std::vector<Object> which_objects;
                        which_objects.push_back(STAIRS);
                        which_objects.push_back(BUMPS);
                        obj  = detect_object_new(which_objects, FCAMERA, "[WALK MODE]");
                }

                if (obj == BUMPS){
                        perform_transform(UGV_MODE);
                        changeMode(UGV);
                } else if(obj == STAIRS){
                        perform_transform(STAIRS_MODE);
                        changeMode(STAIR);
                } else {
                    perform_transform(WALK_MODE);
                    changeMode(WALK);
                }
        }
        int initflag;
        Object detect_object()
        { // TODO: detect for PLANES
            s.my_pause();
                if (0 == initflag){
                        initflag ++;
                        return BUMPS;
                } else if(1 == initflag)
                {
                        initflag ++;
                        return STAIRS;
                }
                else {return GROUND_ONLY;}
        }
        void changeMode(Mode mode) {
                cout << "[WALK MODE] change to Mode: " << mode << endl;
                if (mode == UGV)
                {
                        print("[WALK MODE] OK. Bumps\n");
                        change(s.ugv);
                }
                else if (mode == STAIR) // may not work due to detection
                {
                        print("[WALK MODE] OK. Go downstairs.\n");
                        change(s.stair);
                } else
                        print("Not IMPLEMENTED.");
        }
        void exit() { print("[WALK MODE] leaving Walk"); }
    } walk;


    struct UGVMode : public ModeState {
        explicit UGVMode(Machine &m, ModeState *&state): ModeState(m, state){}
        void entry() {
                print("[UGV MODE] entering UGV Mode");
                Object obj;
                while(obj != STAIRS){
                        std::vector<Object> which_objects;
                        which_objects.push_back(STAIRS);
                        obj  = detect_object_new(which_objects, FCAMERA, "[UGV MODE]");
                }
                perform_transform(STAIRS_MODE);
                changeMode(STAIR);
        }
        void changeMode(Mode mode) {
                cout << "[UGV MODE] change to Mode: " << mode << endl;
                if (mode == STAIR)
                {
                        print("[UGV MODE] OK.\n");
                        change(s.stair);
                }
                else
                        print("Not IMPLEMENTED.");
        }
        void exit() { print("[UGV MODE] leaving UGV"); }
    } ugv;

    struct StairMode : public ModeState {
        explicit StairMode(Machine &m, ModeState *&state): ModeState(m, state){}
        void entry() {
                print("[STAIR MODE] entering STAIR Mode");
                wait_for_alignment();
                Object obj;
                while(obj != GROUND_ONLY){
                        std::vector<Object> which_objects;
                        which_objects.push_back(GROUND_ONLY);
                        obj  = detect_object_new(which_objects, BCAMERA, "[STAIR MODE]");
                }
                perform_transform(WALK_MODE); // make in-place turning; and final
                changeMode(WALK);
        }
        void wait_for_alignment()
        { // TODO: implement alignment checking
            usleep(500000); // change to alignment checking function
            cout << "[STAIR MODE] ALIGNED" << endl;
        }
        void changeMode(Mode mode) {
                cout << "[STAIR MODE] change to Mode: " << mode << endl;
                if (mode == WALK)
                {
                        print("[STAIR MODE] OK. In-place Turn\n");
                        change(s.walk);
                }
                else
                        print("Not IMPLEMENTED.");
        }
        void exit() { print("[STAIR MODE] leaving Stair mode."); }
    } stair;

};

#endif // MACHINE_H
