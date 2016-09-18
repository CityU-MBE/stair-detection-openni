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
       PLANE,
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
            for (size_t i = 0; i<list.size(); i++)
            {
                Object obj = detect_object_core( list[i], s.cloudXYZ_front ); 
                if (obj != NONE_OBJ)
                    return obj;
                else
                    continue;
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
                case PLANE:
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
            //TODO: change to real code
            cout << "try detect => TABLE_OBJ" << endl;
            s.my_pause();
            cout << "detected" << endl;
            return TABLE_OBJ;
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
