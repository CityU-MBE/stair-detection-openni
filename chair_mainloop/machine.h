#ifndef MACHINE_H
#define MACHINE_H

#include <string>
#include <iostream>
#include <unistd.h>
#include "genericstate.h"
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
public:
   void changeMode(Mode mode) { modeState->changeMode(mode); }
    void setCloudRGBA(const pcl::PointCloud<pcl::PointXYZRGBA> &);


private:
    static void print(const std::string &str) { std::cout << str << std::endl; }
    static void unhandledEvent() { print("unhandled event"); }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBA;

    void convertToXYZ(const pcl::PointCloud<pcl::PointXYZRGBA> &, pcl::PointCloud<pcl::PointXYZ> &);


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

        virtual void changeMode(Mode mode) { unhandledEvent(); }
    } * modeState;

    struct StartMode : public ModeState {
        explicit StartMode(Machine &m, ModeState *&state): ModeState(m, state){}
        void entry() {
                print("[START MODE] entering StartMode");
                Object obj;
                while(obj != TABLE_OBJ){
                        obj  = detect_object();
                        usleep(300000);
                }
                changeMode(TABLE);
        }
        Object detect_object()
        { // TODO: detect for PLANES
            //return TABLE_OBJ;
            cout << "[START MODE] try detect => @" << endl;
            return NONE_OBJ;
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
                        obj  = detect_object();
                }
                perform_transform(WALK_MODE);
                changeMode(WALK);
        }
        Object detect_object()
        { // TODO: detect for PLANES
            return GROUND_ONLY;
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
                        obj  = detect_object();
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
                        obj  = detect_object();
                }
                perform_transform(STAIRS_MODE);
                changeMode(STAIR);
        }
        Object detect_object()
        { // TODO: detect for STAIRS
            return STAIRS;
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
                        obj  = detect_object();
                }
                perform_transform(WALK_MODE); // make in-place turning; and final
                changeMode(WALK);
        }
        void wait_for_alignment()
        {
            usleep(500000); // change to alignment checking function
            cout << "[STAIR MODE] ALIGNED" << endl;
        }
        Object detect_object()
        { // TODO: detect for PLANES
            return GROUND_ONLY;
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
