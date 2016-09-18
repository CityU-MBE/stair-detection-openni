#ifndef MACHINE_H
#define MACHINE_H

#include <string>
#include <iostream>
#include <unistd.h>
#include "genericstate.h"
#include <stdio.h>//lq
using namespace std;
class Machine
{
public:
    Machine() :
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
       BUMPS
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


private:
    static void print(const std::string &str) { std::cout << str << std::endl; }
    static void unhandledEvent() { print("unhandled event"); }


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
                system("./test g");
                break;
            case STAIRS_MODE:
                system("./test i");
                break;
            case UGV_MODE:
                system("./test j");
                break;
            case TABLE_MODE:
                system("./test h");
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
                print("entering StartMode");
                Object obj;
                while(obj != TABLE_OBJ){
                        obj  = detect_object();
                        usleep(5);
                }
                changeMode(TABLE);
        }
        Object detect_object()
        { // TODO: detect for PLANES
            return TABLE_OBJ;
        }
        void changeMode(Mode mode) {
                print("change to Mode: TABLE");
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
        void exit() { print("leaving StartMode"); }
    } start_mode;

    struct TableMode : public ModeState {
        explicit TableMode(Machine &m, ModeState *&state): ModeState(m, state){}
        void entry() {
                print("entering TableMode");
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
                cout << "change to Mode: " << mode << endl;
                if (mode == WALK)
                {
                        print("OK.\n");
                        change(s.walk);
                }
                else
                        print("Not IMPLEMENTED.");
        }
        //void bringDown() { change<Low>(); }
        void exit() { print("leaving TableMode"); }
    } table;


    struct WalkMode : public ModeState {
        explicit WalkMode(Machine &m, ModeState *&state): initflag(0), ModeState(m, state){}
        void entry() {
                print("entering Walk Mode");
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
                cout << "change to Mode: " << mode << endl;
                if (mode == UGV)
                {
                        print("OK. Bumps\n");
                        change(s.ugv);
                }
                else if (mode == STAIR) // may not work due to detection
                {
                        print("OK. Go downstairs.\n");
                        change(s.stair);
                } else
                        print("Not IMPLEMENTED.");
        }
        void exit() { print("leaving Walk"); }
    } walk;


    struct UGVMode : public ModeState {
        explicit UGVMode(Machine &m, ModeState *&state): ModeState(m, state){}
        void entry() {
                print("entering UGV Mode");
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
                cout << "change to Mode: " << mode << endl;
                if (mode == STAIR)
                {
                        print("OK.\n");
                        change(s.stair);
                }
                else
                        print("Not IMPLEMENTED.");
        }
        void exit() { print("leaving UGV"); }
    } ugv;

    struct StairMode : public ModeState {
        explicit StairMode(Machine &m, ModeState *&state): ModeState(m, state){}
        void entry() {
                print("entering STAIR Mode");
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
            cout << "ALIGNED" << endl;
        }
        Object detect_object()
        { // TODO: detect for PLANES
            return GROUND_ONLY;
        }
        void changeMode(Mode mode) {
                cout << "change to Mode: " << mode << endl;
                if (mode == WALK)
                {
                        print("OK. In-place Turn\n");
                        change(s.walk);
                }
                else
                        print("Not IMPLEMENTED.");
        }
        void exit() { print("leaving Stair mode."); }
    } stair;
};

#endif // MACHINE_H
