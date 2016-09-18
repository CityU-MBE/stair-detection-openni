#ifndef _SER_CMD_H__
#define _SER_CMD_H__

#include<stdio.h>  
#include<stdlib.h>  
#include<unistd.h>  
#include<sys/types.h>  
#include<sys/stat.h>  
#include<fcntl.h>  
#include<termios.h>  
#include<errno.h>  

#define FALSE -1  
#define TRUE 0  
//define command codes
typedef enum {RDU, RDD, REA, REM, RLO, RLF, G1, G2, G3, G4, GX} CMD;

int serial_cmd_init(const char *port);
void serial_cmd_deinit();

void seat_up();
void seat_down();
void autobalance_enable();
void autobalance_disable();
void light_on();
void light_off();
void go2pos1();
void go2pos2();
void go2pos3();
void go2pos4();
void go2posX();

#endif //_SER_CMD_H__
