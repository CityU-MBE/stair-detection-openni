#include "serial_cmd.h"

int main(int argc, char **argv)  
{  
    //printf("Serial Test\n");  
    if(FALSE == serial_cmd_init("/dev/ttyUSB0"))  
    {  
        perror("serialport error\n"); 
    }  
    else  
    {  
    //    printf("serialport opened succesfully\n");
    } 

    if (argc>1)
    {
        switch(*argv[1])
        {
            case 'a':
                seat_up();
                break;
            case 'b':
                seat_down();
                break;
            case 'c':
                autobalance_enable();
                break;
            case 'd':
                autobalance_disable();
                break;
            case 'e':
                light_on();
                break;
            case 'f':
                light_off();
                break;
            case 'g':
                go2pos1();
                break;
            case 'h':
                go2pos2();
                break;
            case 'i':
                go2pos3();
                break;
            case 'j':
                go2pos4();
                break;
            case 'k':
                go2posX();
                break;
            default: break;

        }
    }

    serial_cmd_deinit();  
    return 0;  
}
