#include "serial_cmd.h"
//const definition
const static char *cmdlist[] = {"RDU","RDD","REA","REM","RLO","RLF","G1","G2","G3","G4","GX"};
const static int speed_arr[] = {B38400, B19200, B9600, B4800, B2400, B1200, B300,B38400, B19200, B9600, B4800, B2400, B1200, B300};  
const static int name_arr[] = {38400,  19200,  9600,  4800,  2400,  1200,  300, 38400, 19200,  9600, 4800, 2400, 1200,  300};  
//global vars
static int g_fd=0;	//pointer to serial port (ttyUSB0 here)
static char wrBuf[20];	//serial port write buffer
//static functions declaration
static void set_speed(int fd, int speed);
static int set_Parity(int fd,int databits,int stopbits,int parity);
static void send_cmd(int fd, CMD c);
//open serial port & set params
//return TRUE if succeeded, else FALSE
int serial_cmd_init(const char *port)
{
	g_fd = open(port,O_RDWR);  
	if (-1 == g_fd)
		return FALSE;

	set_speed(g_fd,9600); 

	if (set_Parity(g_fd,8,1,'N') == FALSE)  
		return FALSE;
	else 
		return TRUE;  
}
// close serial port when exit
void serial_cmd_deinit()
{
	close(g_fd);  
}
//set serial baudrate
static void set_speed(int fd, int speed){  
    int   i;   
    int   status;   
    struct termios   Opt;  
    tcgetattr(fd, &Opt);   
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) {   
        if  (speed == name_arr[i]) {       
            tcflush(fd, TCIOFLUSH);       
            cfsetispeed(&Opt, speed_arr[i]);    
            cfsetospeed(&Opt, speed_arr[i]);     
            status = tcsetattr(fd, TCSANOW, &Opt);    
            if  (status != 0) {      
                perror("tcsetattr fd1");    
                return;       
            }      
            tcflush(fd,TCIOFLUSH);     
        }    
    }  
}
//set serial data format
static int set_Parity(int fd,int databits,int stopbits,int parity)  
{   
    struct termios options;   
    if  ( tcgetattr( fd,&options)  !=  0) {   
        perror("SetupSerial 1");       
        return(FALSE);    
    }  
    options.c_cflag &= ~CSIZE;   
    switch (databits)   
    {     
        case 7:       
            options.c_cflag |= CS7;   
            break;  
        case 8:       
            options.c_cflag |= CS8;  
            break;     
        default:      
            fprintf(stderr,"Unsupported data size\n"); return (FALSE);    
    }  
    switch (parity)   
    {     
        case 'n':  
        case 'N':      
            options.c_cflag &= ~PARENB;   /* Clear parity enable */  
            options.c_iflag &= ~INPCK;     /* Enable parity checking */   
            break;    
        case 'o':     
        case 'O':       
            options.c_cflag |= (PARODD | PARENB);   
            options.c_iflag |= INPCK;         /* Disnable parity checking */   
            break;    
        case 'e':    
        case 'E':     
            options.c_cflag |= PARENB;     /* Enable parity */      
            options.c_cflag &= ~PARODD;      
            options.c_iflag |= INPCK;       /* Disnable parity checking */  
            break;  
        case 'S':   
        case 's':  /*as no parity*/     
            options.c_cflag &= ~PARENB;  
            options.c_cflag &= ~CSTOPB;break;    
        default:     
            fprintf(stderr,"Unsupported parity\n");      
            return (FALSE);    
    }    

    switch (stopbits)  
    {     
        case 1:      
            options.c_cflag &= ~CSTOPB;    
            break;    
        case 2:      
            options.c_cflag |= CSTOPB;    
            break;  
        default:      
            fprintf(stderr,"Unsupported stop bits\n");    
            return (FALSE);   
    }   
    /* Set input parity option */   
    if (parity != 'n')     
        options.c_iflag |= INPCK;   
    tcflush(fd,TCIFLUSH);  
    options.c_cc[VTIME] = 150;   
    options.c_cc[VMIN] = 0; /* Update the options and do it NOW */  
    if (tcsetattr(fd,TCSANOW,&options) != 0)     
    {   
        perror("SetupSerial 3");     
        return (FALSE);    
    }   
    return (TRUE);    
}  
//send command to wheelchair
static void send_cmd(int fd, CMD c)
{
    const char *p=cmdlist[c];
    int i=0;
    wrBuf[i++]='@';
    while(*p)
    {
        wrBuf[i]=*p;
        i++;
        p++;
    }
    wrBuf[i++]='#';
    wrBuf[i++]=0x0d;
    wrBuf[i++]=0x0a;

    write(fd, wrBuf, i);
	printf("serial command: %s\n", wrBuf);//echo command sent  
}

void seat_up()
{
    send_cmd(g_fd, RDU);
}

void seat_down()
{
    send_cmd(g_fd, RDD);
}

void autobalance_enable()
{
    send_cmd(g_fd, REA);
}

void autobalance_disable()
{
    send_cmd(g_fd, REM);
}

void light_on()
{
    send_cmd(g_fd, RLO);
}

void light_off()
{
    send_cmd(g_fd, RLF);
}

void go2pos1()
{
    send_cmd(g_fd, G1);
}

void go2pos2()
{
    send_cmd(g_fd, G2);
}

void go2pos3()
{
    send_cmd(g_fd, G3);
}

void go2pos4()
{
    send_cmd(g_fd, G4);
}

void go2posX()
{
    send_cmd(g_fd, GX);
}
