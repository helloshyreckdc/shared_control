#include "ros/ros.h"
#include "std_msgs/Char.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>


#define BAUDRATE  57600
#define DEVICE 	  "/dev/ttyUSB0"

int ret;
int serial_fd;
char read_buf[3];
int seriallen=4;  //serial data length

std_msgs::Char msg;

int set_serial(int fd, int nSpeed, int nBits, char nEvent, int nStop);
void serial_init();
int read();

int main(int argc, char **argv)
{
        ros::init(argc, argv, "serial_receiver");
	ros::NodeHandle n;
        ros::Publisher cmd_pub = n.advertise<std_msgs::Char>("command_BCI", 1);
	ros::Rate loop_rate(10);

        serial_init();  //open and config serial port
	while(ros::ok())
	{
          ret = read();  //save serialdata to read_buff
          if(ret == -1)
          {
             ROS_INFO("Read data error!");
          }
          else
          {
             msg.data = read_buf[0];
          }

          ROS_INFO("%c", msg.data);
          cmd_pub.publish(msg);
          loop_rate.sleep();
	}
	close(serial_fd);
	return 0;
}


int set_serial(int fd,int nSpeed, int nBits, char nEvent, int nStop)  
{  
	struct termios newtio,oldtio;  
	if( tcgetattr( fd,&oldtio)  !=  0) 
	{   
		ROS_INFO("SetupSerial 1");  
		return -1;  
	}  
	bzero( &newtio, sizeof( newtio ) );  
	newtio.c_cflag  |=  CLOCAL | CREAD;  
	newtio.c_cflag &= ~CSIZE;  

	switch( nBits )  
	{  
		case 7:  
			newtio.c_cflag |= CS7;  
			break;  
		case 8:  
			newtio.c_cflag |= CS8;  
			break;  
	}

	switch( nEvent )  
	{  
		case 'O':  
			newtio.c_cflag |= PARENB;  
			newtio.c_cflag |= PARODD;  
			newtio.c_iflag |= (INPCK | ISTRIP);  
			break;  
		case 'E':   
			newtio.c_iflag |= (INPCK | ISTRIP);  
			newtio.c_cflag |= PARENB;  
			newtio.c_cflag &= ~PARODD;  
	    	break;
		case 'N':    																									
			newtio.c_cflag &= ~PARENB;  
			break;  
	}  

	switch( nSpeed )  
	{  
		case 2400:  
			cfsetispeed(&newtio, B2400);  
			cfsetospeed(&newtio, B2400);  
			break;  
		case 4800:  
			cfsetispeed(&newtio, B4800);  
			cfsetospeed(&newtio, B4800);  
			break;
		case 9600:  
			cfsetispeed(&newtio, B9600);  
			cfsetospeed(&newtio, B9600);  
			break;  
		case 19200:  
			cfsetispeed(&newtio, B19200);  
			cfsetospeed(&newtio, B19200);  
			break; 
		case 57600:  
			cfsetispeed(&newtio, B57600);  
			cfsetospeed(&newtio, B57600);  
			break; 
		case 115200:  
			cfsetispeed(&newtio, B115200);  
			cfsetospeed(&newtio, B115200);  
			break;  
		case 460800:  
			cfsetispeed(&newtio, B460800);  
			cfsetospeed(&newtio, B460800);  
			break;  
		default:  
			cfsetispeed(&newtio, B9600);  
			cfsetospeed(&newtio, B9600);  
			break;  
	}  

	if( nStop == 1 )  
	{
		newtio.c_cflag &=  ~CSTOPB;  
	}
	else if ( nStop == 2 )
	{  
		newtio.c_cflag |=  CSTOPB;  
	} 

	newtio.c_cc[VTIME]  = 0;
	newtio.c_cc[VMIN] = 0;
	tcflush(fd,TCIFLUSH);  
	if((tcsetattr(fd,TCSANOW,&newtio))!=0)  
	{  
          ROS_INFO("com set error!");
          return -1;
	}  
	return 0;  
} 

void serial_init()
{
	serial_fd = open(DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (serial_fd == -1)
	{
          ROS_INFO("Open Error!");
          exit(1);
	}  
	ret = set_serial(serial_fd, BAUDRATE, 8, 'N', 1);
	if (ret == -1)  
	{
          ROS_INFO("Set Serial Error!");
          exit(1);
	}
}

int read()
{
	int ret;

	memset(read_buf, 0, 50);
        ret = read(serial_fd, read_buf, seriallen);
	if (ret < 0)
	{
          ROS_INFO("read error!");
          return ret;
	}
	else if (ret == 0)
        {
          ROS_INFO("No data!");
          return ret;
        }
        else
        {
          return 0;
        }
}
