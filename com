

#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <termios.h>
#include <unistd.h>  //write()
#include <iostream>
#include <stdio.h>  //标准输入输出定义
#include <fcntl.h>  //文件控制定义
#include<opencv2/imgproc/imgproc.hpp>
#include <string.h>
#include <errno.h>     //ERROR数字定义
#include <sys/select.h>
using namespace std;
using namespace cv;
void sleep_ms(unsigned int secs)
{
    struct timeval tval;
    tval.tv_sec = secs / 1000;
    tval.tv_usec = (secs * 1000) % 1000000;
    select(0, NULL, NULL, NULL, &tval);
}
   int fd = open("/dev/ttyUSB0", O_RDWR | O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);   //ttyUSB0打开设备文件
int main()
{
//       VideoCapture capture(1);
//       Mat src;
//    while(1)
//    {
//       capture >>src;
//       imshow(" ",src);


    struct termios options, newstate;

    if(-1==fd)
    printf("can not open the COM!\n");
    else
    printf("open COM ok!\n");

    if (fcntl(fd, F_SETFL, 0) < 0) //改为阻塞模式
        printf("fcntl failed\n");
    else
        printf("fcntl=%d\n", fcntl(fd, F_SETFL, 0));
    tcgetattr(fd, &options);

    //设置波特率
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    //串口设置
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;//设置无奇偶校验位，N
    options.c_cflag &= ~CSTOPB; //设置停止位1
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8; //设置数据位

    //read de return
    options.c_cc[VTIME] = 0;// deng dai de shijian
    options.c_cc[VMIN] = 1;   //deng dai du qu de zui xiao zi jie shu
    //激活新配置
    tcsetattr(fd, TCSANOW, &options);

    char len;
    char buffer[512];
    unsigned char buff[1] ;
    buff[0] = '8';
       while(1){
               memset(buffer,0,strlen(buffer));  //qing nei chun jiu mei you shuchu
                write(fd,buff,sizeof(1));  //shuru & daxiao
                len=read(fd,buffer,sizeof(buffer));
                if(len<0)
                     cout <<  "error" << endl;
                else
                     cout << buffer << endl;

                }
//            int key=waitKey(1);
//            if(char(key)==27)
//                break;

//    close(fd);
//    return 0;
}
