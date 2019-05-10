/**
 * @file serialport.cpp
 * @author GCUROBOT-Visual-Group (GCUROBOT_WOLF@163.com)
 * @brief RM 2019 步兵视觉串口部分源文件
 * @version 1.1
 * @date 2019-05-01
 * @copyright Copyright (c) 2019 GCU Robot Lab. All Rights Reserved.
 */
#include "serialport.h"

int SerialPort::fd;
char SerialPort::g_buf[BUFF_LENGTH];
char SerialPort::g_buf_temp[BUFF_TEMP_LENGTH];
char SerialPort::g_rec_buf[REC_BUFF_LENGTH];
/**
 * @brief Construct a new Serial Port:: Serial Port object
 */
SerialPort::SerialPort()
{
    cout<<"The Serial set ......"<<endl;
}

/**
 * @brief Destroy the Serial Port:: Serial Port object
 */
SerialPort::~SerialPort(void)
{
    if (!close(fd))
        printf("Close Serial Port Successful\n");
}

/**
* @brief: 初始化串口函数
* ------------------------------------------------------
* @param:  波特率,默认为115200
* --------------------------------------------------------
* @param:  char parity 是否进行奇偶校验,'Y'表示需要奇偶校验,'N'表示不需要奇偶校验
* -------------------------------------------------------------
* @param:  int databits 数据位的个数,默认值为8个数据位
*----------------------------------------------------------
* @return: bool  初始化是否成功
* @note:   在使用其他本类提供的函数前,请先调用本函数进行串口的初始化
*　　　　　   函数提供了一些常用的串口参数设置
*           本串口类析构时会自动关闭串口,无需额外执行关闭串口
* @author: Hzkkk
*          Rcxxx (revised)
*/
void SerialPort::serialSet(int port_No)
{    
    const char* DeviceName[4] = {"", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"};

     /* WARNING :  终端设备默认会设置为控制终端，因此open(O_NOCTTY不作为控制终端)
     * Terminals'll default to be set as Control Terminals
     */
    struct termios newstate;
     /*打开串口*/
    bzero(&newstate, sizeof(newstate));//清零
    fd=open(DeviceName[port_No], O_RDWR|O_NONBLOCK|O_NOCTTY|O_NDELAY);
    if (fd == -1)
    {
        perror("Can't Open Serial Port\n");
    }
    else
        printf("Open Serial Port %s Successful\n", DeviceName[port_No]);

    /*设置发送波特率*/
    cfsetospeed(&newstate, B115200);
    cfsetispeed(&newstate, B115200);

    //本地连线, 取消控制功能 | 开始接收
    newstate.c_cflag |= CLOCAL | CREAD;
    //设置字符大小
    newstate.c_cflag &= ~CSIZE;
    //设置停止位1
    newstate.c_cflag &= ~CSTOPB;
    //设置数据位8位
    newstate.c_cflag |= CS8;
    //设置无奇偶校验位，N
    newstate.c_cflag &= ~PARENB;

    /*阻塞模式的设置*/
    newstate.c_cc[VTIME]=0;
    newstate.c_cc[VMIN]=0;

    tcflush(fd,TCIOFLUSH);

    tcsetattr(fd, TCSANOW, &newstate);
}

/**
*  @brief: 串口数据读取函数
*  @return: string  返回收到的字符串
*  @note:   在使用其他本类提供的函数前,请先调用本函数进行串口的初始化
*           逐字节读取并存到字符串
*           等待0.01s后结束读取,将所得字符串返回
*  @authors: Rcxxx
*            Hzkkk
*/
void SerialPort::RMreceiveData(){
    memset(g_rec_buf, '0', REC_BUFF_LENGTH); //清空缓存
    char rec_buf_temp[8];
    read(fd,rec_buf_temp,sizeof(rec_buf_temp));
    for(int i = 0;i< (int)sizeof(rec_buf_temp);++i){
        if(rec_buf_temp[i]=='S' && rec_buf_temp[i+sizeof(g_rec_buf)-1] == 'E'){
            for(int j = 0;j<((int)sizeof(g_rec_buf));++j){
                g_rec_buf[j] = rec_buf_temp[i+j];
            }
            // g_rec_buf[0] = rec_buf_temp[i];
            // g_rec_buf[1] = rec_buf_temp[i+1];
            // g_rec_buf[2] = rec_buf_temp[i+2];
            // g_rec_buf[3] = rec_buf_temp[i+3];
            break;
        }
    }
    tcflush(fd,TCIFLUSH);
   // cout<<"  rec_buf_temp: "<<rec_buf_temp<<endl;
    //cout<<"  g_rec_buf: "<<g_rec_buf<<endl;
}

int SerialPort::RMdatabitProcessing(int bit){
    int return_value = ALL_DEFAULT;
    if(g_rec_buf[0] == 'S' && g_rec_buf[sizeof(g_rec_buf)-1] == 'E'){
        switch (bit)
        {
        case 1:{
            //处理第1位数据
            if(g_rec_buf[1] == '1'){
                return_value = support_shooting_mode;
            }else if (g_rec_buf[1]== '2'){
                return_value = energy_agency_mode;
            }else{
                return_value = mode_default;
            }
        }   break;
        case 2:{
            //处理第2位数据
            if(g_rec_buf[2] == '1'){
                return_value = little_armor;
            }else if (g_rec_buf[2]== '2'){
                return_value = big_armor;
            }else{
                return_value = armor_default;
            }
        }   break;
        case 3:{
            //处理第3位数据
            
        }   break;
        case 4:{
            //处理第4位数据
            
        }   break;
        default:{

        }   break;
        }
    }
    return return_value;
}

/**
 *@brief: RM串口发送格式化函数
 *
 * @param: x 坐标的ｘ值
 * @param: y 坐标的ｙ值
 * @param: SendDataFlag 发送的标志
 *
 * @authors: Rcxxx
 *           Hzkkk
 */
void SerialPort::RMserialWrite(int x,int y,int depth,int mode,int mode_select)
{
    mode = 1;
    mode_select = 1;
    sprintf(g_buf_temp, "%c%1d%1d%03d%03d%03d", 'S', mode, mode_select,x, y, depth);
    uint8_t CRC = Checksum_CRC8(g_buf_temp, sizeof(g_buf_temp));
    sprintf(g_buf, "%c%1d%1d%03d%03d%03d%03d%c", 'S',mode, mode_select, 320, 240, depth, CRC, 'E');
    write(fd,g_buf,sizeof(g_buf));
    // std::cout<<"depth: "<<depth<<endl;
    //std::cout<<"g_buf: "<<g_buf<<std::endl;
    usleep(1);
}

/** CRC8校验函数
 *
 *  @param:  char *buf   需要检验的字符串
 *  @param:  uint16_t len 是否进行奇偶校验,'Y'表示需要奇偶校验,'N'表示不需要奇偶校验
 *
 *  @return: bool  初始化是否成功
 *  @brief:  CRC8校验 ---MAXIM x8+x5+x4+x1  多项式 POLY（Hex）:31(110001)  初始值 INIT（Hex）：00  结果异或值 XOROUT（Hex）：
 *  @note:   在使用其他本类提供的函数前,请先调用本函数进行串口的初始化
 *　　　　　   函数提供了一些常用的串口参数设置
 *           本串口类析构时会自动关闭串口,无需额外执行关闭串口
 */
uint8_t SerialPort::Checksum_CRC8(char *buf,uint16_t len)
{
    uint8_t check = 0;

    while(len--)
    {
        check = CRC8Tab[check^(*buf++)];
    }

    return (check)&0x00ff;
}
