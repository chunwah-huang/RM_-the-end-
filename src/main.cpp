/**
 * @file main.cpp
 * @author GCUROBOT-Visual-Group (GCUROBOT_WOLF@163.com)
 * @brief RM 2019 步兵视觉　main入口文件
 * @version 1.1
 * @date 2019-05-01
 * @copyright Copyright (c) 2019 GCU Robot Lab. All Rights Reserved.
 */
#include "rm_link.h"

int main()
{
    /** code init **/
    RM_Vision_Init run;
    SerialPort::serialSet(1);
    for(;;)
    {
        double t1 = getTickCount();
        /** run **/
        run.Run();
  
        if(run.is_exit())
        {
            break;
        }
        double t2 = getTickCount();
        double FPS = 1.0 / ((t2-t1)/getTickFrequency());
        cout << "fps:  " << FPS<< endl;
    }
    return 1;
}

