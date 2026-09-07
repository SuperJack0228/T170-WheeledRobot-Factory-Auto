#include "Can_Ti5.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <unordered_map>
#include <utility>
#include <vector>



int main()
{
    Can_Motor skt;
    for(int i = 16; i < 30; i++)
    {   
        set_Position_KP( i,  3000);					   // 设置位置环 KP
        set_Position_KD(i ,  100);		
        set_Speed_KP(i ,  1000);	
        Save_Parameters(i);	
    }

    for(int i = 1; i < 5; i++)
    {   
        set_Position_KP( i,  3000);					   // 设置位置环 KP
        set_Position_KD(i ,  100);		
        set_Speed_KP(i ,  1500);		
      

         set_Max_Speed( i,  2000);						   // 设置最大速度
         set_Min_Speed( i,  -2000);	
        Save_Parameters(i);	
    }

   
       
    

  
}
