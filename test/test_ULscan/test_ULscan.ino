#define UL_NUM 3    // 超声波模块的总个数
// 定义所有超声波模块的相关引脚
#define UL_trigPin_1 22
#define UL_echoPin_1 46
#define UL_trigPin_2 3
#define UL_echoPin_2 4
#define UL_trigPin_3 5
#define UL_echoPin_3 6
bool Scan_State[UL_NUM] = {false}; // 记录所有超声波模块的探测状况,true表示探测到


/********************************************************
函数功能: 底层函数，超声波检测
作者：Lu
入口参数：超声波模块的触发引脚 -> ultransonic_tr_pin
         超声波模块的回声引脚 -> ultransonic_ec_pin
         判断距离distance,单位mm
返 回 值: 小于distance返回true
影响参数：
********************************************************/
bool UL_Detecting(int ultransonic_tr_pin,int ultransonic_ec_pin,int distance){
      unsigned long time_echo_us = 0;
      unsigned long dist_mm = 0;
      unsigned long avg_dist = 0;
      unsigned long last_dist = 0;
      int count = 0;

      // 检测5次距离
      for(int i = 0;i<5;i++){
        digitalWrite(ultransonic_tr_pin, LOW);   // 先拉低，以确保脉冲识别正确
        delayMicroseconds(2);         // 等待2us
        digitalWrite(ultransonic_tr_pin, HIGH);  // 开始通过Trig/Pin 发送脉冲
        delayMicroseconds(12);        // 设置脉冲宽度为12us (>10us)
        digitalWrite(ultransonic_tr_pin, LOW);   // 结束脉冲
        time_echo_us = pulseIn(ultransonic_ec_pin, HIGH);          // 计算US-100 返回的脉冲宽度
        if((time_echo_us < 60000) && (time_echo_us > 1))// 脉冲有效范围(1, 60000).
        {
          // dist_mm = (time_echo_us * 0.34mm/us) / 2 (mm)
          dist_mm = time_echo_us*5/29;        // 通过脉冲宽度计算距离.
          count++;
          avg_dist += dist_mm;  
        }
        else
          return false;
     }
        if(count!=0)
          avg_dist = avg_dist/count;
         
//        Serial.println(avg_dist);
//        Serial.println(count);
        if(avg_dist < distance)   return true;
        else return false;
}



/********************************************************
函数功能: 顶层函数，超声波检测，调用此函数可以更改Scan_State[]
作者：Lu
入口参数：无
返 回 值: 无
影响参数：Scan_State[UL_NUM]
********************************************************/
void Scan_Echo(void){
  Scan_State[0] = UL_Detecting(UL_trigPin_1, UL_echoPin_1, 150);
  Scan_State[1] = UL_Detecting(UL_trigPin_2, UL_echoPin_2, 150);
  Scan_State[2] = UL_Detecting(UL_trigPin_3, UL_echoPin_3, 150);
}

// 返回距离，单位mm
unsigned long Scan_distance(int ultransonic_tr_pin,int ultransonic_ec_pin){
    unsigned long time_echo_us = 0;
    unsigned long dist_mm = 0;
    unsigned long avg_dist = 0;
    unsigned long last_dist = 0;
    int count = 0;

    int i;
//    Serial.println("ok----------------------");
    // 检测5次距离

    for(i = 0;i<5;i++){
//        Serial.println(i);
        digitalWrite(ultransonic_tr_pin, LOW);   // 先拉低，以确保脉冲识别正确
        delayMicroseconds(2);         // 等待2us
        digitalWrite(ultransonic_tr_pin, HIGH);  // 开始通过Trig/Pin 发送脉冲
        delayMicroseconds(12);        // 设置脉冲宽度为12us (>10us)
        digitalWrite(ultransonic_tr_pin, LOW);   // 结束脉冲
        time_echo_us = pulseIn(ultransonic_ec_pin, HIGH);          // 计算US-100 返回的脉冲宽度

//        Serial.print("time_echo_us : ");
//        Serial.println(time_echo_us);

        if((time_echo_us < 60000) && (time_echo_us > 1))// 脉冲有效范围(1, 60000).
        {
            // dist_mm = (time_echo_us * 0.34mm/us) / 2 (mm)
            dist_mm = time_echo_us*5/29;        // 通过脉冲宽度计算距离.
            count++;
            avg_dist += dist_mm;  
        }
        else{
        //     Serial.print("time_echo_us = ");
        //   Serial.println(time_echo_us);
            return 0;
            // return false;
        }
    }
    
    if(count!=0)
    {    
        avg_dist = avg_dist/count;
    }

        // Serial.print("time_echo_us = ");
        // Serial.println(time_echo_us);


    return avg_dist;
//        Serial.println(avg_dist);
//        Serial.println(count);
    // if(avg_dist < distance)   return true;
    // else return false;
}


void setup(){
    pinMode(UL_trigPin_1,OUTPUT);
    pinMode(UL_echoPin_1,INPUT);

    Serial.begin(115200);

}


unsigned long dist_mm = 0;

void loop(){

    dist_mm = Scan_distance(UL_trigPin_1, UL_echoPin_1);
    bool x = UL_Detecting(UL_trigPin_1,UL_echoPin_1,50);
    Serial.print("distance (mm) = ");
    Serial.println(dist_mm);
    Serial.println(x);
    delay(1000);

}
