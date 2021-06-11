#include <Servo.h>

Servo myarm_0;
Servo myarm_1;
Servo myarm_2;
Servo myarm_scratch;

int target_position_0 = 50;
int target_position_1 = 50;
int target_position_2 = 10;
#define UL_trigPin_1 28
#define UL_echoPin_1 29
// #define UL_trigPin_1 22
// #define UL_echoPin_1 46


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


// 微调角度
void scrach_angle(Servo myservo, int angle)
{
  int rotation_direction = angle > 0 ? 70 : \
                           angle == 0 ? 90 : 110;

  int tmp = 10 * abs(angle);
  myservo.write(rotation_direction);  // stop
  delay(tmp);
  myservo.write(90);
}

void scrach_angle_catch(Servo myservo, int angle)
{
  int rotation_direction = angle > 0 ? 70 : \
                           angle == 0 ? 90 : 110;

  int tmp = 10 * abs(angle);
  myservo.write(rotation_direction);  
  delay(tmp);
}


void Clean_buf(char *buf,int len)
{
  for(int i = 0;i < len;i++)
  {
    buf[i] = '\0';
  } 
}


void changePositionSlow(Servo myservo, int targetPosition){
    int currentPosition = myservo.read();
    int i;
    if (targetPosition >= currentPosition){
      for (i = currentPosition; i <= targetPosition; i++){
        myservo.write(i);
        delay(15);
      }
    }
    else{
      for (i = currentPosition; i>= targetPosition; i--){
        myservo.write(i);
        delay(15);
      }
    }
}

void armInit(){
  changePositionSlow(myarm_1,60);
  changePositionSlow(myarm_0,70);
  changePositionSlow(myarm_2,0);
}



void armAbove_big(){

  scrach_angle(myarm_scratch,90);
  scrach_angle(myarm_scratch,-60);  // 先张大一点点
  changePositionSlow(myarm_2,45);
  changePositionSlow(myarm_1,80);
  changePositionSlow(myarm_0,90);
  changePositionSlow(myarm_1,110);
  // changePositionSlow(myarm_0,110);
  // delay(500);
  changePositionSlow(myarm_0,120);

  // scrach_angle(myarm_scratch,-50);  // 张大
  // delay(500);

  // 130 zhangkai
  changePositionSlow(myarm_0,130);
  // changePositionSlow(myarm_0,140);
  changePositionSlow(myarm_2,30);
}


void armAbove(){

  scrach_angle(myarm_scratch,90);
  scrach_angle(myarm_scratch,-20);  // 先张大一点点
  changePositionSlow(myarm_2,45);
  changePositionSlow(myarm_1,80);
  changePositionSlow(myarm_0,90);
  changePositionSlow(myarm_1,120);

  // 130 zhangkai
  changePositionSlow(myarm_0,130);
  scrach_angle(myarm_scratch,-45);  // 张大
  
  changePositionSlow(myarm_0,140);
  changePositionSlow(myarm_2,30);

  // changePositionSlow(myarm_1,130);
  // changePositionSlow(myarm_0,150);

}

void armUnder_big(){
  // changePositionSlow(myarm_0,110);
  // changePositionSlow(myarm_2,30);

  scrach_angle(myarm_scratch,50); // 先调至最小
  changePositionSlow(myarm_0,30);

  // 临时加的0604
  changePositionSlow(myarm_2,0);

  changePositionSlow(myarm_1,0);

  // scrach_angle(myarm_scratch,-50);  
  changePositionSlow(myarm_0,50);
  // add?
  scrach_angle(myarm_scratch,-70);
    

  changePositionSlow(myarm_2,30);
  changePositionSlow(myarm_0,80);
  changePositionSlow(myarm_2,60);
  changePositionSlow(myarm_0,100);

  //  changePositionSlow(myarm_2,100);


  //  changePositionSlow(myarm_0,120);



}

void armUnder_small(){
  
  scrach_angle(myarm_scratch,50); // 先调至最小
  changePositionSlow(myarm_0,30);

  // 临时加的0604
  changePositionSlow(myarm_2,0);

  changePositionSlow(myarm_1,0);


  changePositionSlow(myarm_2,10);
  changePositionSlow(myarm_0,60);
  changePositionSlow(myarm_2,50);

  changePositionSlow(myarm_2,10);

  // scrach_angle(myarm_scratch,-50);  
  changePositionSlow(myarm_0,50);

// 0,70
// 0,50


  changePositionSlow(myarm_2,30);
  // add?
  scrach_angle(myarm_scratch,-70);
    
  changePositionSlow(myarm_0,80);
  changePositionSlow(myarm_2,60);
  changePositionSlow(myarm_0,100);

   changePositionSlow(myarm_2,100);


   changePositionSlow(myarm_0,120);

}


void armAbove_back(){
  changePositionSlow(myarm_0,80);
  //changePositionSlow(myarm_1,70);
  //changePositionSlow(myarm_2,50);
  // changePositionSlow(myarm_0,50);
  //changePositionSlow(myarm_1,50);
  // changePositionSlow(myarm_2,50);
}

void armUnder_back(){
  
  changePositionSlow(myarm_0,80);
  changePositionSlow(myarm_2,30);
  changePositionSlow(myarm_0,50);
  changePositionSlow(myarm_2,10);
  changePositionSlow(myarm_0,30);
  changePositionSlow(myarm_2,100);
  
  // changePositionSlow(myarm_1,50);
  // changePositionSlow(myarm_0,50);

}

// 抓住下面的大物品后返回
void armUnder_back_big(){
  
  changePositionSlow(myarm_2,50);
  changePositionSlow(myarm_0,100);
  changePositionSlow(myarm_2,0);
  changePositionSlow(myarm_0,60);
  changePositionSlow(myarm_1,0);
  changePositionSlow(myarm_0,30);
  changePositionSlow(myarm_1,60);

}



void setup(){

    myarm_0.attach(4);
    myarm_1.attach(5);
    myarm_2.attach(6);
    myarm_scratch.attach(7);


    pinMode(UL_trigPin_1,OUTPUT);
    pinMode(UL_echoPin_1,INPUT);

    armInit();

    Serial.begin(115200);

}

#define MAX_BUFFER_LENGTH 1000
char receive_buf[MAX_BUFFER_LENGTH];
int target_arm = -1;
int target_scratchAgle = 90;

unsigned long dist_mm = 0;

void loop(){
  Clean_buf(receive_buf, MAX_BUFFER_LENGTH);
  while(Serial.available()>0){
    delay(100);
    Serial.readBytesUntil(';',receive_buf, MAX_BUFFER_LENGTH);
    int length = strlen(receive_buf);
    target_arm = receive_buf[0] - '0';


    switch( target_arm ){
      case 0 : 
              target_position_0 = atoi(&receive_buf[2]);
              changePositionSlow(myarm_0, target_position_0);
              break;
      case 1 : 
              target_position_1 = atoi(&receive_buf[2]);
              changePositionSlow(myarm_1, target_position_1);       
              break;
      case 2 : 
              target_position_2 = atoi(&receive_buf[2]);
              changePositionSlow(myarm_2, target_position_2);
              break;
      case 3 :
              armInit();
              break;
      case 4 :
              // armAbove();
              target_scratchAgle = atoi(&receive_buf[2]);
              scrach_angle_catch(myarm_scratch,target_scratchAgle);
              break;
      case 5 :
              armInit();
              changePositionSlow(myarm_2, 20);
              delay(2000);

              armAbove_big();
              scrach_angle_catch(myarm_scratch,150);
              delay(300);
              armAbove_back();
              delay(100);
              scrach_angle(myarm_scratch,-30);
              break;
      case 6 :
              armInit();
              changePositionSlow(myarm_2, 20);
              delay(2000);

              armUnder_big();
              scrach_angle_catch(myarm_scratch,200);
              delay(300);
              armUnder_back_big();
              delay(100);
              scrach_angle(myarm_scratch,-30);
              break;
      case 7 :
              target_scratchAgle = atoi(&receive_buf[2]);
              scrach_angle(myarm_scratch,target_scratchAgle);
              break;
      // 以下case 8,9选项能单独测试抓取、放下，一连串的动作
      case 8:
              armInit();
              changePositionSlow(myarm_2, 20);
              delay(2000);

              armAbove();
              scrach_angle_catch(myarm_scratch,150);
              delay(300);
              armAbove_back();
              delay(100);
              scrach_angle(myarm_scratch,-30);
              break;
       case 9:
              armInit();
              changePositionSlow(myarm_2, 20);
              delay(2000);

              armUnder_small();
              scrach_angle_catch(myarm_scratch,200);
              delay(300);
              armUnder_back_big();
              delay(100);
              scrach_angle(myarm_scratch,-30);
              break;
        case 10:
            armAbove();
            break;
    }

    Serial.println("ok!");
    Serial.print("arm0 position = ");
    Serial.println(myarm_0.read());
    Serial.print("arm1 position = ");
    Serial.println(myarm_1.read());
    Serial.print("arm2 position = ");
    Serial.println(myarm_2.read());
    Serial.print("armScratch position = ");
    Serial.println(target_scratchAgle);
    Serial.println("**********************");


  }

   dist_mm = Scan_distance(UL_trigPin_1, UL_echoPin_1);
   Serial.print("distance (mm) = ");
   Serial.println(dist_mm);
   delay(1000);


}
