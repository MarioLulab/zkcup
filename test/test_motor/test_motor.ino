// 下为主动轮引脚
#define PWMA 3    // 左轮
#define PWMB 4    // 右轮
// 下为L298N引脚
#define IN1 24  
#define IN2 26  // 控制左轮
#define IN3 28
#define IN4 30  // 控制右轮

// 下为红外部分引脚
// left1 left2 right2 right1
#define Red_Forward_0  52   // 背对从左往右为  ， 0->3 
#define Red_Forward_1  50
#define Red_Forward_2  48
#define Red_Forward_3  46

#define Red_Center_0  53       // 从前开始，逆时针，0->3
#define Red_Center_1  51
#define Red_Center_2  49
#define Red_Center_3  47

void test_read_REDandMotor(){
        int a0 = digitalRead(Red_Forward_0);
        int a1 = digitalRead(Red_Forward_1);
        int a2 = digitalRead(Red_Forward_2);
        int a3 = digitalRead(Red_Forward_3);

        int b0 = digitalRead(Red_Center_0);
        int b1 = digitalRead(Red_Center_1);
        int b2 = digitalRead(Red_Center_2);
        int b3 = digitalRead(Red_Center_3);

        Serial.print("red_forward0 (left_1)= ");
        Serial.println(a0);
        
        Serial.print("red_forward1 (left_2)= ");
        Serial.println(a1);

        Serial.print("red_forward2 (right_2)= ");
        Serial.println(a2);

        Serial.print("red_forward3 (right_1)= ");
        Serial.println(a3);

        Serial.print("red_center0 (left_1)= ");
        Serial.println(b0);
        
        Serial.print("red_center1 (left_2)= ");
        Serial.println(b1);

        Serial.print("red_center2 (right_2)= ");
        Serial.println(b2);

        Serial.print("red_center3 (right_1)= ");
        Serial.println(b3);

        Serial.println("********************************");
        
}

void setup(){
    pinMode(PWMA,OUTPUT);
    pinMode(PWMB,OUTPUT);
    pinMode(IN1,OUTPUT);
    pinMode(IN2,OUTPUT);
    pinMode(IN3,OUTPUT);
    pinMode(IN4,OUTPUT);

    pinMode(Red_Forward_0,INPUT);
    pinMode(Red_Forward_1,INPUT);
    pinMode(Red_Forward_2,INPUT);
    pinMode(Red_Forward_3,INPUT);
    pinMode(Red_Center_0,INPUT);
    pinMode(Red_Center_1,INPUT);
    pinMode(Red_Center_2,INPUT);
    pinMode(Red_Center_3,INPUT);
    Serial.begin(115200);

   digitalWrite(IN1,HIGH);
   digitalWrite(IN2,LOW);
   digitalWrite(IN3,HIGH);
   digitalWrite(IN4,LOW);
   analogWrite(PWMA,200);
   analogWrite(PWMB,200);

}

void loop(){
  test_read_REDandMotor();

  delay(1500);
}
