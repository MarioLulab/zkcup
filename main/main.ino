#include <FlexiTimer2.h>
#include <Servo.h>

// #define DEBUG ON    // 此行若没注释，则当前为"不与"Python程序通讯的DEBUG模式，串口输出调试信息

#define DEBUG_CONNECTED ON // 此行若没注释，则当前为"与"Python程序通讯的DEBUG模式，串口输出调试信息

/*--------------------------------------------
内容：抓取运动模块关键参量
最后一次修改时间: 2021.03.01
最后 一次修 改人: 陆琦
备注信息： 
--------------------------------------------*/
#define MYARM_0_PIN 4
#define MYARM_1_PIN 5
#define MYARM_2_PIN 6
#define MYARM_SCRATCH_PIN 7


// 定义Point结构体，记录坐标系中的点
typedef struct point{
    char x;
    char y;
}Point;

// 遍历Shelf_State_List的返回值,如果为-1,-1说明该shelf_id所有的货窗都已经被checked
int Traversal_Return_line = -1;
int Traversal_Return_column = -1;


// 记录机器人当前抓取的货架编号
char Buy_shelf = 'D';

// 定义舵机号
Servo myarm_0;  // 底层舵机
Servo myarm_1;  // 第二层舵机
Servo myarm_2;  // 第三层舵机
Servo myarm_scratch;    // 机械爪舵机

// 与抓取运动时对当前点的修正有关,判断是否前向红外传感是否经过了第二条白线
int flag_white_pass = 0;    // 0-->还未经过第二根白线

typedef struct Shelf_State_Node Shelf_State;                             
struct Shelf_State_Node {
    char x;
    char y;     // 记录货窗正前方的坐标
    int num[2];    // 记录目前该货窗还剩下的货品数量,值-1表示初始量，num[0]表示下货窗，num[1]表示上货窗
    int isChecked;  // 记录该货窗是否被拍照识别过，0表示未被拍照识别过
};
Shelf_State Shelf_State_List[4][5] = {
    {{'B','0',{0,0},1},{'C','0',{0,0},1},{'D','0',{0,0},1},{'E','0',{0,0},1},{'F','0',{1,1},1}},   // (B,0) ~ (F,0) --> D货窗,X轴正向
    {{'J','1',{0,0},0},{'J','2',{0,0},0},{'J','3',{0,0},0},{'J','4',{0,0},0},{'J','5',{0,0},0}},   // (J,1) ~ (J,5) --> C货窗,Y轴正向
    {{'I','9',{0,0},0},{'H','9',{0,0},0},{'G','9',{0,0},0},{'F','9',{0,0},0},{'E','9',{0,0},0}},   // (I,9)~ (E,9)--> B货窗,X轴负向
    {{'A','8',{0,0},0},{'A','7',{0,0},0},{'A','6',{0,0},0},{'A','5',{0,0},0},{'A','4',{0,0},0}},   // (A,8) ~ (A,4) --> A货窗,Y轴负向
};



/*--------------------------------------------
内容：下位机通讯模块关键参量
最后一次修改时间: 2021.03.02
最后 一次修 改人: 陆琦
备注信息： 增加了退出串口通讯的设置
--------------------------------------------*/

/********************************************************
定义帧格式：帧头 + 内容


arduino向树莓派发送的帧：
Byte0       Byte1       Byte2...ByteN       Byte(N+1)
  |           |              |                |
请求类型     帧长度         内  容            结束符
_______       
  帧头                              

Byte0 : 请求and回复类型, 'P'表示请求树莓派进行Path planning路径规划;'R'表示请求树莓派进行Recognition物体识别;'S'表示请求关闭串口通讯
Byte1 : 帧的总长度，以字节Byte为单位，不包括结束符
Byte2...ByteN : 内容
Byte(N+1) : 结束符. 以分号';'为结束符

例子1：
比如arduino要向树莓派请求计算从A1->A3的可行路径，则该帧为：
'P'  0x06 'A'   '1'   'A'   '3'   ';'

树莓派回复:
'P' 0x08 'A' '1' 'A' '2' 'A' '3' ';'

意思为树莓派计算得到的路径为A1->A2->A3


例子2：
比如arduino要向树莓派请求拍照识别，则该帧为:
'R' 0x2 ';'

树莓派回复:
'R' 0x3 'A' ';'

意思为树莓派识别得到的物体种类代号为'A'

例子3：
比如arduino要关闭和树莓派的串口通讯，则该帧为：
'S' 0X2 ';'

********************************************************/
#define MAX_BUFFER_LENGTH 200

char Receive_buf[MAX_BUFFER_LENGTH];
char Send_buf[MAX_BUFFER_LENGTH];
char DEBUG_Send_buf[MAX_BUFFER_LENGTH];
char Tmp_buf[MAX_BUFFER_LENGTH];
char Nowpoint_buf[3];

void Request_recog();
void Request_path(char x1, char y1, char x2, char y2);
void Request_Close();
void Do_Path_results();
void Do_Recog_results();
int Receive();
void Parse_receive_buf();
int Check_buf();
void Clean_buf(char *buf,int len);





/*--------------------------------------------
内容：运动模块关键参量
最后一次修改时间: 2021.03.01
最后 一次修 改人: 陆琦
备注信息：引脚定义待修改 
--------------------------------------------*/
// 下为红外部分引脚
// left1 left2 right2 right1
#define Red_Forward_0  38   // 背对从左往右为  ， 0->3 
#define Red_Forward_1  39
#define Red_Forward_2  40
#define Red_Forward_3  41

#define Red_Center_0  42       // 从前开始，逆时针，0->3
#define Red_Center_1  43
#define Red_Center_2  44
#define Red_Center_3  45


// 下为主动轮引脚
#define PWMA 2    // 左轮
#define PWMB 3    // 右轮
// 下为L298N引脚
#define IN1 30  
#define IN2 31  // 控制左轮
#define IN3 32
#define IN4 33  // 控制右轮 

// 小车运动初始常量
#define Value_Setpoint 250 // 正常向前寻迹时的轮子转速
#define Delta_MoveForward 20 // 前进时，使用斜坡信号增速或减速的变化量

// 运动数值
int V_NOW_L, V_NOW_R;   // 车轮当前速度
int Value_Red_ForWard[4] = {0}; // 储存前向红外传感器数值的数组
int Value_Red_Center[4] = {0};  // 储存中间红外传感器数值的数组
int Value_Red_ForWard_last[4] = {0};
int Setpoint_L = Value_Setpoint; // 左轮期望车速
int Setpoint_R = Value_Setpoint; // 右轮期望车速

// 运动状态
int Motion_Status = 0; // 双轮状态{0:停止, 1:前进, 2:转弯, 3:后退}
volatile int Flag_Count = 1; // 与判断是否到线有关的关键参量，具体可见Exam_Arrival_Point()函数
int Move_State; // 在Movement_block()中控制小车该如何运动


// 防误触时间
int dtime_interval;
unsigned long time_interval;

// 向前一格走时用来计时
volatile unsigned long start_time;
volatile unsigned long now_time;

// 移动每格的时间不超过1s
volatile unsigned long Current_time;
volatile unsigned long Last_time;



void Read_RedValue(void);
void stop(void);
void Limit_PWM(void);
void Set_PWM(void);
void Move_ForwardSlope(int delta);
void start_slope(void);
void stop_slope(void);
void Control(int receive_data);



/*--------------------------------------------
内容：路径规划模块关键参量
最后一次修改时间: 2021.03.01
最后 一次修 改人: 陆琦
备注信息： 
--------------------------------------------*/
#define CAPACITY_PATH_ARRAY 100
#define X_POS 0
#define X_NEG 2
#define Y_POS 1
#define Y_NEG 3



// 定义Planned_Path结构体，记录arduino接收到的规划好的路径
typedef struct planned_path{
    Point Path_Array[CAPACITY_PATH_ARRAY];
    int EndIndex;
    int NowIndex;
}Planned_Path;


int Direction; // 标志当前车头转向 {0->x轴正向,右; 1->y轴正向，上; 2->x轴负向, 左; 3->y轴负向, 下}
Point Now_Point, Next_Point;    // 当前所处点和下一个点
Planned_Path Now_Planned_Path;  // 当前规划得到的规划路径, 在主函数中记得初始化EndIndex




/*--------------------------------------------
内容：识别模块（包括拍照识别和超声波探测）关键参量定义
最后一次修改时间: 2021.03.01
最后 一次修 改人: 陆琦
备注信息： 拍照识别部分待添加
--------------------------------------------*/
#define UL_NUM 2    // 超声波模块的总个数


// 定义所有超声波模块的相关引脚
#define UL_trigPin_1 28
#define UL_echoPin_1 29
#define UL_trigPin_2 22
#define UL_echoPin_2 46
#define UL_trigPin_3 5
#define UL_echoPin_3 6


int Scan_State[UL_NUM] = {0}; // 记录所有超声波模块的探测状况,true表示探测到



/*----------------------------------------------------------------------------------------
内容：抓取运动模块
最后一次修改时间: 2021.03.01
最后 一次修 改人: 陆琦
备注信息： 
*/


/********************************************************
函数功能：微调机械爪角度（不会死死的抓住）
作者：Lu
入口参数：myservo : 对应机械爪的舵机对象
         angle : > 0 闭上angle的角度
                 < 0 打开abs(angle)的角度

返 回 值: 无
影响参数：myservo
********************************************************/
void scrach_angle(Servo myservo, int angle)
{
  int rotation_direction = angle > 0 ? 70 : \
                           angle == 0 ? 90 : 110;

  int tmp = 10 * abs(angle);
  myservo.write(rotation_direction);  // stop
  delay(tmp);
  myservo.write(90);
}

/********************************************************
函数功能：微调机械爪角度（死死的抓住）
作者：Lu
入口参数：myservo : 对应机械爪的舵机对象
         angle : > 0 闭上angle的角度
                 < 0 打开abs(angle)的角度

返 回 值: 无
影响参数：myservo
********************************************************/
void scrach_angle_catch(Servo myservo, int angle)
{
  int rotation_direction = angle > 0 ? 70 : \
                           angle == 0 ? 90 : 110;

  int tmp = 10 * abs(angle);
  myservo.write(rotation_direction);  // stop
  delay(tmp);
}

/********************************************************
函数功能：微调舵机角度（死死的抓住）
作者：Lu
入口参数：myservo : 对应机械爪的舵机对象
         targetPosition : 舵机目标角度

返 回 值: 无
影响参数：myservo
********************************************************/
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

/********************************************************
函数功能：机械臂位置调整函数
作者：Lu
入口参数：
返 回 值: 无
影响参数：myarm_0, myarm_1, myarm_2
********************************************************/
// 机械臂初始位置
void armInit(){
  changePositionSlow(myarm_1,60);
  changePositionSlow(myarm_0,70);
  changePositionSlow(myarm_2,0);
}

// 拍照时，为了放置摄像头拍到物品，所以需要调整机械臂高度
void armForPhoto(){
    changePositionSlow(myarm_0,20);
    changePositionSlow(myarm_1,0);
    changePositionSlow(myarm_2,0);   
}

// 舵机运动抓上面的物品
void armAbove(){

    /* 最新版 */
    scrach_angle(myarm_scratch,-20);  // 先张大一点点
    changePositionSlow(myarm_2,45);
    changePositionSlow(myarm_1,80);
    changePositionSlow(myarm_0,90);
    changePositionSlow(myarm_1,120);

    changePositionSlow(myarm_0,130);
    scrach_angle(myarm_scratch,-45);  // 张大
    
    changePositionSlow(myarm_0,140);
    changePositionSlow(myarm_2,30);


}

// 舵机运动抓下面的物品
void armUnder_small(){

    scrach_angle(myarm_scratch,50); // 先调至最小
    changePositionSlow(myarm_0,30);
    changePositionSlow(myarm_1,0);


    changePositionSlow(myarm_2,10);
    changePositionSlow(myarm_0,60);
    changePositionSlow(myarm_2,50);

    changePositionSlow(myarm_2,10);

    // scrach_angle(myarm_scratch,-50);  
    changePositionSlow(myarm_0,50);


    changePositionSlow(myarm_2,30);
    // add?
    scrach_angle(myarm_scratch,-50);
        
    changePositionSlow(myarm_0,80);
    changePositionSlow(myarm_2,60);
    changePositionSlow(myarm_0,100);

    changePositionSlow(myarm_2,100);


    changePositionSlow(myarm_0,120);
}

void armUnder_big(){
  scrach_angle(myarm_scratch,50); // 先调至最小
  changePositionSlow(myarm_0,30);
  changePositionSlow(myarm_1,0);

  changePositionSlow(myarm_0,50);
  // add?
  scrach_angle(myarm_scratch,-60);

  changePositionSlow(myarm_2,30);
  changePositionSlow(myarm_0,80);
  changePositionSlow(myarm_2,60);
  changePositionSlow(myarm_0,100);

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

// 舵机运动抓到上面物品后返回
void armAbove_back(){
  changePositionSlow(myarm_0,80);
//   changePositionSlow(myarm_1,70);
//   changePositionSlow(myarm_2,50);
//   changePositionSlow(myarm_0,50);
//   changePositionSlow(myarm_1,50);
}
// 舵机运动抓到下面物品后返回
void armUnder_back(){
  changePositionSlow(myarm_0,80);
  changePositionSlow(myarm_2,30);
  changePositionSlow(myarm_0,50);
  changePositionSlow(myarm_2,10);
  changePositionSlow(myarm_0,30);
  changePositionSlow(myarm_2,100);

//   changePositionSlow(myarm_1,50);
//   changePositionSlow(myarm_0,50);
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



/********************************************************
函数功能：机械抓取顶层函数
作者：Lu
入口参数：
返 回 值: 无
影响参数：myarm_0, myarm_1, myarm_2, myarm_scratch
********************************************************/
void scratchAbove(){
    armInit();
    changePositionSlow(myarm_2,20);
    scrach_angle(myarm_scratch,90);    // 闭合
    // changePositionSlow(myarm_2,0);   // 不必回到初始位置
    // scrach_angle(myarm_scratch, -20);   // 先让机械臂尽可能张开得大

    if (Buy_shelf == 'D' || Buy_shelf == 'C'){
        armAbove();
    }
    else if(Buy_shelf == 'A' || Buy_shelf == 'B'){
        armAbove_big();
    }

    // 抓上面小的
    // armAbove();


    scrach_angle_catch(myarm_scratch,150);
    delay(300);
    armAbove_back();
    delay(100);
    // scrach_angle(myarm_scratch,-30);    // 松开
}

void scratchUnder_small(){
    // armInit();
    // scrach_angle(myarm_scratch, -20);   // 先让机械臂尽可能张开得大
    armInit();
    changePositionSlow(myarm_2,20);
    scrach_angle(myarm_scratch,90);    // 闭合
    // changePositionSlow(myarm_2,0);   // 不必回到初始位置



    armUnder_small();
    scrach_angle_catch(myarm_scratch,150);
    delay(300);
    armUnder_back_big();
    delay(100);
    // scrach_angle(myarm_scratch,-30);    // 松开
}

void scratchUnder_big(){
    // armInit();
    // scrach_angle(myarm_scratch, -20);   // 先让机械臂尽可能张开得大
    armInit();
    changePositionSlow(myarm_2,20);
    scrach_angle(myarm_scratch,90);    // 闭合
    // changePositionSlow(myarm_2,0);  // 不用回到初始位置



    armUnder_big();
    scrach_angle_catch(myarm_scratch,150);
    delay(300);
    armUnder_back_big();
    delay(100);
    // scrach_angle(myarm_scratch,-30);    // 松开
}


/*---------------------------------------------------------------------------------------*/







/*----------------------------------------------------------------------------------------
内容：下位机通讯模块
最后一次修改时间: 2021.03.02
最后 一次修 改人: 陆琦
备注信息： 增添了退出串口通讯的设置
*/

/********************************************************
函数功能：请求拍照识别
作者：Lu
入口参数：无
返 回 值: 无
影响参数：Send_buf
********************************************************/
void Request_recog()
{
  Clean_buf(Send_buf,MAX_BUFFER_LENGTH);
  Send_buf[0] = 'R';
  Send_buf[1] = 0x2;
  Send_buf[2] = ';';
  Serial.print(Send_buf);
  delay(100);
  Serial.flush();
}

/********************************************************
函数功能：请求路径规划
作者：Lu
入口参数：(x1,y1),(x2,y2)
返 回 值: 无
影响参数：Send_buf
********************************************************/
void Request_path(char x1, char y1, char x2, char y2)
{
    Clean_buf(Send_buf,MAX_BUFFER_LENGTH);
    Send_buf[0] = 'P';
    Send_buf[1] = 0x6;
    Send_buf[2] = x1;
    Send_buf[3] = y1;
    Send_buf[4] = x2;
    Send_buf[5] = y2;
    Send_buf[6] = ';';

    Serial.print(Send_buf);
    delay(100);
    Serial.flush();
}

/********************************************************
函数功能：向串口发送DEBUG数据
作者：Lu
入口参数：无
返 回 值: 无
影 响 值: DEBUG_Send_buf[]
********************************************************/
void Send_DEBUG(char * s)
{
  Clean_buf(DEBUG_Send_buf,MAX_BUFFER_LENGTH);
  DEBUG_Send_buf[0] = 'D';
  DEBUG_Send_buf[1] = 'E';
  DEBUG_Send_buf[2] = 'B';
  DEBUG_Send_buf[3] = 'U';
  DEBUG_Send_buf[4] = 'G';
  DEBUG_Send_buf[5] = ':';

  int send_index = 6;
  int input_index = 0;
  int length = strlen(s);
  while(length--){
      DEBUG_Send_buf[send_index++] = s[input_index++];
  }
  DEBUG_Send_buf[send_index] = ';';
  Serial.print(DEBUG_Send_buf);
  delay(10);
  Serial.flush();
}

/********************************************************
函数功能：请求关闭串口通讯
作者：Lu
入口参数：无
返 回 值: 无
影响参数：Send_buf
********************************************************/
void Request_Close()
{
    Clean_buf(Send_buf,MAX_BUFFER_LENGTH);
    Send_buf[0] = 'S';
    Send_buf[1] = 0x2;
    Send_buf[2] = ';';
    Serial.print(Send_buf);
    Serial.flush();
}


/********************************************************
函数功能：对计算得到的路径规划结果进行处理
作者：Lu
入口参数：Receive_buf
返 回 值: 无
影响参数：Now_Planned_Path
********************************************************/
void Do_Path_results(){

    // 将树莓派规划好的路径存入Now_Planned_Path
    int path_length = (int)Receive_buf[1];
    path_length -= 2;
    path_length /= 2; // path_length 记录的是点数
    int i;

    // 清除原有记录
    for (i=0;i <= CAPACITY_PATH_ARRAY-1;i++){
        Now_Planned_Path.Path_Array[i].x = '-1';
        Now_Planned_Path.Path_Array[i].y = '-1';
    }

    for (i=0;i <= path_length-1;i++){
        Now_Planned_Path.Path_Array[i].x = Receive_buf[(i+1)*2];
        Now_Planned_Path.Path_Array[i].y = Receive_buf[(i+1)*2+1];
    }
    Now_Planned_Path.EndIndex = path_length - 1; 
    Now_Planned_Path.NowIndex = 0;

}

/********************************************************
函数功能：对拍照识别得到的结果进行处理
作者：Lu
入口参数：Traversal_Return_line, Traversal_Return_column, Receive_buf
返 回 值: 无
影响参数：Shelf_State_List[Traversal_Return_line][Traversal_Return_column].num[]
********************************************************/
void Do_Recog_results(){
    // 待完成
    // Traversal_Return_line
    // Traversal_Return_column

    int top_number = Receive_buf[2] - '0';
    int down_number = Receive_buf[3] - '0';

    if (top_number >= 1){
        top_number = 1;
    }
    
    if (down_number >= 1){
        down_number = 1;
    }

    Shelf_State_List[Traversal_Return_line][Traversal_Return_column].isChecked = 1; 
    Shelf_State_List[Traversal_Return_line][Traversal_Return_column].num[0] = down_number;
    Shelf_State_List[Traversal_Return_line][Traversal_Return_column].num[1] = top_number;


}

/*------------------------------------------------------------------------------*/




/*-------------------------------------------------------------------------------
文件内容：下位机通讯模块
最后一次修改时间: 2021.03.02
最后 一次修 改人: 陆琦
备注信息： 增添了退出串口通讯的设置
*/



/********************************************************
函数功能：读取树莓派传来的数据
作者：Lu
入口参数：无
返 回 值: numdata,存入Receive_buf的字符数，0表示没有有效数据，-1表示数据出错
影响参数：Receive_buf[]
注   意: 若串口没有接收到数据，则死循环
********************************************************/
int Receive(){
    int numdata = 0;
    Clean_buf(Receive_buf,MAX_BUFFER_LENGTH);

    // 死循环等待数据
    while(Serial.available()<=0){
        ;
    }

    if(Serial.available()>0){
        delay(100);
        numdata = Serial.readBytesUntil(';', Receive_buf, MAX_BUFFER_LENGTH);
        numdata = Check_buf();
    }

    return numdata;
}

/********************************************************
函数功能：解析Receive_buf里的数据
作者：Lu
入口参数：无
返 回 值: 无
影响参数：无
********************************************************/
void Parse_receive_buf(){
    switch(Receive_buf[0]){
        case 'P' : Do_Path_results();break;
        case 'R' : Do_Recog_results();break;
        defalut : break;
    }
}



/********************************************************
函数功能：检查缓存里的帧长度是否和帧内容里的帧长度一致,检查第一个字节是否是'P'或者'R'
作者：Lu
入口参数：无
返 回 值: numdata,存入Receive_buf的字符数,-1表示数据出错
影响参数：Receive_buf[]
********************************************************/
int Check_buf(){
    int numdata = -1;
    int data_length = (int)Receive_buf[1];

    if (data_length == strlen(Receive_buf) && (Receive_buf[0] == 'P' || Receive_buf[0] == 'R')){
        numdata = data_length;
    }


    return numdata;
}


/********************************************************
函数功能：清除缓存
作者：Lu
入口参数：无
返 回 值: 无
影响参数：Receive_buf[]或Send_buf[]
********************************************************/
void Clean_buf(char *buf,int len)
{
  for(int i = 0;i < len;i++)
  {
    buf[i] = '\0';
  } 
}

/*--------------------------------------------------------------------------------*/



/*--------------------------------------------------------------------------------
文件内容：运动模块
最后一次修改时间: 2021.03.06
最后 一次修 改人: 陆琦
备注信息： 该写的函数都写完了
*/


void test_read_REDandMOTOR(){
        int a0 = digitalRead(Red_Forward_0);
        int a1 = digitalRead(Red_Forward_1);
        int a2 = digitalRead(Red_Forward_2);
        int a3 = digitalRead(Red_Forward_3);

        int b0 = digitalRead(Red_Center_0);
        int b1 = digitalRead(Red_Center_1);
        int b2 = digitalRead(Red_Center_2);
        int b3 = digitalRead(Red_Center_3);

        #if defined(DEBUG)

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

        Serial.print("Setpoint_L = ");
        Serial.println(Setpoint_L);
        Serial.print("V_NOW_L = ");
        Serial.println(V_NOW_L);

        Serial.print("Setpoint_R = ");
        Serial.println(Setpoint_R);
        Serial.print("V_NOW_R = ");
        Serial.println(V_NOW_R);

        Serial.println("********************************");

        #endif

        // sprintf(Tmp_buf,"Setpoint_L=%d,Setpoint_R=%d",Setpoint_L,Setpoint_R);
        // Send_DEBUG(Tmp_buf);
        
}


/********************************************************
函数功能：读取红外传感当前电平
作者：Lu
入口参数：无
返 回 值：无
影响参数：Value_Red_ForWard[]和Value_Red_ForWard[]
备注信息：如果是黑块，则值为1；如果是白块，则值为0
********************************************************/

void Read_RedValue(){

    Value_Red_ForWard[0] = digitalRead(Red_Forward_0);
    Value_Red_ForWard[1] = digitalRead(Red_Forward_1);
    Value_Red_ForWard[2] = digitalRead(Red_Forward_2);
    Value_Red_ForWard[3] = digitalRead(Red_Forward_3);
    Value_Red_Center[0] = digitalRead(Red_Center_0);
    Value_Red_Center[1] = digitalRead(Red_Center_1);
    Value_Red_Center[2] = digitalRead(Red_Center_2);
    Value_Red_Center[3] = digitalRead(Red_Center_3);

}

/********************************************************
函数功能：存储上次中间的红外传感的电平，用于抓取运动时的手工
          计点
作者：Lu
入口参数：Value_Red_Forward[]
返 回 值：无
影响参数：Value_Red_ForWard_last[]
备注信息：如果是黑块，则值为1；如果是白块，则值为0
********************************************************/
void Store_ForwardRedValue(){
    Value_Red_ForWard_last[0] = Value_Red_ForWard[0];
    Value_Red_ForWard_last[1] = Value_Red_ForWard[1];
    Value_Red_ForWard_last[2] = Value_Red_ForWard[2];
    Value_Red_ForWard_last[3] = Value_Red_ForWard[3];
}


/********************************************************
函数功能：车轮电机停止
作者：Lu
入口参数：无
返 回 值：无
影响参数：motion_status
备注：具体要看L298N的接线
********************************************************/
void stop(){

    Motion_Status = 0;
    digitalWrite(PWMA,LOW);
    digitalWrite(PWMB,LOW);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

}

/********************************************************
函数功能：限制PWM大小
作者：Lu
入口参数：无
返 回 值：无
影响参数：V_NOW_R,V_NOW_L
备注：满幅值需要另外调整
********************************************************/
void Limit_PWM(void){
int Amplitude = 250;  //===PWM满幅是255 限制在250
  if (V_NOW_R < -Amplitude) V_NOW_R = -Amplitude;
  if (V_NOW_R > Amplitude)  V_NOW_R = Amplitude;
  if (V_NOW_L < -Amplitude) V_NOW_L = -Amplitude;
  if (V_NOW_L > Amplitude)  V_NOW_L = Amplitude;
}

/********************************************************
函数功能：这里根据V_NOW_L和V_NOW_R的正负决定电机的转动方向, 并且给端口PWMA、PWMB赋值
作者：Lu
入口参数：无
返 回 值：无
影响参数：无.但是直接影响左右轮转速
********************************************************/
void Set_PWM(){

  int DIFFERENCE = 0;   // 误差量
  
  // 决定左轮转向
  if (V_NOW_L > 0){
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      }  //L298N的电平控制
  else{
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      } //L298N的电平控制

  analogWrite(PWMA, abs(V_NOW_L) - DIFFERENCE); // 写左轮转速

  // 决定右轮转向 
  if (V_NOW_R > 0){
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }//L298N的电平控制
  else{
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    } //L298N的电平控制

  analogWrite(PWMB, abs(V_NOW_R));  // 写右轮转速

}

/********************************************************
函数功能：向前移动时，用到的斜坡信号增速
作者：Lu
入口参数：delta, 斜坡增量
返 回 值：无
影响参数：V_NOW_L 和 V_NOW_R
********************************************************/
void Move_ForwardSlope(void){
    int delta = Delta_MoveForward;
    int flag_L = Setpoint_L < V_NOW_L ? -1 : 1;
    int flag_R = Setpoint_R < V_NOW_R ? -1 : 1;

    if( abs(V_NOW_L + delta*flag_L) < abs(Setpoint_L)){
        V_NOW_L += flag_L*delta;
    }else{
        V_NOW_L = Setpoint_L;
    }

    if( abs(V_NOW_R + delta*flag_R) < abs(Setpoint_R)){
        V_NOW_R += flag_R*delta;
    }else{
        V_NOW_R = Setpoint_R;
    }
    
    Limit_PWM();
    Set_PWM();

}


void start_slope(){
    FlexiTimer2::start();
}

void stop_slope(){
    FlexiTimer2::stop();
}


/********************************************************
函数功能：底层函数，控制小车直行、原地左转向、原地右转向和直行倒车
作者：Lu
入口参数：command {1:直行, 2:原地左转 3:原地右转, 4:直行倒车}
返 回 值：无
影响参数：Setpoint_L,Setpoint_R,V_NOW_L,V_NOW_R
备注：给的V_NOW_L和V_NOW_R可能需要另外调整
********************************************************/
void Control(int receive_data){
    switch(receive_data){
        case 0x01 : {
            // 直行
            Setpoint_L = Value_Setpoint;
            Setpoint_R = Value_Setpoint;
            break;
        }
        case 0x02 : {
            // 原地左转向
            V_NOW_L = -190;
            V_NOW_R = 190;
            Limit_PWM();
            Set_PWM();
            break;
        }
        case 0x03 : {
            // 原地右转向
            V_NOW_L = 190;
            V_NOW_R = -190;
            Limit_PWM();
            Set_PWM();
            break;
        }
        case 0x4 : {
            // 直行倒车
            V_NOW_L = -160;
            V_NOW_R = -160;
            Limit_PWM();
            Set_PWM();
            break;
        }
    }
}

/********************************************************
函数功能：防止转弯的过程中Value_Red_ForWard[0]或Value_Red_ForWard[3]传感器
        突然闪光造成的“转弯到位误判”
作者：Lu
入口参数:edge: 0 时，为左转检测
        edge : 3 时，为右转检测
返 回 值：1 时，为“真实转向到位”
          0 时，为“转弯误闪”或者并没有转到位
影响参数：
备注：
********************************************************/
int avoidTurnError(int edge){
    int ret = 0;

    if (!Value_Red_ForWard[edge]){
        delay(100);
        Read_RedValue();
        if (!Value_Red_ForWard[edge]){
            // 如果检测到“白” 0.1s后，还是检测到了“白”，说明不是误闪
            ret = 1;
        }
    }

    return ret;
}


/********************************************************
函数功能：顶层函数控制左转
作者：Lu
入口参数：无
返 回 值：无
影响参数：
备注：注意调用此函数时，小车必须中心的红外传感器位于白线上，而非前排的红外传感器
********************************************************/
void Turn_Left(){
    // 待完成...
    stop_slope();
    stop();
    Control(2);     // 再左转
    delay(1000);
    Read_RedValue();


    do{
        Read_RedValue();
    }while(!avoidTurnError(0));

    #if defined(DEBUG)
    Serial.println("Turn left ok !");
    #endif
    // delay(500); // 旋转修正，待车子完全旋转
    Direction = (Direction + 1) % 4;
    
    
    Setpoint_L = Setpoint_R = Value_Setpoint;
    start_slope();
    delay(100);
    time_interval = millis();
}



/********************************************************
函数功能：顶层函数控制右转
作者：Lu
入口参数：无
返 回 值：无
影响参数：
备注：
********************************************************/
void Turn_Right(){
    // 待完成...
    stop_slope();
    stop();
    Control(3);     // 再右转
    delay(1000);
    Read_RedValue();
    
    
    do{
        Read_RedValue();
    }while(!avoidTurnError(3));
    
    
    Direction = (Direction - 1) % 4;
    if (Direction < 0) {
        Direction += 4;
    }
    #if defined(DEBUG)
    Serial.println("Turn right ok !");
    #endif

    Setpoint_L = Setpoint_R = Value_Setpoint;
    start_slope();
    delay(100);     // 防止转弯后误触发
    time_interval = millis();
}



/********************************************************
函数功能：顶层函数控制转180°
作者：Lu
入口参数：无
返 回 值：无
影响参数：
备注：
********************************************************/
void Turn_Around(){
    stop_slope();
    stop();
    Control(2);
    delay(450);
    Read_RedValue();
    do{
        Read_RedValue();
    }while(!avoidTurnError(0));
    Direction = (Direction + 1) % 4;

    delay(380);
    Read_RedValue();
   do{
        Read_RedValue();
    }while(!avoidTurnError(0));
    Direction = (Direction + 1) % 4;
    Setpoint_L = Setpoint_R = Value_Setpoint;
    start_slope();
    delay(100);
    time_interval = millis();
}

/********************************************************
函数功能：向前，前进一格
作者：Lu
入口参数：无
返 回 值：无
影响参数：
备注：
********************************************************/
void Move_Forward_1(){

    stop_slope();
    stop();
    // 车速初始化
    int tag_0 = -1;
    int tag_1 = -1;
    Setpoint_L = Value_Setpoint;
    Setpoint_L = Value_Setpoint;
    V_NOW_L = Value_Setpoint;
    V_NOW_L = Value_Setpoint;
    start_slope();

    start_time = millis();
    Flag_Count = 1;

    // 走出来
    delay(2500);


    while(1){
        Read_RedValue();
        Rectify();

        if( Value_Red_Center[0] || Value_Red_Center[3]){
            Flag_Count = 0;
            tag_0 = 0;
        }


        Read_RedValue();

    if( !Value_Red_Center[0] && !Value_Red_Center[1] && !Value_Red_Center[2] && !Value_Red_Center[3]){
        if(Flag_Count == 0){
            dtime_interval = millis() - time_interval;
            time_interval = millis();

            Flag_Count = 1;
            delay(50); // 再走一会儿

            #if defined(DEBUG)
            Serial.println(tag_0);
            Serial.println(tag_1);
            #endif

            stop();
            stop_slope();   // 已经前进了一格，此时停止

            Now_Point.x = 'B';
            Now_Point.y = '2';

            break;
        }
    }
}
}


/********************************************************
函数功能：车头调整函数
作者：Lu
入口参数：Target，Direction
返 回 值：无
影响参数：Direction
备注：
********************************************************/
void Adjust_carhead(int Target){
    if (Target == Direction){
        // 方向正常
        return;
    }else if (Target - Direction == 2 || Target - Direction == -2) {
        // 转180度
        Turn_Around();
    }
    else if (Target - Direction == 1 || Target - Direction == -3) {
        // 左转90度
        Turn_Left();
    }
    else if (Target - Direction == -1 || Target - Direction == 3) {
        // 右转90度
        Turn_Right();
    }
    else{
    //     Serial.println("Cal_Direction,调整方向异常");
    }

    stop();
    stop_slope();

}


/********************************************************
函数功能：红外纠偏
作者：Lu
入口参数：无
返 回 值：无
影响参数：
备注：
********************************************************/
void Rectify(){
    int bias = 70;
    if(Value_Red_ForWard[0] && Value_Red_ForWard[1] &&//左中出界
            !Value_Red_ForWard[2] && !Value_Red_ForWard[3]){
                Setpoint_R = Value_Setpoint - bias - 10; // 右轮减速
    }
    else if(!Value_Red_ForWard[0] && !Value_Red_ForWard[1] &&//右中出界
            Value_Red_ForWard[2] && Value_Red_ForWard[3]){
                Setpoint_L = Value_Setpoint - bias - 10;
    }
    else if(Value_Red_ForWard[0] && !Value_Red_ForWard[1] && //正常
            !Value_Red_ForWard[2] && Value_Red_ForWard[3]){
                Setpoint_L = Setpoint_R = Value_Setpoint;
    }
    else if(!Value_Red_ForWard[0] && !Value_Red_ForWard[1] && //过线的时候, 正常
            !Value_Red_ForWard[2] && !Value_Red_ForWard[3]){
                Setpoint_L = Setpoint_R = Value_Setpoint;
    }
    else if(Value_Red_ForWard[0] && Value_Red_ForWard[1] && //左小出界
            !Value_Red_ForWard[2] && Value_Red_ForWard[3]){
                 Setpoint_R = Value_Setpoint - bias;
    }
    else if(Value_Red_ForWard[0] && !Value_Red_ForWard[1] && //右小出界
            Value_Red_ForWard[2] && Value_Red_ForWard[3]){
                 Setpoint_L = Value_Setpoint - bias;
    }
    else if(Value_Red_ForWard[0] && Value_Red_ForWard[1] && //左大出界
            Value_Red_ForWard[2] && !Value_Red_ForWard[3]){
                 Setpoint_R = Value_Setpoint - bias-20;
    }
    else if(!Value_Red_ForWard[0] && Value_Red_ForWard[1] && //右大出界
            Value_Red_ForWard[2] && Value_Red_ForWard[3]){
                 Setpoint_L = Value_Setpoint - bias-20;
    }
}


/********************************************************
函数功能：顶层函数，控制小车从A(x1,y1)运动到B(x2,y2)
作者：Lu
入口参数：x1,y1,x2,y2
返 回 值：无
影响参数：
备注：
********************************************************/
void Movement_block(char x1, char y1, char x2, char y2){
    // 待完成...
    if (x1 == x2 && y1 == y2){
        return ;
    }

    Path_Planning(x1,y1,x2,y2);
    Now_Point = Now_Planned_Path.Path_Array[Now_Planned_Path.NowIndex];
    Next_Point = Now_Planned_Path.Path_Array[Now_Planned_Path.NowIndex + 1];
    start_slope();

    int end_while = 0;
    Flag_Count = 1;

    Last_time = millis();


    while(end_while == 0){
        Read_RedValue(); // 读取红外传感器的值
        Exam_Arrival_Point();
        Cal_Direction();
        #if defined(DEBUG)
        Serial.println(Move_State);
        #endif
        
        switch(Move_State){
            case 1:
                Read_RedValue();
                Rectify();
                break;
            case 2:
                // 学长代码写的是， if 只有两个点????
                #if defined(DEBUG_CONNECTED)
                Nowpoint_buf[0] = Now_Point.x;
                Nowpoint_buf[1] = Now_Point.y;
                Nowpoint_buf[2] = '\0';
                Send_DEBUG(Nowpoint_buf);

                Send_DEBUG("Turn Left!");
                #endif
                Turn_Left();
                break;
            case 3:
                // 学长代码写的是， if 只有两个点????
                #if defined(DEBUG_CONNECTED)
                Send_DEBUG("Turn Right!");
                Nowpoint_buf[0] = Now_Point.x;
                Nowpoint_buf[1] = Now_Point.y;
                Nowpoint_buf[2] = '\0';
                Send_DEBUG(Nowpoint_buf);
                #endif

                Turn_Right();
                break;
            case 4:
                Turn_Around();
                break;
            case 5:
                stop_slope();
                Control(4);
                delay(100);
                stop();
                end_while = 1;
                break;
            default:
                // Movement_block()函数异常提醒
                break;
        }
        #if defined(DEBUG)
        Serial.print("Now Point = ");
        Serial.print(Now_Point.x);
        Serial.print(",");
        Serial.println(Now_Point.y);
        Serial.print("Next Point = ");
        Serial.print(Next_Point.x);
        Serial.print(",");
        Serial.println(Next_Point.y);
        Serial.println(Move_State);
        #endif
        test_read_REDandMOTOR();

    }




}


/********************************************************
函数功能：检查到达下一个点没有
作者：Lu
入口参数：Value_Red_Center[],Flag_Count,dtime_interval,time_interval,Now_Point,Next_Point,Now_Planned_Path
返 回 值：
影响参数：Flag_Count,dtime_interval,time_interval,Now_Point,Next_Point,Now_Planned_Path
备注：
********************************************************/
void Exam_Arrival_Point(void){
    if( Value_Red_Center[0] || Value_Red_Center[3]){
        Flag_Count = 0;
    }
    if( !Value_Red_Center[0] && !Value_Red_Center[1] && !Value_Red_Center[2] && !Value_Red_Center[3]){
            if(Flag_Count == 0){
                dtime_interval = millis() - time_interval;
                time_interval = millis();
                if(dtime_interval < 200){
                    return;
                }

                Current_time = millis();
                int L_C_time = Current_time - Last_time;

                // 如果上一个点到这个点的时间小于1s，则跳出
                if (L_C_time <= 200){
                    return ;
                }

                sprintf(Tmp_buf,"Last_time - Current_time = %d",L_C_time);
                Send_DEBUG(Tmp_buf);
                
                Last_time = Current_time;


                Flag_Count = 1;


                Now_Point = Next_Point;

                #if defined(DEBUG_CONNECTED)
                Nowpoint_buf[0] = Now_Point.x;
                Nowpoint_buf[1] = Now_Point.y;
                Nowpoint_buf[2] = '\0';
                Send_DEBUG(Nowpoint_buf);
                #endif

                //   if (!QueneIsEmpty()) 
                if (Now_Planned_Path.NowIndex < Now_Planned_Path.EndIndex)
                {
                    // Next_Point = Dequene();
                    Now_Planned_Path.NowIndex++;
                    Next_Point = Now_Planned_Path.Path_Array[Now_Planned_Path.NowIndex];
                    if((Now_Planned_Path.NowIndex < Now_Planned_Path.EndIndex) && (Now_Point.x == Next_Point.x  &&  Now_Point.y == Next_Point.y)){
                    //   Next_Point = Dequene();
                    Now_Planned_Path.NowIndex++;
                    Next_Point = Now_Planned_Path.Path_Array[Now_Planned_Path.NowIndex];
                    }
              }
              else{
                // 已经读到了最后一个点
                Now_Point = Next_Point;
              }

            }
       }
}


/*-----------------------------------------------------------------------------*/









/*------------------------------------------------------------------------------
文件内容：路径规划模块
最后一次修改时间: 2021.03.02
最后 一次修 改人: 陆琦
备注信息： 路径规划后出现错误信息的错误处理未添加
*/

/********************************************************
函数功能: 路径规划顶层函数
作者：Lu
入口参数：当前点(x1,y1)和目标点(x2,y2)
返 回 值: 
影响参数：Send_buf, Receive_buf, Now_Planned_Path.Now_Planned_Path里将出现新规划好的路径
********************************************************/
void Path_Planning(char x1, char y1, char x2, char y2){
    Clean_buf(Receive_buf,MAX_BUFFER_LENGTH);
    Now_Planned_Path.NowIndex = 0;
    Now_Planned_Path.EndIndex = 0;
    Request_path(x1, y1, x2, y2);
    delay(100);
    Receive();
    Parse_receive_buf();
}




/********************************************************
函数功能: 方向计算程序：根据当前坐标和下一个临接点的坐标计算得到应走的方向
作者：Lu
入口参数：Now_Point, Next_Point
返 回 值: 无
影响参数：Move_State,影响小车下一步该怎么走
********************************************************/
// 标志当前车头朝向
// Direction = 0 -> x轴正向  右
// Direction = 1 -> y轴正向  上
// Direction = 2 -> x轴负向  左
// Direction = 3 -> y轴负向  下

void Cal_Direction(void){
    // 待完成...

    int Target; // 相当于target-Direction，机器人将要移动的方向

    // if (Now_Point.x == Next_Point.x && Now_Point.y == Next_Point.y)
    if (Now_Point.x == Now_Planned_Path.Path_Array[Now_Planned_Path.EndIndex].x && Now_Point.y == Now_Planned_Path.Path_Array[Now_Planned_Path.EndIndex].y)
    {
        Move_State = 5; // 停止
        return ;
    }
    if (Now_Point.x == Next_Point.x - 1 && Now_Point.y == Next_Point.y){
        // 此时要向右移动
        Target = 0;
    }
    else if (Now_Point.x == Next_Point.x + 1 && Now_Point.y == Next_Point.y){
        // 此时要向左移动
        Target = 2;
    }
    else if (Now_Point.x == Next_Point.x && Now_Point.y == Next_Point.y - 1) {
        // 此时要向上移动
        Target = 1;
    }
    else if (Now_Point.x == Next_Point.x && Now_Point.y == Next_Point.y + 1) {
        // 此时要向下移动
        Target = 3;
    }
    else {
        return;
    }

    if (Target == Direction){
        // 保持巡线
        Move_State = 1;
    }else if (Target - Direction == 2 || Target - Direction == -2) {
        // 转180度
        Move_State = 4;
    }
    else if (Target - Direction == 1 || Target - Direction == -3) {
        // 左转90度
        Move_State = 2;
    }
    else if (Target - Direction == -1 || Target - Direction == 3) {
        // 右转90度
        Move_State = 3;
    }
    else{
    //     Serial.println("Cal_Direction,调整方向异常");
    }

}


/*------------------------------------------------------------------------------*/




/*-------------------------------------------------------------------------------
文件内容：识别模块（包括拍照识别和超声波探测）
最后一次修改时间: 2021.03.01
最后 一次修 改人: 陆琦
备注信息： 拍照识别相关内容未添加
*/


/********************************************************
函数功能: 底层函数，超声波检测
作者：Lu
入口参数：超声波模块的触发引脚 -> ultransonic_tr_pin
         超声波模块的回声引脚 -> ultransonic_ec_pin
         判断距离distance,单位mm
返 回 值: 小于distance返回true
影响参数：
********************************************************/
int UL_Detecting(int ultransonic_tr_pin,int ultransonic_ec_pin,int distance){
      unsigned long time_echo_us = 0;
      unsigned long dist_mm = 0;
      unsigned long avg_dist = 0;
      unsigned long last_dist = 0;
      int count = 0;
      #if defined(DEBUG)
      Serial.println("ok1");
      #endif
      int i;
      // 检测5次距离
      for(i = 0;i<5;i++){
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
        else{
            #if defined(DEBUG)
            Serial.println("no1");
            Serial.print("time_echo_us = ");
            Serial.println(time_echo_us);
            #endif
          return 0;
          }
     }
        if(count!=0)
        {
          avg_dist = avg_dist/count;
          }
    #if defined(DEBUG)
    Serial.println(avg_dist);
    #endif

        int ret = 0;
        if( avg_dist < distance && avg_dist != 0){
            ret = 1;
        }   
        else {ret = 0;}

    #if defined(DEBUG)
    Serial.println(ret);
    #endif
    return ret;
}

/********************************************************
函数功能: 顶层函数，超声波检测，调用此函数可以更改Scan_State[]
作者：Lu
入口参数：无
返 回 值: 无
影响参数：Scan_State[UL_NUM]
********************************************************/
void Scan_Echo(void){
  Scan_State[0] = UL_Detecting(UL_trigPin_1, UL_echoPin_1, 100);
  Scan_State[1] = UL_Detecting(UL_trigPin_2, UL_echoPin_2, 100);
}

/********************************************************
函数功能：低速红外纠偏
作者：Lu
入口参数：无
返 回 值：无
影响参数：
备注：
********************************************************/
void Rectify_Low(){
    int bias = 50;
    int Value_Setpoint_Low = 150;
    if(Value_Red_ForWard[0] && Value_Red_ForWard[1] &&//左中出界
            !Value_Red_ForWard[2] && !Value_Red_ForWard[3]){
                Setpoint_R = Value_Setpoint_Low - bias - 1; // 右轮减速W
    }
    else if(!Value_Red_ForWard[0] && !Value_Red_ForWard[1] &&//右中出界
            Value_Red_ForWard[2] && Value_Red_ForWard[3]){
                Setpoint_L = Value_Setpoint_Low - bias - 1;
    }
    else if(Value_Red_ForWard[0] && !Value_Red_ForWard[1] && //正常
            !Value_Red_ForWard[2] && Value_Red_ForWard[3]){
                Setpoint_L = Setpoint_R = Value_Setpoint_Low;
    }
    else if(!Value_Red_ForWard[0] && !Value_Red_ForWard[1] && //过线的时候, 正常
            !Value_Red_ForWard[2] && !Value_Red_ForWard[3]){
                Setpoint_L = Setpoint_R = Value_Setpoint_Low;
    }
    else if(Value_Red_ForWard[0] && Value_Red_ForWard[1] && //左小出界
            !Value_Red_ForWard[2] && Value_Red_ForWard[3]){
                 Setpoint_R = Value_Setpoint_Low - bias;
    }
    else if(Value_Red_ForWard[0] && !Value_Red_ForWard[1] && //右小出界
            Value_Red_ForWard[2] && Value_Red_ForWard[3]){
                 Setpoint_L = Value_Setpoint_Low - bias;
    }
    else if(Value_Red_ForWard[0] && Value_Red_ForWard[1] && //左大出界
            Value_Red_ForWard[2] && !Value_Red_ForWard[3]){
                 Setpoint_R = Value_Setpoint_Low - bias-2;
    }
    else if(!Value_Red_ForWard[0] && Value_Red_ForWard[1] && //右大出界
            Value_Red_ForWard[2] && Value_Red_ForWard[3]){
                 Setpoint_L = Value_Setpoint_Low - bias-2;
    }
}

/********************************************************
函数功能：低速超声波扫描时，用于检测中心位于十字交叉口没有
作者：Lu
入口参数：无
返 回 值： 0时说明中心没到点，1时说明中心到点了
影响参数：
备注：
********************************************************/
// 判断前置红外传感是否从非白线到白线
int judgeToWhite(){
    int lastIswhite = !Value_Red_ForWard_last[0] &&!Value_Red_ForWard_last[1] &&!Value_Red_ForWard_last[2] &&!Value_Red_ForWard_last[3];

    int nowIswhite = !Value_Red_ForWard[0] &&!Value_Red_ForWard[1] &&!Value_Red_ForWard[2] &&!Value_Red_ForWard[3];

    return ( !(lastIswhite) ) && (nowIswhite);
}
int Exam_Arrival_Point_Low(void){

    if (judgeToWhite()){
        flag_white_pass = 1;
    }
    
    if( Value_Red_Center[0] || Value_Red_Center[3]){
        Flag_Count = 0;
    }

    if( !Value_Red_Center[0] && !Value_Red_Center[1] && !Value_Red_Center[2] && !Value_Red_Center[3]){
            if(Flag_Count == 0 && flag_white_pass == 1){
                dtime_interval = millis() - time_interval;
                time_interval = millis();
                // if(dtime_interval < 100){
                //     return 0;   // 没到点
                // }
                Flag_Count = 1;
                return 1;   // 到点了
            }
       }
    return 0;   // 没到点
}

/********************************************************
函数功能：防止低速巡线扫描的时候误扫
作者：Lu
入口参数:pos: 0 时，为下货窗检测
        pos: 1 时，为上货窗检测
返 回 值：1 时，为“真实扫到物品”
          0 时，为“错误扫到物品”
影响参数：
备注：
********************************************************/
int avoidScanError(int pos){
    int ret = 0;

    if (Scan_State[pos]){
        // Serial.println(Scan_State[0]);
        delay(10);
        Scan_Echo();
        if (Scan_State[pos]){
            // 如果检测到有物品 0.01s后，还是检测到了物品，说明不是误扫
            ret = 1;
        }
    }
    return ret;
}

/********************************************************
函数功能: 顶层函数，沿上货窗扫描并抓取
作者：Lu
入口参数：Buy_shelf
返 回 值: 无
影响参数：
********************************************************/
void Catchitem_Move_High(){

    // 为了防止扫描时爪子卡住
    armInit();



    start_slope();

    // 先运动到前方白线
    int end_while_0 = 0;
    while(end_while_0 == 0){
        Read_RedValue();
            if (!Value_Red_ForWard[0] && !Value_Red_ForWard[1] && 
            !Value_Red_ForWard[2] && !Value_Red_ForWard[3]) // 当前面四个灯都亮
            {
                end_while_0 = 1;
                Rectify();
                delay(100); //再往前走一段路
            }
            else{
                Rectify();
            }
    }


    stop();
    stop_slope();
    // 调整机械臂至适合上方扫描
    armInit();

    delay(500);     // 停一下



    Read_RedValue();
    Setpoint_L = 150;
    Setpoint_R = 150;
    V_NOW_L = 150;
    V_NOW_R = 150;
    start_slope();
    flag_white_pass = 0;

    if (Buy_shelf == 'D' || Buy_shelf == 'C'){
        do{
            // 向前低速巡线
            Store_ForwardRedValue();
            Read_RedValue();
            Rectify_Low();
            Exam_Arrival_Point_Low();

        }while(!UL_Detecting(UL_trigPin_2,UL_echoPin_2,120));

    }
    else if(Buy_shelf == 'A' || Buy_shelf == 'B'){
        do{
            // 向前低速巡线
            Store_ForwardRedValue();
            Read_RedValue();
            Rectify_Low();
            Exam_Arrival_Point_Low();

        }while(!UL_Detecting(UL_trigPin_2,UL_echoPin_2,360));

    }

    // 扫到物品后继续走300ms，直到正中间
    delay(150);

    // 扫到物品后停下
    stop();
    stop_slope();

    // 抓取
    scratchAbove();    

    // 向前继续巡到下一个白线, 直至中心位于前方的十字交叉点
    int end_while_1 = 0;
    Flag_Count = 1;
    Setpoint_L = 150;
    Setpoint_R = 150;
    V_NOW_L = 150;
    V_NOW_R = 150;
    start_slope();

    while(end_while_1 == 0){
        Store_ForwardRedValue();
        Read_RedValue();

        end_while_1 = Exam_Arrival_Point_Low();

        Rectify_Low();
    }

    // 更新位置
    switch(Buy_shelf){
        case 'D' :
                    Now_Point.x = Now_Point.x + 2;
                    break;
        case 'C' :
                    Now_Point.y = Now_Point.y + 2;
                    break;
        case 'B' :
                    Now_Point.x = Now_Point.x - 2;
                    break;
        case 'A' :
                    Now_Point.y = Now_Point.y - 2;
                    break;
    }

}

/********************************************************
函数功能: 顶层函数，沿下货窗扫描并抓取
作者：Lu
入口参数：Buy_shelf
返 回 值: 无
影响参数：
********************************************************/
void Catchitem_Move_Low(){
    
    armInit();

    start_slope();

    // 先运动到前方白线
    int end_while_0 = 0;
    while(end_while_0 == 0){
        Read_RedValue();
            if (!Value_Red_ForWard[0] && !Value_Red_ForWard[1] && 
            !Value_Red_ForWard[2] && !Value_Red_ForWard[3]) // 当前面四个灯都亮
            {
                end_while_0 = 1;
                Rectify();
                delay(100); //再往前走一段路
            }
            else{
                Rectify();

            }
    }

    stop();
    stop_slope();
    delay(500);     // 停一下


    Read_RedValue();
    Setpoint_L = 150;
    Setpoint_R = 150;
    V_NOW_L = 150;
    V_NOW_R = 150;
    start_slope();
    flag_white_pass = 0;

        do{
            // 向前低速巡线
            Store_ForwardRedValue();
            Read_RedValue();
            Rectify_Low();
            Exam_Arrival_Point_Low();
        }while(!UL_Detecting(UL_trigPin_1,UL_echoPin_1,120));

    
    // 扫到物品后继续走300ms，直到正中间
    if (Buy_shelf == 'A' || Buy_shelf == 'B'){
        delay(150);
    }
    else if(Buy_shelf == 'C' || Buy_shelf == 'D'){
        delay(100);
    }

    // 扫到物品后停下
    stop();
    stop_slope();

    // 抓取
    #if defined(DEBUG_CONNECTED)
    Send_DEBUG("Catch!");
    #endif

    // scratchUnder();
    
    if (Buy_shelf == 'A' || Buy_shelf == 'B'){
        // A,B货架，抓大物品
        scratchUnder_big();
    }
    else if(Buy_shelf == 'C' || Buy_shelf == 'D'){
        // C,D货架，抓小物品
        scratchUnder_small();
    }


    // 向前继续巡到下一个白线, 直至中心位于前方的十字交叉点
    int end_while_1 = 0;
    Flag_Count = 1;
    Setpoint_L = 150;
    Setpoint_R = 150;
    V_NOW_L = 150;
    V_NOW_R = 150;
    start_slope();

    while(end_while_1 == 0){
        Store_ForwardRedValue();
        Read_RedValue();

        end_while_1 = Exam_Arrival_Point_Low();

        Rectify_Low();
    }

    // 更新位置
    switch(Buy_shelf){
        case 'D' :
                    Now_Point.x = Now_Point.x + 2;
                    break;
        case 'C' :
                    Now_Point.y = Now_Point.y + 2;
                    break;
        case 'B' :
                    Now_Point.x = Now_Point.x - 2;
                    break;
        case 'A' :
                    Now_Point.y = Now_Point.y - 2;
                    break;
    }
    stop();
    stop_slope();

}


/********************************************************
    根据shelf_id遍历Shelf_State_List的不同行，
    1. 先找到第一个isChecked == 1, num != 0的点
    2. 再找到第一个isChekced == 0的点
    3. 如果该shelf_id的所有的货窗都已经被checked则Traversal_Return_line = -1,Traversal_Return_column = -1, 进而在主循环中更改全局变量Buy_Shelf--
********************************************************/

/********************************************************
函数功能: 根据shelf_id遍历Shelf_State_List的不同行
        1. 先找到第一个isChecked == 1, num != 0的点
        2. 再找到第一个isChekced == 0的点
        3. 如果该shelf_id的所有的货窗都已经被checked,则Traversal_Return_line = -1,Traversal_Return_column = -1, 进而在主循环中更改全局变量Buy_Shelf--
作者：Lu
入口参数：shelf_id, Shelf_State_List[][]
返 回 值: 无
影响参数：Traversal_Return_line, Traversal_Return_column
********************************************************/
void traversal_Shelf_State_List(char shelf_id){
    int line = 0;
    int column = 0;
    switch (shelf_id)
        {
        case 'D':
            line = 0;
            break;
        case 'C':
            line = 1;
            break;
        case 'B':
            line = 2;
            break;
        case 'A':
            line = 3;
            break; 
        }

    // 先找到第一个isChecked == 1, num != 0的点
    for(column=0;column<5;column++){
        if (Shelf_State_List[line][column].isChecked == 1 && (Shelf_State_List[line][column].num[0] != 0 || Shelf_State_List[line][column].num[1] != 0)){
            Traversal_Return_line = line;
            Traversal_Return_column = column;
            return ;
        }
    }

    // 再找到第一个isChekced == 0的点
    for(column=0;column<5;column++){
        if (Shelf_State_List[line][column].isChecked == 0){
            Traversal_Return_line = line;
            Traversal_Return_column = column;
            return ;
        }
    }

    // 如果该shelf_id的所有的货窗都已经被checked,则Traversal_Return_line = -1,Traversal_Return_column = -1, 进而在主循环中更改全局变量Buy_Shelf--

    Traversal_Return_column = -1;
    Traversal_Return_line   = -1;
}


/********************************************************
函数功能: 移动到货窗前抓取，并返回起点放置
作者：Lu
入口参数：Traversal_Return_line, Traversal_Return_column, Buy_Shelf
返 回 值: 无
影响参数：Shelf_State_List[][]
********************************************************/
void Move_Catch_And_Back(){
    // 查询到目标货窗正对着的点
    char shelf_x = Shelf_State_List[Traversal_Return_line][Traversal_Return_column].x;
    char shelf_y = Shelf_State_List[Traversal_Return_line][Traversal_Return_column].y;



    // 移动到货窗前，调整车头朝向，准备抓取
    switch(Buy_shelf){
        case 'D':
            Movement_block(Now_Point.x, Now_Point.y, shelf_x - 1, shelf_y);
            #if defined(DEBUG_CONNECTED)
            Send_DEBUG("adjust for photo!");        
            #endif
            Adjust_carhead(0);
            // Move_Forward_1();
            break;
        case 'C':
            Movement_block(Now_Point.x, Now_Point.y, shelf_x, shelf_y - 1);
            #if defined(DEBUG_CONNECTED)
            Send_DEBUG("adjust for photo!");        
            #endif
            Adjust_carhead(1);
            // Move_Forward_1();
            break;
        case 'B':
            Movement_block(Now_Point.x, Now_Point.y, shelf_x + 1, shelf_y);
            #if defined(DEBUG_CONNECTED)
            Send_DEBUG("adjust for photo!");        
            #endif
            Adjust_carhead(2);
            // Move_Forward_1();
            break;
        case 'A':
            Movement_block(Now_Point.x, Now_Point.y, shelf_x, shelf_y + 1);
            #if defined(DEBUG_CONNECTED)
            Send_DEBUG("adjust for photo!");        
            #endif            
            Adjust_carhead(3);
            // Move_Forward_1();
            break;
    }


    // 看是下货窗有物品还是上货窗有物品
    // 调用Catchitem_Move_Low()或Catchitem_Move_High()抓取物品
    if (Shelf_State_List[Traversal_Return_line][Traversal_Return_column].num[0] != 0){
        // 调用Catchitem_Move_Low()
        armInit();
        scrach_angle(myarm_scratch,-60);
        Catchitem_Move_Low();
        // 更新Shelf_State_List[Traversal_Return_line][Traversal_Return_column].num[0]
        Shelf_State_List[Traversal_Return_line][Traversal_Return_column].num[0]--;
    }
    else if(Shelf_State_List[Traversal_Return_line][Traversal_Return_column].num[1] != 0){
        // 调用Catchitem_Move_High()
        armInit();
        scrach_angle(myarm_scratch,-60);
        Catchitem_Move_High();
        // 更新Shelf_State_List[Traversal_Return_line][Traversal_Return_column].num[1]
        Shelf_State_List[Traversal_Return_line][Traversal_Return_column].num[1]--;
    }

    // 返回起点，调整车头朝向
    Clean_buf(Receive_buf,MAX_BUFFER_LENGTH);
    // Movement_block(Now_Point.x, Now_Point.y, 'A', '2');

    // 为了防止起点区的红色造成的误判，先到A3,再到A2    
    Movement_block(Now_Point.x, Now_Point.y, 'A', '4');
    Movement_block(Now_Point.x, Now_Point.y, 'A', '2');

    Adjust_carhead(3);
    // Move_Forward_1();

    // 放下货品
    armInit();
    changePositionSlow(myarm_0,120);
    scrach_angle(myarm_scratch,-60);    // 松开
    scrach_angle(myarm_scratch, 30);
    armInit();

}

/********************************************************
函数功能: 货窗扫描主循环
作者：Lu
入口参数：Buy_shelf
返 回 值: 无
影响参数：Shelf_State_List[][]
********************************************************/
void ShelfCatch_LOOP(char shelf_id){
    /*
    根据shelf_id遍历Shelf_State_List的不同行，
    1. 先找到第一个isChecked == 1, num != 0的点
    2. 再找到第一个isChekced == 0的点
    3. 如果该shelf_id的所有的货窗都已经被checked且抓完了则更改全局变量Buy_Shelf--
     */

    traversal_Shelf_State_List(shelf_id);
    int line   = Traversal_Return_line;
    int column = Traversal_Return_column;

    if (line != -1 && column != -1){
        // 如果找到下一个具体的点，则找到的点
        sprintf(Tmp_buf,"(x,y)=(%c,%c),num = [%d,%d]",Shelf_State_List[line][column].x,Shelf_State_List[line][column].y,Shelf_State_List[line][column].num[0],Shelf_State_List[line][column].num[1]);
        Send_DEBUG(Tmp_buf);
    }



    // 如果shelf_id的所有货窗都被checked抓完了，则改变Buy_Shelf
    if (line == -1 && column == -1){
        Buy_shelf --;
        return ;
    }

    // 如果是找到了第一个isChecked == 1, num != 0的点
    if (Shelf_State_List[line][column].isChecked == 1 && (Shelf_State_List[line][column].num[0] != 0 || Shelf_State_List[line][column].num[1] != 0)){
        // 根据Traversal_Return_line和Traversal_Return_column 查到目标位置然后抓取

        // 移动到货窗前，调整车头朝向，准备抓取

        // 看是下货窗有物品还是上货窗有物品

        // 调用Catchitem_Move_Low()或Catchitem_Move_High()抓取物品
        // 更新Shelf_State_List[line][column].num = {?,?}

        // 返回起点，调整车头朝向

        // 放下货品
        Move_Catch_And_Back();



        return ;
    }

    // 如果所有被checked的货窗，num[] = {0,0}, 则此时的line和column指向第一个未被checked的点
    if (Shelf_State_List[line][column].isChecked == 0){
        char shelf_x = Shelf_State_List[line][column].x;
        char shelf_y = Shelf_State_List[line][column].y;

        // 到货窗前拍照，调整车头拍照
        switch(Buy_shelf){
            case 'D':
                Movement_block(Now_Point.x, Now_Point.y, shelf_x, shelf_y+1);
                Adjust_carhead(X_POS);
                break;
            case 'C':
                Movement_block(Now_Point.x, Now_Point.y, shelf_x-1, shelf_y);
                Adjust_carhead(Y_POS);
                break;
            case 'B':
                Movement_block(Now_Point.x, Now_Point.y, shelf_x, shelf_y-1);
                Adjust_carhead(X_NEG);
                break;
            case 'A':
                Movement_block(Now_Point.x, Now_Point.y, shelf_x+1, shelf_y);
                Adjust_carhead(Y_NEG);
                break;
        }

        armForPhoto();  // 拍照前调整摄像头方向
        Request_recog();
        Receive();
        
        // 更新Shelf_State_List[line][column].isChecked和num[0],num[1]
        Parse_receive_buf();


        // 重新调用ShelfCatch_LOOP(shelf_id)函数

        return ;
    }

}


/*---------------------------------------------------------------------------------*/


void setup() {
  // put your setup code here, to run once:
    FlexiTimer2::set(10,Move_ForwardSlope);

    pinMode(Red_Forward_0,INPUT);
    pinMode(Red_Forward_1,INPUT);
    pinMode(Red_Forward_2,INPUT);
    pinMode(Red_Forward_3,INPUT);
    pinMode(Red_Center_0,INPUT);
    pinMode(Red_Center_1,INPUT);
    pinMode(Red_Center_2,INPUT);
    pinMode(Red_Center_3,INPUT);
    Serial.begin(9600);
    pinMode(PWMA,OUTPUT);
    pinMode(PWMB,OUTPUT);
    pinMode(IN1,OUTPUT);
    pinMode(IN2,OUTPUT);
    pinMode(IN3,OUTPUT);
    pinMode(IN4,OUTPUT);

    pinMode(UL_trigPin_1,OUTPUT);
    pinMode(UL_echoPin_1,INPUT);
    pinMode(UL_trigPin_2,OUTPUT);
    pinMode(UL_echoPin_2,INPUT);


    // 机械臂和机械爪初始化
    myarm_0.attach(MYARM_0_PIN);
    myarm_1.attach(MYARM_1_PIN);
    myarm_2.attach(MYARM_2_PIN);
    myarm_scratch.attach(MYARM_SCRATCH_PIN);
    armInit();
    scrach_angle(myarm_scratch,-90);    // 长到最大


    // 位置初始化
    Now_Point.x = 'A';
    Now_Point.y = '2';

    Buy_shelf = 'D';

}

int Master_OK = 0;

void loop() {

    while(!Master_OK){
        if (Serial.available()){
            delay(100);
            Receive();
            if(Receive_buf[0] == 's'){
                Master_OK = 1;
                Send_DEBUG("connected success...");
                Move_Forward_1();   // 向前一格，让小车从起点区前进到(A,2)
                stop();
                Send_DEBUG("Let's Start!");
            }
        }
    }

    if (Master_OK == 1){

        // 测试能否根据Shelf_State_List[][]循环抓取
        // if (Shelf_State_List[3][3].num[0] == 0 && Shelf_State_List[3][3].num[1] == 0){
        //     stop();
        // }
        // else{
        //     ShelfCatch_LOOP(Buy_shelf);
        // }

        ShelfCatch_LOOP(Buy_shelf);

    }


}
