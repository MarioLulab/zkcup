import serial
import time
import BFS_plan
from model import MODEL
import cv2
from PIL import Image


#--------------------------------------------
# 文件内容：上位机通讯模块
# 最后一次修改时间: 2021.05.07
# 最后 一次修 改人: 陆琦
# 备注信息： 加入了神经网络模块   
#--------------------------------------------


# asii码 -> 字符 : chr
# 字符   -> asii码 : ord
# 比如：
# chr(32)  -> ' '
# ord(' ') -> 32



# 串口接收数据检验函数
# 接收参数 : ReceiveData
# 返回参数 : 数据正确->返回接收数据长度，不包括句末分号
#           数据错误->返回-1
def check_receive_data():
    global ReceiveData
    receive_length = len(ReceiveData)
    data_length = ord(ReceiveData[1])
    if (receive_length == data_length):
        return receive_length
    else :
        return -1


# 处理串口接收到的数据
# 接收参数 : ReceiveData
# 返回参数 : 无
def do_request():
    if (ReceiveData[0] == 'P'):
        # 执行路径规划算法
        start_point = ReceiveData[2] + ReceiveData[3]
        finishing_point = ReceiveData[4] + ReceiveData[5]
        path = BFS_plan.calculate_BFS(BFS_plan.map, start_point, finishing_point)

        # 封装发送帧
        WriteData = "" # clear
        WriteData += 'P'
        WriteData += chr( ( len(path)*2 + 2 ) )
        for point in path:
            WriteData += point
        WriteData += ';'
        print("Send---->" + WriteData + " length=" + str(len(WriteData)))
        ser.write(WriteData.encode("gbk"))


    if (ReceiveData[0] == 'R'):
        # 调用摄像头拍照
        global photo_index
        global mymodel

        ret, frame = cap.read()

        # 裁剪照片
        cropImg = frame[50:,120:500]
        
        # 保存照片
        print("*"*60)
        print("this is " + str(photo_index) + " frame !")
        print("*"*60)

        save_path = "running_photo/" + str(photo_index) + ".jpg"
        cv2.imwrite(save_path,cropImg)

        
        image = Image.open(save_path)

        # cv2.imwrite(save_path + "luqi" + ".jpg",cropImg)

        photo_index += 1

        # 使用神经网络进行识别
        result = mymodel.predict(image)

        # 测试封装发送帧
        WriteData = "" # clear
        WriteData += 'R'
        WriteData += chr( 4 )
        WriteData += str(result[0]) # 上层货架目标货物数
        WriteData += str(result[1]) # 下层货架目标货物数
        WriteData += ';'
        print("Send---->" + WriteData + " length=" + str(len(WriteData)))
        ser.write(WriteData.encode("gbk"))
        # pass


# 使串口接收到数据
# 接收参数 : ReceiveData
# 返回参数 : 无
def receive():
    global ReceiveData
    while(1):
        tmp = str(ser.read(1), encoding = "gbk")
        if (tmp == ';'):
            print("Receive---->"+ ReceiveData + " ," +" length=" + str(len(ReceiveData)))
            break
        else:
            ReceiveData += tmp
            # print(ReceiveData)

# 使树莓派唤醒arduino
# 接收参数 : 无
# 返回参数 : 无
def connect():
    global WriteData

    ser.write(b's') # 激活
    time.sleep(1) # 等待唤醒

    # 正式发送, 通知arduino开始工作！
    WriteData = "" # clear
    WriteData += 's'
    WriteData += chr( 2 )
    WriteData += ';'
    ser.write(WriteData.encode("gbk"))
    print("Send---->" + WriteData + " length=" + str(len(WriteData)))
    


if __name__ == "__main__":
    since = time.time()


    # 串口通讯参数设置
    ser = serial.Serial()
    ser.baudrate = 9600
    # ser.port = "COM3"
    # for raspberry pi -> ser.port = "/dev/xxx"
    ser.port = "/dev/ttyACM0"

    # 数据接收和发送缓冲
    ReceiveData = ""
    WriteData = ""
    ExitHeader = 'E'

    # 识别拍照的照片序号
    photo_index = 0


    # 停止标志
    Stop = False

    # 生成神经网络模型
    mymodel = MODEL()
    print("*"*60)
    print("正在生成神经网络模型...")
    mymodel.generate()


    print("*"*60)
    print("预热模型")
    image = Image.open("running_photo/warm.jpg")
    tmp_reuslt = mymodel.predict(image)
    print("模型预热结束")
    print("*"*60)
        
    if (ser.is_open == False):
        ser.open()

    if (ser.is_open == False):
        # recheck
        print("Open Serial Failed!")
    else:
        # serial is open!
        print("Serial Parameters : ", ser)
        
        a = input() # wait for connecting
        
        # 打开摄像头
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.7)

        connect()

        while (Stop == False):

            # 因为 cv2 读取的是视频流，故应该在循环中不停读取frame
            ret, frame = cap.read()
            
            if ser.in_waiting != 0:
                # receive data
                ReceiveData = "" # clear
                receive()

                if (check_receive_data() != -1):
                    if (ReceiveData[0] == ExitHeader):
                        Stop = True
                    if (ReceiveData[0] == 'P' or ReceiveData[0] == 'R'):
                        do_request()
                
    print("-----"*20+"Run Complete"+"-----"*20)
    end = time.time()
    print("Total run time: {}".format(end-since))
    print("Serial Parameters : ", ser)
