#include "mbed.h"
#include "library/scrp_master.hpp"
#include "library/rotary_inc.hpp"
#include "ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#define MAXPWM 250
#define Period 256

ScrpMaster master(PA_9,PA_10,PA_12);

constexpr PinName pwmpin[3] = {PB_6 ,PA_11,PB_7};

PwmOut servo1(PB_6);//ロック
PwmOut servo2(PA_11);

DigitalIn slit_up1(PA_3);//昇降
DigitalIn slit_up2(PA_1);

DigitalIn slit_towel1(PA_4);//バスタオル
DigitalIn slit_towel2(PA_0);

DigitalIn limit_clip(PB_1);//洗濯バサミ
AnalogIn potentiometer(PA_6);//ポテンショメータ
InterruptIn start_switch(PB_0);//スタート

//DigitalOut led1(PB_7);

int spread = 0;//0:展開前　1:第一展開　2:第二展開
int spread_goal = 0;
bool towel_arm_goal = false;//1:展開　2:戻す
bool sheet_open = false;
bool lock[2] = {false,false};

//const PinName rotarypin[2] = {PA_0,PA_4};

//RotaryInc rotary(rotarypin[0],rotarypin[1],10,200,0);
uint8_t flag;

bool solenoid(int data){
    if(data == 0){
        flag = 0;
        DigitalOut Moter1(pwmpin[0],0);
        DigitalOut Moter2(pwmpin[1],0);
    }else if(data == 1){
        flag |= 0x01;
        DigitalOut Moter1(pwmpin[0],1);
    }else if(data == 2){
        flag |= 0x02;
        DigitalOut Moter2(pwmpin[1],1);
    }else if(data == -1){
        flag &= 0xfe;
        DigitalOut Moter1(pwmpin[0],0);
    }else if(data == -2){
        flag &= 0xfd;
        DigitalOut Moter2(pwmpin[1],0);
    }
    if(flag == 0){
        DigitalOut Led(pwmpin[2],0);
    }else{
        DigitalOut Led(pwmpin[2],1);
    }
    return true;
}

void safe(){/*
	for(int i = 0;i < 5;i++){
    	DigitalOut Moter1(pwmpin[i][0],0);
        DigitalOut Moter2(pwmpin[i][1],0);
        DigitalOut Led(pwmpin[i][2],0);
    }*/
}

void sendSerial(const std_msgs::Int32 &msg){
    uint8_t id = (msg.data >> 24) & 0xff;
    uint8_t cmd = (msg.data >> 16) & 0xff;
    int16_t data = (msg.data) & 0xffff;
    if(id == 1 || id == 255){
        switch (cmd){
            case 1:
                if(data == 1){
                    servo1.pulsewidth_us(1450);//ロック
                    lock[0] = true;
                }else if(data == 2){
                    servo2.pulsewidth_us(1450);
                    lock[1] = true;
                }else if(data == -1){
                    servo1.pulsewidth_us(2350);//ロック解除
                    lock[0] = false;
                }else if(data == -2){
                    servo2.pulsewidth_us(2350);
                    lock[1] = false;
                }
                break;
            case 2:
                towel_arm_goal = (bool)data;
                break;
            case 3:
                spread_goal = data;
                break;
            case 4:
                sheet_open = (bool)data;
                break;
            case 5:
                solenoid(data);
                break;
        }
    }else{
        master.send(id,cmd,data);
    }
}

std_msgs::Int32 status;
std_msgs::Float32MultiArray pot;
std_msgs::Bool limit_msg,start_msg;
ros::Subscriber<std_msgs::Int32> mdd("Motor_Serial",&sendSerial);
ros::Publisher pub("Mechanism_Status",&status);
ros::Publisher potato("potato",&pot);
ros::Publisher limit_pub("Clip_Limit",&limit_msg);
ros::Publisher start("start_switch",&start_msg);

void start_fall(){
	start_msg.data = start_switch.read();
	start.publish(&start_msg);
}

int main(){
    ros::NodeHandle nh;
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(mdd);
    nh.advertise(pub);
    nh.advertise(potato);
    nh.advertise(limit_pub);
    nh.advertise(start);
    start_switch.mode(PullUp);
    start_switch.fall(&start_fall);
    pot.data_length = 3;
    pot.data = (float*)malloc(sizeof(float)*pot.data_length);
    safe();
    servo1.period_ms(20);
    servo2.period_ms(20);
    servo1.pulsewidth_us(1450);
    servo2.pulsewidth_us(1450);
    bool on_flag[2] = {false,false};
    bool off_flag[2] = {false,false};
    bool towel_arm[2] = {false,false};
    bool moving = false;
    int count = 0;
    std_msgs::Int32 msg;
    uint8_t slit_up;
    uint8_t status = 0;
    Timer loop;
    loop.start();
    while(true){
        nh.spinOnce();
        if(loop.read_ms() > 20){
            loop.reset();
            if(count  == 0){
                if(towel_arm[0] != towel_arm_goal){//右側
                    if(!(status & 0b00000001)){
                        status |= 0b00000001;
                    }
                    if(on_flag[0]){
                        if(slit_towel1.read()){
                            master.send(15,2,50 * (towel_arm_goal == 1 ? -1 : 1));
                        }else{
                            master.send(15,2,0);
                            towel_arm[0] = towel_arm_goal;
                            on_flag[0] = false;
                            status &= 0b11111110;
                        }
                    }else{
                        if(slit_towel1.read() && off_flag[0]){
                            on_flag[0] = true;
                        }else if(!slit_towel1.read()){
                            off_flag[0] = true;
                        }
                        master.send(15,2,50 * (towel_arm_goal == 1 ? 1 : -1));
                    }
                }
            }else if(count == 1){
                if(towel_arm[1] != towel_arm_goal){//左側
                    if(!(status & 0b00000010)){
                        status |= 0b00000010;
                    }
                    if(on_flag[1]){
                        if(slit_towel2.read()){
                            master.send(15,3,50 * (towel_arm_goal == 1 ? 1 : -1));
                        }else{
                            master.send(15,3,0);
                            towel_arm[1] = towel_arm_goal;
                            on_flag[1] = false;
                            status &= 0b11111101;
                        }
                    }else{
                        if(slit_towel2.read()){
                            on_flag[1] = true;
                        }else{
                            master.send(15,3,50 * (towel_arm_goal == 1 ? -1 : 1));
                        }
                    }
                }
            }else if(count == 2){
                slit_up = slit_up1 << 1 + slit_up2;//上　下
                if(spread != spread_goal){
                    if(!(status & 0b00000100)){
                        status |= 0b00000100;
                    }
                    if(spread_goal == 0){
                        if(moving){
                            if(slit_up == 3){
                                master.send(15,4,0);
                                spread = 0;
                                moving = false;
                                status &= 0b11111011;
                            }
                        }else{
                            if(lock[0] || lock[1]){
                                servo1.pulsewidth_us(2350);//ロック解除
                                servo2.pulsewidth_us(2350);
                                wait(1);
                                lock[0] = false;
                                lock[1] = false;
                            }
                            if(slit_up == 1){
                                moving = true;
                            }
                            master.send(15,4,100);
                        }
                    }else if(spread_goal == 1){
                        if(moving){
                            if(slit_up == 3){
                                master.send(15,4,0);
                                spread = 1;
                                moving = false;
                                status &= 0b11111011;
                            }
                        }else{
                            if(!lock[0] || lock[1]){
                                servo1.pulsewidth_us(1450);//サーボ1個目（上に上がる方）
                                servo2.pulsewidth_us(2350);//サーボ2個め（下に残る方）
                                wait(1);
                                lock[0] = true;
                                lock[1] = false;
                            }
                            if(slit_up == 3 && spread == 0){
                                master.send(15,4,-250);//上昇
                            }else if(slit_up == 3 && spread == 2){
                                master.send(15,4,100);//下降
                            }else if(slit_up == 1){
                                master.send(15,4,-250);//上昇
                                moving = true;
                            }else if(slit_up == 2){
                                master.send(15,4,100);//下降
                                moving = true;
                            }
                        }
                    }else if(spread_goal == 2){
                        if(moving){
                            if(slit_up == 3){
                                master.send(15,4,0);
                                spread = 2;
                                moving = false;
                                status &= 0b11111011;
                            }
                        }else{
                            if(!lock[0] || !lock[1]){
                                servo1.pulsewidth_us(1450);
                                servo2.pulsewidth_us(1450);
                                wait(1);
                                lock[0] = true;
                                lock[1] = true;
                            }
                            if(slit_up == 2){
                                moving = true;
                            }
                            master.send(15,4,-250);
                        }
                    }
                }else if(slit_up == 3){
                    master.send(15,4,0);
                }else if(spread > 0){
                    master.send(15,4,-250);
                }
            }else if(count == 4){
                if(potentiometer > 0.99 || potentiometer < 0.01){//0.0252 ~ 0.042
                }else if(sheet_open){
                    if(potentiometer < 0.042){
                        master.send(15,5,50);
                        if(!(status & 0b00001000)){
                            status |= 0b00001000;
                        }
                    }else{
                        master.send(15,5,0);
                        if(status & 0b00001000){
                            status &= 0b11110111;
                        }
                    }
                }else{
                    if(potentiometer > 0.025){
                        master.send(15,5,-50);
                        if(!(status & 0b00001000)){
                            status |= 0b00001000;
                        }
                    }else{
                        master.send(15,5,0);
                        if(status & 0b00001000){
                            status &= 0b11110111;
                        }
                    }
                }
                count = -1;
            }
            count ++;
            limit_msg.data = limit_clip;
            limit_pub.publish(&limit_msg);

            msg.data = status;
            pub.publish(&msg);

            pot.data[0] = slit_towel1;
            pot.data[1] = slit_towel2;
            pot.data[2] = potentiometer;
            potato.publish(&pot);
        }
    }
}
