#include "mbed.h"
#include "library/scrp_master.hpp"
#include "library/rotary_inc.hpp"
#include "ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#define MAXPWM 250
#define Period 256

ScrpMaster master(PA_9, PA_10, PA_12);

constexpr PinName pwmpin[3] = { PA_8, PA_7, PB_5 };

PwmOut servo1(PB_6); //ロック
PwmOut servo2(PA_11);

DigitalIn slit_up1(PA_3); //昇降
DigitalIn slit_up2(PA_1);

DigitalIn slit_towel1(PA_4); //バスタオル
DigitalIn slit_towel2(PA_0);

DigitalIn limit_clip(PB_1); //洗濯バサミ
AnalogIn potentiometer(PA_6); //ポテンショメータ
InterruptIn start_switch(PB_0); //スタート

//DigitalOut led1(PB_7);

int8_t spread = 0; //0:展開前　1:第一展開　2:第二展開
int8_t spread_goal = 0;
bool towel_arm_goal = false; //1:展開　2:戻す
bool sheet_open = false;
bool lock[2] = { false, false };
bool towel_order[2] = { false, false };
bool open_order = false;

//const PinName rotarypin[2] = {PA_0,PA_4};

//RotaryInc rotary(rotarypin[0],rotarypin[1],10,200,0);
uint8_t flag;

bool solenoid(int data) {
	if (data == 0) {
		flag = 0;
		DigitalOut Moter1(pwmpin[0], 0);
		DigitalOut Moter2(pwmpin[1], 0);
	} else if (data == 1) {
		flag |= 0x01;
		DigitalOut Moter1(pwmpin[0], 1);
	} else if (data == 2) {
		flag |= 0x02;
		DigitalOut Moter2(pwmpin[1], 1);
	} else if (data == -1) {
		flag &= 0xfe;
		DigitalOut Moter1(pwmpin[0], 0);
	} else if (data == -2) {
		flag &= 0xfd;
		DigitalOut Moter2(pwmpin[1], 0);
	}
	if (flag == 0) {
		DigitalOut Led(pwmpin[2], 0);
	} else {
		DigitalOut Led(pwmpin[2], 1);
	}
	return true;
}

void safe() {/*
 for(int i = 0;i < 5;i++){
 DigitalOut Moter1(pwmpin[i][0],0);
 DigitalOut Moter2(pwmpin[i][1],0);
 DigitalOut Led(pwmpin[i][2],0);
 }*/
}

void sendSerial(const std_msgs::Int32 &msg) {
	uint8_t id = (msg.data >> 24) & 0xff;
	uint8_t cmd = (msg.data >> 16) & 0xff;
	int16_t data = (msg.data) & 0xffff;
	if (id == 1 || id == 255) {
		switch (cmd) {
		case 1:
			if (data == 1) {
				servo1.pulsewidth_us(1450); //ロック
				lock[0] = true;
			} else if (data == 2) {
				servo2.pulsewidth_us(1450);
				lock[1] = true;
			} else if (data == -1) {
				servo1.pulsewidth_us(2300); //ロック解除
				lock[0] = false;
			} else if (data == -2) {
				servo2.pulsewidth_us(2300);
				lock[1] = false;
			}
			break;
		case 2:
			towel_arm_goal = (bool) data;
			towel_order[0] = true;
			towel_order[1] = true;
			break;
		case 3:
			spread_goal = data;
			break;
		case 4:
			sheet_open = (bool) data;
			open_order = true;
			break;
		case 5:
			solenoid(data);
			break;
		case 6:
			spread = data;
			spread_goal = spread;
			break;
		case 255:
			solenoid(0);
			master.send(255, 255, 0);
			break;
		}
	} else {
		master.send(id, cmd, data);
	}
}

std_msgs::Int32 msg;
std_msgs::Float32MultiArray pot;
std_msgs::Bool /*limit_msg, */start_msg;
ros::Subscriber<std_msgs::Int32> mdd("motor_serial", &sendSerial);
ros::Publisher pub("mechanism_status", &msg);
ros::Publisher potato("potato", &pot);
//ros::Publisher limit_pub("clip_limit", &limit_msg);
ros::Publisher start("start_switch", &start_msg);

void start_fall() {
	start_msg.data = !start_switch.read();
	start.publish(&start_msg);
}

int main() {
	ros::NodeHandle nh;
	nh.getHardware()->setBaud(115200);
	nh.initNode();
	nh.subscribe(mdd);
	nh.advertise(pub);
	nh.advertise(potato);
	//nh.advertise(limit_pub);
	nh.advertise(start);
	start_switch.mode(PullUp);
	start_switch.fall(&start_fall);
	pot.data_length = 3;
	pot.data = (float*) malloc(sizeof(float) * pot.data_length);
	safe();
	servo1.period_ms(20);
	servo2.period_ms(20);
	servo1.pulsewidth_us(1450);
	servo2.pulsewidth_us(1450);
	slit_up1.mode(PullDown);
	slit_up2.mode(PullDown);
	bool on_flag[2] = { false, false };
	bool moving = false;
	int count = 1;
	uint8_t slit_up;
	uint8_t status = 0;
	Timer loop;
	/*if (!slit_up1) {
	 spread = 1;
	 }*/
	loop.start();
	while (true) {
		nh.spinOnce();
		if (loop.read_ms() > 20) {
			loop.reset();
			if (count == 1) {
				if (towel_order[0]) { //右側
					if (!(status & 0b00000001)) {
						status |= 0b00000001;
					}
					if (on_flag[0]) {
						if (slit_towel1.read()) {
							master.send(15, 2,
									50 * (towel_arm_goal == 1 ? 1 : -1));
						} else {
							master.send(15, 2, 0);
							towel_order[0] = false;
							on_flag[0] = false;
							status &= 0b11111110;
						}
					} else {
						if (slit_towel1.read()) {
							on_flag[0] = true;
						} else if (!slit_towel1.read()) {
							master.send(15, 2,
									50 * (towel_arm_goal == 1 ? -1 : 1));
						}
					}
				}
			} else if (count == 2) {
				if (towel_order[1]) { //左側
					if (!(status & 0b00000010)) {
						status |= 0b00000010;
					}
					if (on_flag[1]) {
						if (slit_towel2.read()) {
							master.send(15, 3,
									50 * (towel_arm_goal == 1 ? -1 : 1));
						} else {
							master.send(15, 3, 0);
							towel_order[1] = false;
							on_flag[1] = false;
							status &= 0b11111101;
						}
					} else {
						if (slit_towel2.read()) {
							on_flag[1] = true;
						} else {
							master.send(15, 3,
									50 * (towel_arm_goal == 1 ? 1 : -1));
						}
					}
				}
			} else if (count == 3) {
				slit_up = (slit_up1 << 1) + slit_up2; //上　下
				if (spread != spread_goal) {
					if (!(status & 0b00000100)) {
						status |= 0b00000100;
					}
					if (spread_goal == 0) {
						if (slit_up == 2) {
							master.send(15, 4, 0);
							spread = 0;
							moving = false;
							status &= 0b11111011;
							servo1.pulsewidth_us(1450); //ロックを緩めておく
							servo2.pulsewidth_us(1450);
							lock[0] = true;
							lock[1] = true;
						} else {
							if (lock[0] || lock[1]) {
								servo1.pulsewidth_us(2300); //ロック解除
								servo2.pulsewidth_us(2300);
								lock[0] = false;
								lock[1] = false;
								wait(0.5);
							}
							if (slit_up == 1) {
								moving = true;
							}
							master.send(15, 4, 200);
						}
					} else if (spread_goal == 1) {
						if (moving) {
							if (slit_up == 1) {
								master.send(15, 4, 50);
								spread = 0;
							} else if (slit_up == 0) {
								if (spread == 0) {
									master.send(15, 4, 0);
									spread = 1;
									moving = false;
									status &= 0b11111011;
									servo2.pulsewidth_us(1450);
									lock[1] = true;
								}
							}
						} else {
							if (!lock[0] || lock[1]) {
								servo1.pulsewidth_us(1450); //サーボ1個目（上に上がる方）
								servo2.pulsewidth_us(2300); //サーボ2個め（下に残る方）
								lock[0] = true;
								lock[1] = false;
								wait(0.5);
							}
							if (spread == 0) {
								master.send(15, 4, -250); //上昇
							} else if (spread == 2) {
								master.send(15, 4, 100); //下降
							}
							if (slit_up == 0) {
								moving = true;
							}
						}
					} else if (spread_goal == 2) {
						if (moving) {
							if (slit_up == 1) {
								master.send(15, 4, 0);
								spread = 2;
								moving = false;
								status &= 0b11111011;
							}
						} else {
							if (!lock[0] || !lock[1]) {
								servo1.pulsewidth_us(1450);
								servo2.pulsewidth_us(1450);
								lock[0] = true;
								lock[1] = true;
								wait(0.5);
							}
							if (slit_up == 1 && spread == 0) {
								spread = 1;
							} else if (spread == 1 && slit_up == 0) {
								moving = true;
							}
							master.send(15, 4, -250);
						}
					}
				} else if (slit_up > 0) {
					master.send(15, 4, 0);
				} else if (spread > 0) {
					master.send(15, 4, -150);
				}
			} else if (count == 4) {
				if (potentiometer > 0.90 || potentiometer < 0.01
						|| !open_order) { //0.03~ 0.02
				} else if (sheet_open) {
					if (potentiometer < 0.03) {
						master.send(15, 5, 50);
						if (!(status & 0b00001000)) {
							status |= 0b00001000;
						}
					} else {
						master.send(15, 5, 0);
						open_order = false;
						if (status & 0b00001000) {
							status &= 0b11110111;
						}
					}
				} else {
					if (potentiometer > 0.02) {
						master.send(15, 5, -50);
						if (!(status & 0b00001000)) {
							status |= 0b00001000;
						}
					} else {
						master.send(15, 5, 0);
						open_order = false;
						if (status & 0b00001000) {
							status &= 0b11110111;
						}
					}
				}
				count = 0;
			}
			count++;
			//limit_msg.data = limit_clip; //NCにつなぐ
			//limit_pub.publish(&limit_msg);

			msg.data = status + (spread << 8);
			pub.publish(&msg);

			pot.data[0] = slit_up;
			pot.data[1] = moving;
			pot.data[2] = potentiometer;
			potato.publish(&pot);
		}
	}
}
