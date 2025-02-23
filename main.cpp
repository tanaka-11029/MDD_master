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

DigitalOut led1(PB_7);

Timer sheet_time;
Timer start_time;

int8_t spread = 0; //0:展開前　1:第一展開　2:第二展開
int8_t spread_goal = 0;
bool towel_arm_goal = false; //1:展開　2:戻す
bool sheet_open = false;
bool sheet_now = false;
bool lock[2] = { false, false };
bool towel_order[2] = { false, false };
bool towel_now[2] = { false, false };
bool open_order = false;
bool emergency = false;
bool send_order = false;
int led_color = 10;
int led_last = 0;
int send_id;
int send_cmd;
int send_data;
int open_count = 10;

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
			break;
		case 3:
			spread_goal = data;
			break;
		case 4:
			sheet_open = (bool) data;
			open_order = true;
			if (sheet_open) {
				open_count = 0;
			}
			break;
		case 5:
			solenoid(data);
			break;
		case 6:
			spread = data;
			spread_goal = spread;
			break;
		case 7:
			emergency = (bool) data;
			break;
		case 8:
			towel_now[0] = (bool) data;
			break;
		case 9:
			towel_now[1] = (bool) data;
			break;
		case 255:
			solenoid(0);
			master.send(255, 255, 0);
			break;
		}
	} else {
		send_order = true;
		send_id = id;
		send_cmd = cmd;
		send_data = data;
	}
}

std_msgs::Int16 msg;
std_msgs::Float32MultiArray pot;
std_msgs::Bool /*limit_msg, */start_msg;
ros::Subscriber<std_msgs::Int32> mdd("motor_serial", &sendSerial);
ros::Publisher pub("mechanism_status", &msg);
ros::Publisher potato("potato", &pot);
//ros::Publisher limit_pub("clip_limit", &limit_msg);
ros::Publisher start("start_switch", &start_msg);

void start_fall() {
	start_msg.data = !start_switch.read();
	if (start_time.read_ms() > 1000 && start_msg.data) {
		start.publish(&start_msg);
		start_time.reset();
	}
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
	bool off_flag[2] = { false, false };
	bool moving = false;
	bool spread_order = false;
	bool down_flag = false;
	int updown = 0;
	int count = 1;
	int send_count = 0;
	uint8_t slit_up;
	uint8_t status = 0;
	Timer loop;
	start_time.start();
	/*if (!slit_up1) {
	 spread = 1;
	 }*/
	loop.start();
	while (true) {
		nh.spinOnce();
		if (loop.read_ms() > 10) {
			loop.reset();
			if (count == 1) {
				if (towel_arm_goal != towel_now[0] && !towel_order[0]) {
					towel_order[0] = true;
				} else if (towel_order[0]) { //右側
					if (!(status & 0b00000001)) {
						status |= 0b00000001;
					}
					if (off_flag[0]) {
						if (slit_towel1.read()) {
							master.send(10, 2, 0);
							towel_order[0] = false;
							off_flag[0] = false;
							towel_now[0] = towel_arm_goal;
							status &= 0b11111110;
						} else {
							master.send(10, 2,
									200 * (towel_arm_goal == 1 ? -1 : 1));
						}
					} else {
						master.send(10, 2,
								200 * (towel_arm_goal == 1 ? -1 : 1));
						if (!slit_towel1.read()) {
							off_flag[0] = true;
						}
						led_color = 40;
					}
					/*
					 if (on_flag[0]) {
					 if (slit_towel1.read()) {
					 master.send(10, 2,
					 50 * (towel_arm_goal == 1 ? 1 : -1));
					 } else {
					 master.send(10, 2, 0);
					 towel_order[0] = false;
					 on_flag[0] = false;
					 status &= 0b11111110;
					 }
					 } else {
					 if (slit_towel1.read()) {
					 on_flag[0] = true;
					 } else if (!slit_towel1.read()) {
					 master.send(10, 2,
					 100 * (towel_arm_goal == 1 ? -1 : 1));
					 }
					 }*/
				} else {
					master.send(10, 2, 0);
				}
			} else if (count == 2) {
				if (towel_arm_goal != towel_now[1] && !towel_order[1]) {
					towel_order[1] = true;
				} else if (towel_order[1]) { //左側
					if (!(status & 0b00000010)) {
						status |= 0b00000010;
					}
					if (off_flag[1]) {
						if (slit_towel2.read()) {
							master.send(10, 3, 0);
							towel_order[1] = false;
							off_flag[1] = false;
							towel_now[1] = towel_arm_goal;
							status &= 0b11111101;
						} else {
							master.send(10, 3,
									200 * (towel_arm_goal == 1 ? -1 : 1));
						}
					} else {
						master.send(10, 3,
								200 * (towel_arm_goal == 1 ? -1 : 1));
						if (!slit_towel2.read()) {
							off_flag[1] = true;
						}
						led_color = 40;
					}
					/*
					 if (on_flag[1]) {
					 if (slit_towel2.read()) {
					 master.send(10, 3,
					 50 * (towel_arm_goal == 1 ? -1 : 1));
					 } else {
					 master.send(10, 3, 0);
					 towel_order[1] = false;
					 on_flag[1] = false;
					 status &= 0b11111101;
					 }
					 } else {
					 if (slit_towel2.read()) {
					 on_flag[1] = true;
					 } else {
					 master.send(10, 3,
					 100 * (towel_arm_goal == 1 ? 1 : -1));
					 }
					 }*/
				} else {
					master.send(10, 3, 0);
				}
			} else if (count == 3) {
				slit_up = (slit_up1 << 1) + slit_up2; //上　下
				if (!spread_order && spread != spread_goal
						&& (slit_up != 0 || spread == 0)) {
					spread_order = true;
				} else if (spread_order) {
					if (!(status & 0b00000100)) {
						status |= 0b00000100;
					}
					if (spread_goal == 0) {
						if (slit_up == 2) {
							master.send(17, 5, 0);
							spread = 0;
							moving = false;
							status &= 0b11111011;
							servo1.pulsewidth_us(1450); //ロックを緩めておく
							servo2.pulsewidth_us(1450);
							spread_order = false;
							lock[0] = true;
							lock[1] = true;
							updown = 0;
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
							updown = 2;
							master.send(17, 5, 200);
							led_color = 30;
						}
					} else if (spread_goal == 1) {
						if (moving) {
							if (slit_up == 0) {
								if (down_flag) {
									master.send(17, 5, -150);
									spread = 0;
								} else if (spread == 0) {
									master.send(17, 5, -250);
								} else if (spread == 2) {
									master.send(17, 5, 250);
								} else if (spread == 1) {
									if (updown == 1) {
										master.send(17, 5, 250);
									} else if (updown == 2) {
										master.send(17, 5, -250);
									}
								}
							} else if (slit_up == 1) {
								if (spread == 0) {
									master.send(17, 5, 0);
									spread = 1;
									moving = false;
									down_flag = false;
									status &= 0b11111011;
									updown = 0;
									servo2.pulsewidth_us(1450);
									spread_order = false;
									lock[1] = true;
								} else {
									master.send(17, 5, 200);
									down_flag = true;
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
								master.send(17, 5, -250); //上昇
								updown = 1;
								led_color = 20;
							} else if (spread == 2) {
								master.send(17, 5, 250); //下降
								updown = 2;
								led_color = 30;
							} else if (spread == 1) {
								if (updown == 1) {
									master.send(17, 5, 250);
								} else if (updown == 2) {
									master.send(17, 5, -250);
								}
							}
							if (slit_up == 0) {
								moving = true;
							}
						}
					} else if (spread_goal == 2) {
						if (moving) {
							if (slit_up == 1) {
								led1 = 1;
								master.send(17, 5, 0);
								spread_order = false;
								spread = 2;
								moving = false;
								updown = 0;
								status &= 0b11111011;
								servo1.pulsewidth_us(1450); //ロックを緩めておく
								servo2.pulsewidth_us(1450);
								lock[0] = true;
								lock[1] = true;
							} else {
								master.send(17, 5, -250);
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
							updown = 1;
							led1 = 0;
							master.send(17, 5, -250);
							led_color = 20;
						}
					}
				} else if (slit_up > 0) {
					master.send(17, 5, 0);
				} else if (spread > 0) {
					master.send(17, 5, -150);
				}
			} else if (count == 4) {
				if (!open_order) { //0.0058 ~ 0.002
					master.send(10, 5, 0);
				} else if (sheet_open) {
					if (potentiometer < 0.005) {
						master.send(10, 5, -50);
						if (!(status & 0b00001000)) {
							status |= 0b00001000;
						}
					} else {
						master.send(10, 5, 0);
						if (open_count > 5) {
							open_order = false;
							if (status & 0b00001000) {
								status &= 0b11110111;
							}
						} else {
							open_count++;
						}
					}
					led_color = 50;
				} else {
					if (potentiometer > 0.003) {
						master.send(10, 5, 50);
						if (!(status & 0b00001000)) {
							status |= 0b00001000;
						}
					} else {
						master.send(10, 5, 0);
						open_order = false;
						if (status & 0b00001000) {
							status &= 0b11110111;
						}
					}
				}/*
				 if (sheet_open != sheet_now) {
				 if (sheet_open) {
				 master.send(10, 5, -50);
				 if (!(status & 0b00001000)) {
				 status |= 0b00001000;
				 sheet_time.reset();
				 sheet_time.start();
				 }
				 if (sheet_time.read_ms() > 1500) {
				 master.send(10, 5, 0);
				 sheet_now = true;
				 sheet_time.stop();
				 if (status & 0b00001000) {
				 status &= 0b11110111;
				 }
				 }
				 } else {
				 sheet_now = sheet_open;
				 master.send(10, 5, 50);
				 if (!(status & 0b00001000)) {
				 status |= 0b00001000;
				 sheet_time.reset();
				 sheet_time.start();
				 }
				 if (sheet_time.read_ms() > 1000) {
				 master.send(10, 5, 0);
				 sheet_now = false;
				 sheet_time.stop();
				 if (status & 0b00001000) {
				 status &= 0b11110111;
				 }
				 }
				 }
				 } else {
				 master.send(10, 5, 0);
				 }*/
			} else if (count == 5) {
				if (emergency) {
					led_color = 200;
				} else if (status == 0) {
					led_color = 10;
				}
				if (led_color != led_last) {
					led_last = led_color;
					send_count = 0;
				}
				if (send_count < 1) {
					send_count++;
					if (emergency) {
						master.send(84, 200, 150);
					} else {
						master.send(84, led_color, 40);
					}
				}
			} else if (count == 6) {
				count = 0;
				if (send_order) {
					send_order = false;
					master.send(send_id, send_cmd, send_data);
				} else if (emergency) {
					master.send(255, 255, 0);
				}
			}
			count++;
			//limit_msg.data = limit_clip; //NCにつなぐ
			//limit_pub.publish(&limit_msg);

			msg.data = status + (spread << 8);
			pub.publish(&msg);

			pot.data[0] = (slit_towel1) + (slit_towel2 << 1);
			pot.data[1] = slit_up;
			pot.data[2] = potentiometer;
			potato.publish(&pot);
		}
	}
}
