#include "mbed.h"
#include "library/scrp_slave.hpp"
#include "library/rotary_inc.hpp"
#define MAXPWM 250
#define Period 256

ScrpSlave slave(PA_9,PA_10,PA_12,SERIAL_TX,SERIAL_RX,0x0803e000);

const PinName pwmpin[5][3] = {
    {PB_0 ,PB_1 ,PB_3},
    {PA_1 ,PA_3 ,PB_4},
    {PA_8 ,PA_7 ,PB_5},
    {PB_6 ,PA_11,PB_7},
    {PA_0 ,PA_4 ,NC}//ロータリーエンコーダー用
};

//const PinName rotarypin[2] = {PA_0,PA_4};

//RotaryInc rotary(rotarypin[0],rotarypin[1],10,200,0);
uint8_t flag[5];

bool Drive(int id,int pwm){
    pwm = constrain(pwm,-MAXPWM,MAXPWM);
    DigitalOut Led(pwmpin[id][2]);
    if(!pwm){
        DigitalOut Moter1(pwmpin[id][0],0);
        DigitalOut Moter2(pwmpin[id][1],0);
        flag[id] = 0;
        Led.write(0);
    }else if(0 < pwm){
        PwmOut Moter1(pwmpin[id][0]);
        Moter1.period_us(Period);
        Moter1.write((float)pwm/255);
        DigitalOut Moter2(pwmpin[id][1],0);
        flag[id] = 2;
        Led.write(1);
    }else{
        DigitalOut Moter1(pwmpin[id][0],0);
        PwmOut Moter2(pwmpin[id][1]);
        Moter2.period_us(Period);
        Moter2.write((float)-pwm/255);
        flag[id] = 1;
        Led.write(1);
    }
    return true;
}

bool solenoid(int id,int data){
    if(data == 0){
        flag[id] = 0;
        DigitalOut Moter1(pwmpin[id][0],0);
        DigitalOut Moter2(pwmpin[id][1],0);
    }else if(data == 1){
        flag[id] |= 0x01;
        DigitalOut Moter1(pwmpin[id][0],1);
    }else if(data == 2){
        flag[id] |= 0x02;
        DigitalOut Moter2(pwmpin[id][1],1);
    }else if(data == -1){
        flag[id] &= 0xfe;
        DigitalOut Moter1(pwmpin[id][0],0);
    }else if(data == -2){
        flag[id] &= 0xfd;
        DigitalOut Moter2(pwmpin[id][1],0);
    }
    if(flag[id] == 0){
        DigitalOut Led(pwmpin[id][2],0);
    }else{
        DigitalOut Led(pwmpin[id][2],1);
    }    
    return true;    
}

bool safe(int rx_data,int &tx_data){
    for(int i = 0;i < 5;i++){
        DigitalOut Moter1(pwmpin[i][0],0);
        DigitalOut Moter2(pwmpin[i][1],0);   
        DigitalOut Led(pwmpin[i][2],0);
    }
    return true;
}

bool DM1(int rx_data,int &tx_data){
    return Drive(0,rx_data);
}

bool DM2(int rx_data,int &tx_data){
    return Drive(1,rx_data);
}

bool DM3(int rx_data,int &tx_data){
    return Drive(2,rx_data);
}

bool DM4(int rx_data,int &tx_data){
    return Drive(3,rx_data);
}

bool SL1(int rx_data,int &tx_data){
    return solenoid(0,rx_data);
}

bool SL2(int rx_data,int &tx_data){
    return solenoid(1,rx_data);
}

bool SL3(int rx_data,int &tx_data){
    return solenoid(2,rx_data);
}

bool SL4(int rx_data,int &tx_data){
    return solenoid(3,rx_data);
}

bool RO(int rx_data,int &tx_data){
    //tx_data = rotary.get();
    //return true;
    return solenoid(4,rx_data);
}

int main(){
    Drive(0,0);
    Drive(1,0);
    Drive(2,0);
    Drive(3,0);
    solenoid(4,0);
    slave.addCMD(2,DM1);
    slave.addCMD(3,DM2);
    slave.addCMD(4,DM3);
    slave.addCMD(5,DM4);
    slave.addCMD(6,SL1);
    slave.addCMD(7,SL2);
    slave.addCMD(8,SL3);
    slave.addCMD(9,SL4);
    slave.addCMD(10,RO);
    slave.addCMD(255,safe);
    while(true);
}
