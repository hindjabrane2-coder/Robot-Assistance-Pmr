#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H
class MotorDriver{
    int eL,i1L,i2L,eR,i1R,i2R;
public:
    MotorDriver(int el,int a,int b,int er,int c,int d):eL(el),i1L(a),i2L(b),eR(er),i1R(c),i2R(d){}
    void begin(){pinMode(eL,OUTPUT);pinMode(i1L,OUTPUT);pinMode(i2L,OUTPUT);pinMode(eR,OUTPUT);pinMode(i1R,OUTPUT);pinMode(i2R,OUTPUT);stop();}
    void drive(int sL,bool fL,int sR,bool fR){analogWrite(eL,sL);digitalWrite(i1L,fL?HIGH:LOW);digitalWrite(i2L,fL?LOW:HIGH);analogWrite(eR,sR);digitalWrite(i1R,fR?HIGH:LOW);digitalWrite(i2R,fR?LOW:HIGH);}
    void stop(){analogWrite(eL,0);analogWrite(eR,0);}
};
#endif
