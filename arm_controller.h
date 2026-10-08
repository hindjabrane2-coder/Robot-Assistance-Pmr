#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H
#include <Servo.h>
#define GRIPPER_OPEN 30
#define GRIPPER_CLOSED 130
#define MOVE_DELAY 15
class ArmController {
    Servo sBase,sShoulder,sElbow,sGripper;
    int pB,pS,pE,pG,cB=90,cS=90,cE=90,cG=GRIPPER_OPEN;
    bool closed=false;
    void smooth(Servo &s,int &cur,int tgt){int st=(tgt>cur)?1:-1;while(cur!=tgt){cur+=st;s.write(cur);delay(MOVE_DELAY);}}
public:
    ArmController(int b,int s,int e,int g):pB(b),pS(s),pE(e),pG(g){}
    void begin(){sBase.attach(pB);sShoulder.attach(pS);sElbow.attach(pE);sGripper.attach(pG);}
    void goHome(){smooth(sBase,cB,90);smooth(sShoulder,cS,90);smooth(sElbow,cE,90);smooth(sGripper,cG,GRIPPER_OPEN);closed=false;}
    void goUp(){smooth(sShoulder,cS,45);smooth(sElbow,cE,60);}
    void goDown(){smooth(sShoulder,cS,135);smooth(sElbow,cE,120);}
    void grab(){goDown();delay(200);smooth(sGripper,cG,GRIPPER_CLOSED);closed=true;delay(300);goUp();}
    void release(){smooth(sGripper,cG,GRIPPER_OPEN);closed=false;}
    void toggleGripper(){if(closed)release();else{smooth(sGripper,cG,GRIPPER_CLOSED);closed=true;}}
    void setBase(int a){smooth(sBase,cB,constrain(a,0,180));}
    void setShoulder(int a){smooth(sShoulder,cS,constrain(a,30,150));}
    void setElbow(int a){smooth(sElbow,cE,constrain(a,30,150));}
    bool isGripperClosed(){return closed;}
};
#endif
