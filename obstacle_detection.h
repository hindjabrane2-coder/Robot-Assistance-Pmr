#ifndef OBSTACLE_DETECTION_H
#define OBSTACLE_DETECTION_H
enum Direction{FRONT=0,BACK=1,LEFT=2,RIGHT=3};
class ObstacleDetector{
    int trig[4],echo[4];
    float measure(int t,int e){digitalWrite(t,LOW);delayMicroseconds(2);digitalWrite(t,HIGH);delayMicroseconds(10);digitalWrite(t,LOW);long d=pulseIn(e,HIGH,30000);return d==0?-1:d*0.034/2.0;}
public:
    void begin(int ft,int fe,int bt,int be,int lt,int le,int rt,int re){trig[0]=ft;echo[0]=fe;trig[1]=bt;echo[1]=be;trig[2]=lt;echo[2]=le;trig[3]=rt;echo[3]=re;for(int i=0;i<4;i++){pinMode(trig[i],OUTPUT);pinMode(echo[i],INPUT);}}
    float getDistance(Direction d){return measure(trig[d],echo[d]);}
    bool isObstacle(Direction d,float min){float v=getDistance(d);return v>0&&v<min;}
};
#endif
