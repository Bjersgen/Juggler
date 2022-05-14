
#include <Servo.h>
#include "PID_v2.h"

//创建四个舵机
Servo myservolu;
Servo myservoru;
Servo myservold;
Servo myservord;

double  InputX, OutputX, deltaX,deltaZ;
double  InputY, OutputY, deltaY;
double  InputVx,OutputVx;
double  InputVy,OutputVy;
double SetpointX,SetpointVx;
double SetpointY,SetpointVy;
double kp = 0.05,ki = 0.008,kd =0.06;
unsigned long time1;
//位置PID
PID myPID1(&InputX, &OutputX, &SetpointX,kp,ki,kd,DIRECT);
PID myPID2(&InputY, &OutputY, &SetpointY,kp,ki,kd,DIRECT);
//速度PID
PID myPID3(&InputVx, &OutputVx, &SetpointVx,kp,ki,kd,DIRECT);
PID myPID4(&InputVy, &OutputVy, &SetpointVy,kp,ki,kd,DIRECT);
int flag = 1;
int Ald = 0;
int Ard = 0;
int Alu = 0;
int Aru = 0; 
int t = 0;
int cy=0;
int cx=0;
int cz=33;
int cx_1 =0;
int cy_1 =0;
int cd=0;
int vx=0;
int vy=0;
  //X:10-480
  //Y:86-566
int Xmin = 52;
int Xmax = 460;
int Ymin = 198;
int Ymax = 582;
int Xmid = 256;
int Ymid = 390;
int rx0 = 0;
int ry0 = 0;
int r1=0;
int r2=0;
int r3=0;
int r4=0;
int rz=0;
int rd=0;
int rx=0;//plate center x
int ry=0;//plate center y
int r_b=0;
int b_flag = 0;
int gap_flag = 1;
unsigned int now = 0;
unsigned int gap = 0;
void setup() {
  // put your setup code here, to run once:
  myservolu.attach(7);
  myservoru.attach(9);
  myservold.attach(6);
  myservord.attach(12);
  myPID1.SetTunings(kp,ki,kd);
  myPID2.SetTunings(kp,ki,kd);
  myPID3.SetTunings(kp,ki,kd);
  myPID4.SetTunings(kp,ki,kd);

  myPID1.SetOutputLimits(-255,255);
  myPID2.SetOutputLimits(-255,255);
  //x、y轴运动速度限制
  myPID3.SetOutputLimits(-10,10);
  myPID4.SetOutputLimits(-10,10);

  myPID1.SetSampleTime(20);
  myPID2.SetSampleTime(20);
  myPID3.SetSampleTime(20);
  myPID4.SetSampleTime(20);

  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  myPID3.SetMode(AUTOMATIC);
  myPID4.SetMode(AUTOMATIC);

  Ald=50;
  Ard=50;
  Aru=50;
  Alu=50;

  //在9600 bps打开串行端口：
  Serial.begin(115200);
  Serial1.begin(115200);
  SetpointX = 0;
  SetpointY = 0;
  //myPID2.mySetpoint = 0;
  myservolu.write(angle_lu(Ald));
  myservold.write(angle_ld(Ard));
  myservoru.write(angle_ru(Aru));
  myservord.write(angle_rd(Alu));
}
int angle_rd(int angle)
{
  return 90- angle*(130-35)/95;
}


int angle_ld(int angle)
{
  return angle*(115-20)/90+30;
}

int angle_ru(int angle)
{
  return 100 - angle;
}

int angle_lu(int angle)
{
  return 113 - angle;
}

int edge(int angle)
{
  if( angle>=45)
    return 45;
  else if(angle<=25)
    return 25;
  else return angle;
}

int edge_Bounce(int angle){
  if(angle>7.5)
    return 7.5;
  else if(angle<=-7.5)
    return -7.5;
  else return angle;
}

void loop() {
  // put your main code here, to run repeatedly:
  if(t%1000==0)t=0;
  //Serial.println("*****");
  while(Serial1.available()){
     String c1 = "";
     String c2;
     int numx[3] = {0,0,0};
     int numy[3] = {0,0,0};
//     int numy1[3] = {0,0,0};
//     int numy2[3] = {0,0,0};
     int numr[2] = {0,0};
//     int num4[3] = {0,0,0};
     int num_mx[3] = {0,0,0};
     int num_my[3] = {0,0,0};
     int readX1 = 0;
     int readX2 = 0;
     int readX = 0;
     int readX3 = 0;
     int readY1 = 0;
     int readY2 = 0;
     int readY = 0;
     int readZ = 0;
     int readD = 0;
     int readMX = 0;
     int readMY = 0;
     int i = 0;
     int j = 0;
     int k = 0;
     int q = 0;
     int m_i = 0;
     int m_j = 0;
     //int rd = 0;
     //while(1)
     //Serial.println(1);
     char inChar = Serial1.read();
     //Serial.println(1);
     while(inChar!= 'S')
      {
        //Serial.println(inChar);
        numx[i] = int(inChar-'0')*pow(10,2-i);       
        inChar = Serial1.read();
        //Serial.println('S');
        
        //readX3 +=numx[i];
        readX +=numx[i];
        if(t%2!=0){
            
            readX1 += numx[i];
        }
        if(t%2==0){
            readX2+=numx[i];
        } 
        if(t==0){
          readX1=256;
        }
        //Serial.println("T:");
        //Serial.println(t);
        i++;
        delay(1);
      }
      if(i<=2)readX1/=10;
      if(i<=2)readX2/=10;
      if(i<=2)readX/=10;
      inChar = Serial1.read();
      while(inChar!= 'T')
      {
        numy[j] = int(inChar-'0')*pow(10,2-j); 
        inChar = Serial1.read();
        delay(1);
        readY +=numy[j];
        if(t%2==0){
            readY1 += numy[j];
        }
        if(t%2!=0){
            readY2+= numy[j];
        } 
        if(t==0){
            readY1=390;
        }
        
        j++;
      }
      if(j<=2)readY1/=10;
      if(j<=2)readY2/=10;
      if(j<=2)readY/=10;
      inChar = Serial1.read();
      while(inChar!='Z')
      {
        //Serial.println(inChar);
        numr[k] = int(inChar -'0')*pow(10,1-k);
        inChar = Serial1.read();
        delay(1);
        readZ +=numr[k];
        k++;
      }
      inChar = Serial1.read();
      while(inChar!='D')
      {
//        if(inChar =='2')
//          readD = 1;//上升
//        else if(inChar=='1')
//          readD = -1;//下降
        readD = int(inChar-'0');
        inChar = '0';
        inChar = Serial1.read();
        delay(3);
      }
      inChar = Serial1.read();
      while(inChar!='X')
      {
        num_mx[m_i] = int(inChar -'0')*pow(10,2-m_i);
        inChar = Serial1.read();
        delay(1);
        readMX +=num_mx[m_i];
        //Serial.println(char(inChar));

        m_i++;
      }
      inChar = Serial1.read();
      while(inChar!='Y')
      {
        num_my[m_j] = int(inChar -'0')*pow(10,2-m_j);
        inChar = Serial1.read();
        delay(1);
        readMY +=num_my[m_j];
        m_j++;
      }
      //delay(5);
      //摄像头大概16毫秒捕捉一次，计算速度
      vx=flag*(readX2-readX1)/16;
      vy=flag*(readY1-readY2)/16;
      rx0 = readX;
      ry0 = readY;
      r1=readX1;
      r2=readX2;
      r3=readY1;
      r4=readY2;
      rd=readD;
      rz=readZ;
      rx=readMX;
      ry=readMY;
      
  }
  //Serial.println(rz);
  vx =0;
  vy =0;
  rx = 210;
  ry = 204;
  //Serial.println(ry0);
  InputVx=vx;
  InputVy=vy;
  //Serial.println(vx);
  myPID3.Compute();
  myPID4.Compute();
  //根据PID后的速度推断出应该有的x和y的位置
  if(t%2==0){
    cx=double(2*((OutputVx*16+r2)-rx))/300*100;
    cy=double(2*((OutputVy*16+r4)-rx))/300*100;
  }
  if(t%2!=0){
    cx=double(2*((OutputVx*16+r1)-rx))/300*100;
    cy=double(2*((OutputVy*16+r3)-rx))/300*100;
  }

  cx_1 = double(rx0 - rx)/300*100;
  cy_1 = double(ry0 - ry)/300*100; 
  //cx =0; 
  //cy =100;
  InputY=-cy_1;
  InputX=cx_1;
  myPID1.Compute();
  myPID2.Compute();
   
  deltaX = OutputX*270/255*0.1;
  deltaY = OutputY*270/255*0.1;
  //Serial.println(deltaX);
  //Serial.println(deltaY);
  Serial.println("___");
  //Serial.println(t);
  //Serial.println(r1);
  //判断小球是否即将落到板子上
  deltaZ=0;
//  if(rz<20&&rd==-1){
//    deltaZ=40;
//  }
//  Ald = 35 + (-deltaX + deltaY) +deltaZ;
//  Alu = 35 + (-deltaX - deltaY) +deltaZ;
//  Aru = 35 + (deltaX  - deltaY) +deltaZ;
//  Ard = 35 + (deltaX  + deltaY) +deltaZ;
//
//  
//
//  
//  Ald = edge(Ald);
//  Alu = edge(Alu);
//  Ard = edge(Ard);
//  Aru = edge(Aru);
//  //Serial.println(Aru);
////
////Serial.println(r1);
//  myservolu.write(angle_lu(Alu));
//  myservold.write(angle_ld(Ald));
//  myservoru.write(angle_ru(Aru));
//  myservord.write(angle_rd(Ard)); 
  //Serial.println(r1);
//  Serial.print("Ard:");
//  Serial.print(Ard);
//  Serial.print("Aru:");
//  Serial.print(Aru);
//  Serial.println(" ");
//  Ard = 75;
//  Alu = 75;
//  Aru = 75;
//  Ald = 75; 
  //Aru = 0;
//
  //Serial.println(rd);
//  if(rz>r_b)r_b=rz;
//  int Bounce(int r)
//  {
//    while(rz>=r_b){
//      
//    }
//    r_b = 0;
//    
//  }
int angle_ = 35;
  if(b_flag==0)
  {
  
  Alu = angle_-5; 
  Ald = angle_-5;
  Aru = angle_;
  Ard = angle_;

  myservolu.write(angle_lu(Alu));
  myservold.write(angle_ld(Ald));
  myservoru.write(angle_ru(Aru));
  myservord.write(angle_rd(Ard));
  }
  //time1 = millis();
  //Serial.println(rd);
  //delay(200);
  //rz = 1;
  //rd = 1;
  if(rd==1 && b_flag==0 && gap_flag==1){
  now = millis();
  b_flag = 1;
  
  
  angle_ = 15;
  
  Alu = angle_+ edge_Bounce(-deltaX + deltaY);
  Ald = angle_+ edge_Bounce(-deltaX - deltaY);
  Aru = angle_+ edge_Bounce(deltaX  - deltaY);
  Ard = angle_+ edge_Bounce(deltaX  + deltaY);
  //Serial.println(rd);
  myservord.write(angle_rd(Ard));  
  myservolu.write(angle_lu(Alu));
  myservold.write(angle_ld(Ald));
  myservoru.write(angle_ru(Aru));
  delay(10);
  //Serial.println(Alu);
  
  }
  //Serial.println(millis()-now);
  //Serial.println(millis()-now);
  //delay(200);
 if(millis()-now>200 && b_flag==1)
{
  b_flag = 0;
  gap = millis();
  gap_flag = 0;
  int angle_ = 35;
  Alu = angle_-5; 
  Ald = angle_-5;
  Aru = angle_;
  Ard = angle_;
  //Serial.println(rd);
  myservolu.write(angle_lu(Alu));
  myservold.write(angle_ld(Ald));
  myservoru.write(angle_ru(Aru));
  myservord.write(angle_rd(Ard));
  delay(10);
  //Serial.println(b_flag);
  //Serial.println("here");
  //delay(200);
}
if(millis() - gap >200 && b_flag==0){
  gap = 0;
  gap_flag = 1;
}
Serial.println(rd);
  

  //rd = 0;


  
//  if(rz>25&&rd==-1){
//    myservolu.write(angle_lu(Ald-40));
//    myservold.write(angle_ld(Alu-40));
//    myservoru.write(angle_ru(Aru-40));
//    myservord.write(angle_rd(Ard-40));
//  }
  
  flag*=-1;
  t++;
  
  //delay(10);
}
  
