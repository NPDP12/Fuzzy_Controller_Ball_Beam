#include <Arduino.h>
#include <Servo.h>
#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


LiquidCrystal_I2C lcd(0x27, 16,2);


//#include <TimerOne.h>
// put function declarations here:
#define LED PC13
#define pinservo PA0
#define TRIG_PIN PB12 //DO KHOANG CACH HIEN TAI
#define ECHO_PIN PB13//

#define TRIG_PIN2 PA4//dat setpoint
#define ECHO_PIN2 PA5
#define trig_set PB2
#define echo_set PB3

Servo Myservo;
SimpleKalmanFilter boloc(2, 2, 0.85); //loc nhieu cam bien sieu am
float e[5];
float edot[5];
float ce[5] = {-0.75, -0.25, 0, 0.25, 0.75}; // gia tri  sai so e
float cedot[5] = {-0.75, -0.42, 0, 0.42, 0.75}; // gia tri toc do bien thien sai so edot

float y_out[7] = {-1, -0.8, -0.4, 0, 0.4, 0.8, 1}; //giá trị trục hoành của ngõ ra ngôn ngữ góc lệch (servo)
float y_tam[5][5];
float y_sao;
//float y_kalman;
float beta[5][5];
int cout=0;
// int cout1=0;
// int matrix[5][5]= { {PB, PB, PM, PS, ZE},
//                      {PB, PM, PS, ZE, NS},
//                      {PM, PS, ZE, NS, NM},
//                      {PS, ZE, NS, NM, NB},
//                      {ZE, NS, NM, NB, NB}                    
// };

//setup he so K ngo vao
float Ke =1/50.000;
float Kedot = 1/10.000;
float Ku = 90; // độ


float s_pre;
int s_setpoint;
float s_past=0;
float a;
float error;
float errordot;
float time_sample= 0.05; // thời gian lấy mẫu (giây)
float goclech;
float goclech_temp;
unsigned long t1;

float hth_hinhthang(float L, float C1, float C2, float R);

void center();
void timerover();

float GetDistance2()
{
  float duration, distanceCm;
   
  digitalWrite(TRIG_PIN2, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN2, LOW);
  
  duration = pulseIn(ECHO_PIN2, HIGH);
  // convert to distance
  distanceCm = (duration / 29.1 / 2.0);
  
  return distanceCm;
}

float GetDistance()
{
  float duration, distanceCm;
   
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  duration = pulseIn(ECHO_PIN, HIGH);
  // convert to distance
  distanceCm = (duration / 29.1 / 2.0);
  if(distanceCm <12)
  {
    distanceCm -=1;
  }
  return distanceCm;
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  lcd.init();                    
  lcd.backlight();
  lcd.setCursor(2,0);


  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(trig_set, OUTPUT);
  pinMode(echo_set, INPUT);
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT);

  Myservo.attach(pinservo);
  Myservo.write(90);

  // timer.pause();
  // timer.setOverflow(10000, MICROSEC_FORMAT); //ngắt 10ms
  // timer.attachInterrupt(timerover);
  // timer.resume();
}

void loop() {
  
 // t1=millis();
  s_pre = boloc.updateEstimate(GetDistance()); //đo khoảng cách hiện tại
  s_setpoint =15; // đo khoảng cách đặt setpoint
  error = (float)(s_setpoint - s_pre)*Ke; //tính toán sai số ngõ vào e quy về [-1;1]

  // phần chặn trên và dưới [-1;1] cho e
  if(error >1)
  {
    error = 1;
  }else if(error<-1)
  {
    error = -1;
  }
 
  errordot = (float)(((s_setpoint - s_pre)-(s_setpoint - s_past))/time_sample)*Kedot;
  s_past = s_pre;

  // phần chặn trên và dưới [-1;1] cho edot
  if(errordot >1)
  {
    errordot = 1;
  }else if(errordot<-1)
  {
    errordot = -1;
  }
  
  center(); // phần tính toán ngõ ra y_sao dựa vào ngõ vào e, edot
  // phương pháp suy luận mờ max prod, tính trung bình có trọng số,   dạng sugeno

  goclech_temp = y_sao*Ku;// tính toán góc quay của servo
  

  // chặn trên và dưới cho góc quay servo
  if(goclech > 0)
  {
    goclech = -goclech_temp +85;
  }
  else 
  {
    goclech= 85 + goclech_temp ;
  }

  if(goclech >120)
  {
    goclech = 120;
  }else if(goclech<50)
  {
    goclech = 50;
  }

  //  Serial.print(errordot);
  // Serial.print("         ");
  // phần này hiển thị để check lỗi sai :))))
  // Serial.print("y_out:  ");
  // Serial.print(s_pre );
  // Serial.print("         setpoint: ");
  
  // Serial.println(s_setpoint );
  lcd.setCursor(0,0);
  lcd.print("d_out:    ");
  lcd.setCursor(10,0);
  lcd.print(s_pre);

  lcd.setCursor(0,1);
  lcd.print("setpoint: ");
  lcd.setCursor(11,1);

  lcd.print("    ");
  lcd.setCursor(10,1);
  lcd.print(s_setpoint);
    
  
  //lcd.clear();

  // Serial.print("        ");
  // Serial.print(s_pre );
  // Serial.print("        ");
  // Serial.println(s_setpoint);
  // Serial.print("        ");
  // Serial.println(setpoint);
  

  Myservo.write(goclech); // xuất vị trí cho servo
  delay(3);
  
 
}

// tạo hàm hình thang cho từng giá trị ngon ngu
float hlt_hinhthang(float x, float L, float C1, float C2, float R)
{
  float val;
  if((x<L))
  {
    val= 0;
  }
  else if(x<C1)
  {
    val = (x-L)/(C1-L);
  }
  else if(x<C2)
  {
    val = 1;
  }
  else if(x<R)
  {
    val = (R-x)/(R-C2);
  }
  else
  {
    val =0;
  }
  return val;

}

void center()
{
  //sai so
  e[0] = hlt_hinhthang(error, -3, -2 , ce[0], ce[1]);   // NB
  e[1] = hlt_hinhthang(error, ce[0], ce[1] , ce[1], ce[2]);  //NS
  e[2] = hlt_hinhthang(error, ce[1], ce[2] , ce[2], ce[3]);      //ZE
  e[3] = hlt_hinhthang(error, ce[2], ce[3] , ce[3], ce[4]);     //PS
  e[4] = hlt_hinhthang(error, ce[3], ce[4] , 2 , 3);      //PB

  //toc do sai so
  edot[0] = hlt_hinhthang(errordot, -3, -2 , cedot[0], cedot[1]);   // NB
  edot[1] = hlt_hinhthang(errordot, cedot[0], cedot[1] , cedot[1], cedot[2]);  //NS
  edot[2] = hlt_hinhthang(errordot, cedot[1], cedot[2] , cedot[2], cedot[3]);      //ZE
  edot[3] = hlt_hinhthang(errordot, cedot[2], cedot[3] , cedot[3], cedot[4]);     //PS
  edot[4] = hlt_hinhthang(errordot, cedot[3], cedot[4] , 2 , 3);      //PB

  //output tinh toan ngo ra dua vao sai so  va toc do sai so
  for (int i=0; i<5; i++)  // i hàng
  {
    for (int j=0;j<5; j++) // j cột
    {
      //tinh beta
        // if(e[i]<edot[j])
        // {
        //   beta[i][j] = e[i];
        // }
        // else
        // {
        //   beta[i][j] = edot[j];
        // }
        beta[i][j] = e[i]*edot[j];

        //tinh y_tam
        if(((i==0)&&(j==0))||((i==0)&&(j==1))||((i==1)&&(j==0)))
        {
          y_tam[i][j] = y_out[6]; //vi tri PB
        }
        else if(((i==2)&&(j==0))||((i==1)&&(j==1))||((i==0)&&(j==2)))
        {
          y_tam[i][j] = y_out[5]; // vi tri PM
        }
        else if(((i==3)&&(j==0))||((i==2)&&(j==1))||((i==1)&&(j==2))||((i==0)&&(j==3)))
        {
          y_tam[i][j] = y_out[4]; // vi tri PS
        }
        else if(((i==4)&&(j==0))||((i==3)&&(j==1))||((i==2)&&(j==2))||((i==1)&&(j==3))||((i==0)&&(j==4)))
        {
          y_tam[i][j] = y_out[3]; // vi tri ZE
        }
        else if(((i==4)&&(j==1))||((i==3)&&(j==2))||((i==2)&&(j==3))||((i==1)&&(j==4)))
        {
          y_tam[i][j] = y_out[2]; // vi tri NS
        }
        else if(((i==4)&&(j==2))||((i==3)&&(j==3))||((i==2)&&(j==4)))
        {
          y_tam[i][j] = y_out[1]; // vi tri NM
        }
        else
        {
          y_tam[i][j] = y_out[0]; // vi tri NB
        }
    }
  }
  float tuso = 0.0;
  float mauso = 0.0;

  for (int i =0; i<5; i++)
  {
    for(int j=0; j<5; j++)
    {
      tuso = tuso + (beta[i][j])*(y_tam[i][j]);
      mauso = mauso + beta[i][j];
    }
  }
  y_sao = tuso/mauso;
}