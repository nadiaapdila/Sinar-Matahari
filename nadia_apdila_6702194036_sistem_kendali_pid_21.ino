#include <EEPROM.h>

//encoder pin motor 1
#define encoderPinA_1 = 2;
#define encoderPinB_1 = 4;

//encoder pin motor 2
#define encoderPinA_2 = 10;
#define encoderPinB_2 = 11;

//Pin kontrol motor
#define motor-kiri1 = 7;
#define motor-kiri2 = 9;
#define motor-kanan1 = 5;
#define motor-kanan1 = 6;

#define pinEnable1 = 8;
#define pinEnable2 = 3;

//Pin tombol
#define button1 = 12;
#define button2 = 13;

//Variabel posisi encoder untuk menampilkan serial plot kurva respon PID
int encoderMotor1 = 0;
int encoderMotor2 = 0;

int x;
int alamat = 0;
int baca_sensor[6];
int peka[6]; //Variabel
//Array penyimpanan apabila sensor mencapai nilai terbesar
int sensorMax[6] = {607, 607, 607, 607, 607, 607};
//Array penyimpanan apabila sensor mencapai nilai terkecil
int sensorMin[6] = {33,33,33,33,33,33};
int sensor1 = A0;
int sensor2 = A1;
int sensor3 = A2;
int sensor4 = A3;
int sensor5 = A4;
int sensor6 = A5;
int value;
int Kp = 5, Ki = 0, Kd = 1;
int setPointKecepatan = 150;
int error, lastError, moveControl;

//external interrupt encoder
void doEncoderA()
{
  digitalRead(encoderPinB_1)?encoderMotor1--:encoderMotor2;
}

void setup(){
Serial.begin (9600);
  //Pin mode input sensor
  pinMode(sensor1, INPUT); //Set pin A0 sebagai input 
  pinMode(sensor2, INPUT); //Set pin A1 sebagai input
  pinMode(sensor3, INPUT); //Set pin A2 sebagai input
  pinMode(sensor4, INPUT); //Set pin A3 sebagai input
  pinMode(sensor5, INPUT); //Set pin A4 sebagai input
  pinMode(sensor6, INPUT); //Set pin A5 sebagai input  
 //pin mode driver motor 1 dan 2
  pinMode(encoderPinA_1, INPUT_PULLUP);
  pinMode(encoderPinB_1, INPUT_PULLUP);
  pinMode(encoderPinA_2, INPUT_PULLUP);
  pinMode(encoderPinB_2, INPUT_PULLUP);
  pinMode(pinEnable1, OUTPUT);
  pinMode(pinEnable2, OUTPUT);
  pinMode(motor_kiri1, OUTPUT);
  pinMode(motor_kiri2, OUTPUT);
  pinMode(motor_kanan1, OUTPUT);
  pinMode(motor_kanan2, OUTPUT);
  
  //Pin mode untuk tombol
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  
  attachinterrupt(digitalPinToInterrupt(encoderPinA1),
}

void loop(){
  if (//tombol button 1 ditekan 1 kali)
  kalibrasi ();
}  

    if(//tombol button 2 ditekan 2 kali){
     follow_line ();    
    }
void kalibrasi(){
  baca_sensor[0] = analogRead(sensor1);
  baca_sensor[1] = analogRead(sensor2);
  baca_sensor[2] = analogRead(sensor3);
  baca_sensor[3] = analogRead(sensor4);
  baca_sensor[4] = analogRead(sensor5);
  baca_sensor[5] = analogRead(sensor6);
  
  for (x=0; x<=5; x++){
    EEPROM.write(alamat, baca_sensor[x]);
    if (baca_sensor[x] > sensoMax[x] {
      sensorMax [x] = baca_sensor[x];
    //Untuk mengkalibrasi nilai sensor maksimum
  }
     if (baca_sensor[x] < sensoMin[x] {
      sensorMin [x] = baca_sensor[x];
    //Untuk mengkalibrasi nilai sensor minimum
  }
  //Variabel peka untuk menentukan titik tengah atau median
         peka[x] = (sensorMax[x] + sensorMin[x])/2;
         Serial.println(peka[x]);
         }
  
  /* EEPROM.write (alamat, baca_sensor[x]/4);
  value = EEPROM.read(alamat);
  Serial.print(alamat);
  Serial.print("\t");
  Serial.println(value);
  Serial.println();
  alamat = alamat+1;
  if (alamat = 512)
    address = 0;
  delay(100);
}*/
}
         
      void follow_line(){
           if (error = -3) {
             
           }
           if (error = -2) {
             
           }
           if (error = -1) {
             
           }
           if (error = 0) {
             
           }
           if (error = 1) {
             
           }
           if (error = 2) {
              
           }
        
           if (error = 3) {
             
           
        rate_d = error - lastError;
        rate_i = error + lastError;
        lastError = error;
        
        int moveControl = (kp * error) + (ki * rate_i) + (kd * rate_d);
        kecepatanmotorKiri = setPoint + moveControl; //Output duty cycle analogwrite untuk menggerakkan motor kiri
        kecepatanmotorKanan = setPoint - moveControl; //Untuk menggerakkan motor kanan
          
        Serial.println(encoderMotor1);//Untuk menampilkan grafik pada serial motor
        Serial.println(encoderMotor2);
      }
      }