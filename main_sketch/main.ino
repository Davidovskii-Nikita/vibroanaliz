#include <ESP8266WiFi.h> 
#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <Wire.h>     
#include <ArduinoJson.h> 
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ESP8266HTTPClient.h>
WiFiUDP ntpUDP;

#include <ESP8266HTTPUpdateServer.h>
//------------------------------------------------------------------------------------------------------------------
NTPClient timeClient(ntpUDP, "by.pool.ntp.org", 10800);// выбор сервера NTP("by.pool.ntp.org"), cмещение пояса(10800)

#define PERIOD_A 1000// частота опроса СКЗ ускорения 3 оси
#define PERIOD_T 10000// частота опроса температуры  

#define RANGE_A 100//колличество данных опроса в 1 цикл по ускорению
#define RANGE_T 10//колличество данных опроса в 1 цикл по температуре

// Настройка фильтра бегущего среднего 
//filteredValue = newValue * FILTER_COEF + filteredValue * (1 – FILTER_COEF)
#define FILTER_STEP 2// время вызова фильтра
#define FILTER_COEF 0.5// коэффициент фильтра

// Настройка фильтра Калмана
float errmeasure = 10; // разброс измерения
float errestimate = 40;  // разброс оценки
float q = 1;  // скорость изменения значений

// Настройка моды
const int range=100;// объем колличества значений в выборке

const char* ssid = "SSID"; //имя Wi-Fi сети
const char* password = "PASS";  //пароль cети
const char* host ="host.com";// адрес хоста
String URL="host.com/data";// адрес куда отправляются POST запросы
const char* update_path = "/firmware";
const char* update_username = "admin";// логин для OTA-обновлений
const char* update_password = "admin";// пароль для OTA-обновлений
const char* host_OTA = "esp-01_black";// название устройства в локальной сети для прошивки через браузер 
// в виде http://esp-01_black.local/firmware/ 

const uint16_t Full_Scale_Range=4;// выбор диапазона измерений акселерометра +-2,+-4,+-8,+-16
//-----------------------------------------------------------------------------------------------------------------

int val;
float val_f;
unsigned long filter_timer;
float currentestimate = 0.0;
float lastestimate = 0.0;
float kalmangain = 0.0;
unsigned long timer_A, timer_T;
int countA=0;
int countT=0;
String opros_axel [RANGE_A];
String opros_axel_time [RANGE_A];
String opros_temp [RANGE_T];
String opros_temp_time [RANGE_T];
uint16_t AccelScaleFactor;
const uint8_t MPU6050SlaveAddress = 0x68;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_TEMP =  0x41;
int16_t AccelX, AccelY, AccelZ, Temperature;
double Ax, Ay, Az, T;
String MAC;
double val_filter;
float r1,r2,r3,r4,r5,r6,r7,h;

size_t capacity;

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;
  
void setup() 
{
  init_moda(Full_Scale_Range);
  
  switch (Full_Scale_Range)
  {
    case 2:
      AccelScaleFactor=16384;
        break;
    case 4:
      AccelScaleFactor=8192;
        break;
    case 8:
      AccelScaleFactor=4096;
        break;
    case 16:
      AccelScaleFactor=2048;
        break;
  }

  Wire.begin(0,2);
  Serial.begin(9600);
  WiFi.mode(WIFI_AP_STA);
  delay(10);
 

  while (WiFi.waitForConnectResult() != WL_CONNECTED) 
  {
    WiFi.begin(ssid, password);
    Serial.println("WiFi failed, retrying.");
    delay(100);
  }

  MDNS.begin(host_OTA);

  httpUpdater.setup(&httpServer, update_path, update_username, update_password);
  httpServer.begin();

  MDNS.addService("http", "tcp", 80);
  
  MPU6050_Init();
  timeClient.begin();
  MAC=WiFi.macAddress();
  capacity= 2*JSON_ARRAY_SIZE(RANGE_T ) + 2*JSON_ARRAY_SIZE(RANGE_A) + JSON_OBJECT_SIZE(5) + 3500;

}

void loop() 
{
  httpServer.handleClient();
  MDNS.update();
  filter();
  if (millis() - timer_A >= PERIOD_A) 
  { 
    timer_A = millis();   
    if(countA<RANGE_A)
    {
      String r_c_time=getTime();
      String Z=String(val_filter);
      opros_axel_time[countA]=r_c_time;
      opros_axel[countA]=Z;
      countA++;
    }
  }
  if (millis() - timer_T >= PERIOD_T)
  {
    timer_T = millis();
     if(countT<RANGE_T)
     {
      Read_RawValue_TEMP(MPU6050SlaveAddress, MPU6050_REGISTER_TEMP);
      T = (double)Temperature/340+36.53; //temperature formula
      String Temp=String(T);
      String r_c_time=getTime();
      opros_temp_time[countT]=r_c_time;
      opros_temp[countT]=Temp;
      countT++;
      /*
      Serial.print(capacity);
      Serial.println('\t');
      Serial.print(ESP.getMaxFreeBlockSize());
      Serial.println('\t');
      Serial.print(countT);
      Serial.println('\t');
      */
     }
  }
  if(countA==RANGE_A)
  {
    if(countT==RANGE_T)
    { 
     post_json();
     countA=0;
     countT=0;
    }
  }
}

double filter()
{
  double Z=Read_RawValue_AXEL(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  // Для того, что бы включить фильтр, требуестья закоментировать 
  //строку ниже,и раскоментировать cтроку с интересующим фитром.
  val_filter=Z;
  //val_filter=filter_moda(Z);
  //val_filter=filter_kalman(Z);
 // val_filter=running_midle(Z);
  return val_filter;
}

void post_json(void)
{
  WiFiClient client;
  HTTPClient http;
 if (client.connect(host,80))
      {
        String buffer;
        DynamicJsonDocument jsonDocument(capacity);
        jsonDocument["MAC"] = MAC;
        jsonDocument["CAP"] = capacity;
        jsonDocument["MEM"] = ESP.getMaxFreeBlockSize();
        JsonArray Axel_time = jsonDocument.createNestedArray("Axel_time");
        JsonArray Axel = jsonDocument.createNestedArray("Axel");
        JsonArray Temp_time = jsonDocument.createNestedArray("Temp_time");
        JsonArray Temp = jsonDocument.createNestedArray("Temp");

        int i=0;
        int j=0;
        for(i=0;i<RANGE_T; i++)
        {
          Temp_time.add(opros_temp_time[i]);
          Temp.add(opros_temp[i]);
        }
   
        for(j=0;j<RANGE_A; j++)
        {
          Axel_time.add(opros_axel_time[j]);
          Axel.add(opros_axel[j]);
        }
        serializeJson(jsonDocument, buffer );
        
        http.begin(client,URL); 
        http.addHeader("Content-Type", "application/x-www-form-urlencoded");    
        http.POST(buffer);
        http.end();
        
        Serial.println("Отправляю! "); 
      }
      else
      {
        Serial.println("Не удалось отправить,пробелмы с подлкючлением!");
      }
}

double Read_RawValue_AXEL(uint8_t deviceAddress, uint8_t regAddress)
{
  double sum;
  double root;
  double SKZ;
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)6);
  AccelX = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelY = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());
  Ax=(double)AccelX/AccelScaleFactor;
  Ay=(double)AccelY/AccelScaleFactor;
  Az=(double)AccelZ/AccelScaleFactor;
  Ax=Ax*Ax;
  Ay=Ay*Ay;
  Az=Az*Az;
  sum=Ax+Ay+Az;
  root=sum/3.0;
  SKZ=pow(root,0.5);
  return SKZ;
}

void Read_RawValue_TEMP(uint8_t deviceAddress, uint8_t regAddress)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)2);
  Temperature = (((int16_t)Wire.read()<<8) | Wire.read());
}

String getTime()
{
  timeClient.update();
  String data = String(timeClient.getEpochTime());
  return(data);
}

void MPU6050_Init()
{
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x08);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

void init_moda(float diap)
{
  float step_r =diap/8;
  r1=step_r;
  r2=r1+step_r;
  r3=r2+step_r;
  r4=r3+step_r;
  r5=r4+step_r;
  r6=r5+step_r;
  r7=r6+step_r;
  float k=1.0+3.33211*log10(range);
  h=diap/k;
}

float filter_moda(float Z)
{
  int countr = 0;
  float Nm1=0;
  float Nm2=0;
  float Nm3=0;
  float Nm4=0; 
  float Nm5=0;
  float Nm6=0;
  float Nm7=0;
  float Nm8=0; 
  int mx=0;
  float M0;
  
  while (countr<range)
  {
    if (Z<=r1)
    {
     Nm1++;
    }
    else if (Z<=r2 && Z>r1)
    {
     Nm2++;
    }
    else if (Z<=r3 && r2)
    {
      Nm3++;
    }
    else if (Z<=r4 && Z>r3)
    {
     Nm4++;
    }
    else if (Z<=r5 && Z>r4)
    {
     Nm5++;;
    }
    else if (Z<=r6 && Z>r5)
    {
     Nm6++;
    }
    else if (Z<=r4 && Z>r6)
    {
     Nm7++;
    }
    else if (Z<=Full_Scale_Range && Z>r7)
    {
     Nm8++;
    }
  countr++;
  }
  
  float max_freq[]={Nm1,Nm2,Nm3,Nm4,Nm5,Nm6,Nm7,Nm8};
  int j=0;
  
  for(j=0; j<9;j++)
  {
    if(max_freq[j]>max_freq[mx])
    {
      mx=j;
    }
  }
  if(mx==0)
  {
   M0=(Nm1/((Nm1-Nm2)+Nm1))*h;
  }
  else if(mx==1)
  {
   M0=r1+(((Nm2-Nm1)/((Nm2-Nm1)+(Nm2-Nm3)))*h);
  }
  else if(mx==2)
  {
  M0=r2+((Nm3-Nm2)/((Nm3-Nm2)+(Nm3-Nm4)))*h;
  }
  else if (mx==3)
  {
   M0=r3+((Nm4-Nm3)/((Nm4-Nm3)+(Nm4-Nm5)))*h;
  }
  else if(mx==4)
  {
   M0=r4+(((Nm5-Nm4)/((Nm5-Nm4)+(Nm5-Nm6)))*h);
  }
  else if(mx==5)
  {
  M0=r5+((Nm6-Nm5)/((Nm6-Nm5)+(Nm6-Nm7)))*h;
  }
  else if (mx==6)
  {
  M0=r6+((Nm7-Nm6)/((Nm7-Nm6)+(Nm7-Nm8)))*h;
  }
  else if (mx==7)
  {
   M0=r7+(Nm8/(Nm8+(Nm8-Nm7)))*h;
  }
   countr=0;
   mx=0;
   return(M0);
}

float filter_kalman(float value) 
{
  kalmangain = errestimate / (errestimate + errmeasure);
  currentestimate = lastestimate + kalmangain * (value - lastestimate);
  errestimate =  (1.0 - kalmangain) * errestimate + fabs(lastestimate - currentestimate) * q;
  lastestimate = currentestimate;
  return currentestimate;
}

float running_midle(float Z)
{
  if (millis() - filter_timer > FILTER_STEP) {
    filter_timer = millis();
    val_f = Z * FILTER_COEF + val_f * (1 - FILTER_COEF);
  }
  return(val_f);
}
