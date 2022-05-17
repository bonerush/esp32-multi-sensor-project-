
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
/******************************************************************************************/
#define DHTPIN   4   //连接DHT传感器的数字引脚
#define DHTTYPE    DHT11     // DHT 11
/***********MQ-2************/
#define  MQ2_analogPin  35
#define  MQ2_digitalPin  32 
/***********Earth************/
#define EarthHumiDi 3 //土壤传感器数字输入
#define EarthHumiAn  34 //土壤传感器模拟输入
/***********ligth************/
#define LigthAnPin 33//光敏的模拟数值读取引脚
/******************************************************************************************/
char* sensortopic="senordata";
/*************************************/
uint32_t MQaramNum;//阈值
unsigned int mqaver=0;//烟雾传感器平均值
void* mqaram;//烟雾报警信号
TaskHandle_t MQtask_Handle=NULL;//mq任务句柄
  static char*MqTopic="MQTASk";
  unsigned int MQaver=0;
  unsigned char MQi=0;
  unsigned int MQsum=0;
  unsigned int astate=0;
  unsigned int dstate=0;
  unsigned char MQaram=0;
/*************************************/
unsigned int humidityStatus=0;
unsigned int humidityAnalog=0;
TaskHandle_t EarthHumidityTask_Handle=NULL;
/*************************************/
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;
sensor_t sensor;//创建sensor对象
float DHTTemperture;//DHT温度
float DHTHumidity;//DHT湿度
TaskHandle_t DHTtask_Handle=NULL;//DHT任务句柄
/*************************************/
unsigned int LightAnalogRead;
TaskHandle_t LigthTask_Handle=NULL;//光敏传感任务句柄
unsigned char LightState;
/*************************************/
WiFiClient espClient;
PubSubClient mqttClient(espClient);
const char *ssid="...";
const char*psw="...";
const char*mqttServer="...";
char json_string[4096];
/******************************************************************************************/
void MQTask(void *MQTask);
void DHTTask(void *DHTTask);
void EarthHumidityTask(void *EarthHumidityTask);
void LigthTask(void *LigthTask);
void WifiSetup(const char *ssid,const char *psw);
void MQTTSetup();
void MQTTPublish(char* Topic, String message);
void getdata(unsigned int mq,float tem,float hum,unsigned int light,unsigned char LightState,unsigned int earth,unsigned int humidityStatus);
/******************************************************************************************/
void setup() 
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WifiSetup(ssid,psw);//wifi的初始设置函数
  MQTTSetup();
  xTaskCreate(MQTask,"MQTask",1024,(void*) MQTask,3,&MQtask_Handle);
  xTaskCreate(DHTTask,"DHTTask",1024,(void*) DHTTask,3,&DHTtask_Handle);
  xTaskCreate(EarthHumidityTask,"EarthHumidityTask",1024,(void*) EarthHumidityTask,3,&EarthHumidityTask_Handle);
  xTaskCreate(LigthTask,"LigthTask",1024,(void*) LigthTask,3,&LigthTask_Handle);
}

void loop() 
{
  getdata(MQaver,DHTTemperture,DHTHumidity,LightAnalogRead,LightState,humidityAnalog,humidityStatus);
  MQTTPublish(sensortopic,json_string);
  delay(1000);
}
/******************************************************************************************/
void MQTask(void *MQTask) 
{
  pinMode(MQ2_analogPin,INPUT);
  pinMode(MQ2_digitalPin,INPUT);
  while(1)
  {
    astate=analogRead(MQ2_analogPin );
    dstate=digitalRead(MQ2_digitalPin ); 
    // Serial.println("astate");
    // Serial.println(astate);
    if(MQi==0)
    MQsum=astate;//如果计数器在一开始，那么总值就直接等于模拟值
    else if(MQi<=49)
    {    
    MQsum=MQsum+astate;//如果不是的话就依次累加
    MQaver=MQsum/(MQi+1);//算五十次平均数
      if(MQi==49)
      {
        MQsum=0;
        MQi=0;
        mqaver=MQaver;
        Serial.println("/***********MQ************/");
        Serial.println("MQaver");
        Serial.println(MQaver);
      }
    }
    else if(MQi%4==0)
    {
      if(MQaver>=MQaramNum)
      {
        MQaram=1;
        mqaram=&MQaram;
        vTaskPrioritySet(MQtask_Handle,24);
      }
      else
      {
        vTaskPrioritySet(MQtask_Handle,1);
      }
    }
    MQi++;
    vTaskDelay(100/portTICK_PERIOD_MS);
  }
}
/******************************************************************************************/
void DHTTask(void *DHTTask)
{
  dht.begin();//dht的初始化
  sensors_event_t event;
  dht.temperature().getSensor(&sensor);//调用传感器类型数据
  delayMS = sensor.min_delay / 1000;
  while(1)
  {
  Serial.println("/***********DHT************/");
  delay(delayMS);
  //获取温度事件并打印其值。
  dht.temperature().getEvent(&event);//这里是调用dht对象温度数据
  Serial.println("temp");
  Serial.println(event.temperature);
  DHTTemperture=event.temperature;
  dht.humidity().getEvent(&event);//这里是调用dht对象湿度数据
  Serial.println("humi");
  Serial.println(event.relative_humidity);
  DHTHumidity=event.relative_humidity;
  vTaskDelay(5000/portTICK_PERIOD_MS);
  }
}
/******************************************************************************************/
void EarthHumidityTask(void *EarthHumidityTask)
{
  dht.begin();//dht的初始化
  while(1)
  {
  Serial.println("/***********EARTH************/");
  humidityStatus = digitalRead(EarthHumiDi);
  Serial.print("EARTHState:");
  if(humidityStatus){
    Serial.println("Dry");
  }else{
    Serial.println("Moist");
  }
  humidityAnalog = analogRead(EarthHumiAn);
  Serial.print("EARTHHumidity:");
  Serial.println(humidityAnalog);

  vTaskDelay(5000/portTICK_PERIOD_MS);
  }
}
/******************************************************************************************/
void LigthTask(void *LigthTask)
{
  while(1)
  {
  Serial.println("/***********LIGHT************/");
  pinMode(LigthAnPin,INPUT);
  LightAnalogRead=analogRead(LigthAnPin);
  
  if(LightAnalogRead>=400)
  {
    Serial.println("light---LOW");
    LightState=0;
  }
  else
  {
    Serial.println("light---HIGH");
    LightState=1;
  }
  Serial.println("lightread");
  Serial.println(LightAnalogRead);
  // MQTTPublish(LighTopic,Lightmessage );
   vTaskDelay(5000/portTICK_PERIOD_MS);
  }
}
/******************************************************************************************/
void WifiSetup(const char *ssid,const char *psw)
{
  WiFi.begin(ssid,psw);
  while(WiFi.status() !=WL_CONNECTED)
  {
    delay(30);
    Serial.print(".");
  }
  Serial.println("wifi connected");
  Serial.println("IP adress:");
  Serial.println(WiFi.localIP());
}
/******************************************************************************************/
void MQTTSetup()
{
  mqttClient.setServer(mqttServer,1883);
  String ClientId="esp32"+WiFi.macAddress();
  if(mqttClient.connect(ClientId.c_str()))
  {
    Serial.println("MQTT Server Connected.");
    Serial.println("Server Address: ");
    Serial.println(mqttServer);
    Serial.println("ClientId:");
    Serial.println(ClientId);
  } 
  else 
  {
    Serial.print("MQTT Server Connect Failed. Client State:");
    Serial.println(mqttClient.state());
    delay(3000);
  }   
}
/******************************************************************************************/
void MQTTPublish(char* Topic, String message)
{
  String topicString =Topic+WiFi.macAddress();
  char publishTopic[topicString.length() + 1];  
  strcpy(publishTopic, topicString.c_str());

  String messageString =message;
  char publishMsg[messageString.length() + 1];   
  strcpy(publishMsg, messageString.c_str());

   if(mqttClient.publish(publishTopic, publishMsg))
    {
    Serial.println("Publish Topic:");Serial.println(publishTopic);
    Serial.println("Publish message:");Serial.println(publishMsg);  
    }
   else 
    {
    Serial.println("Message Publish Failed."); 
    }
}
/******************************************************************************************/
void getdata(unsigned int mq,float tem,float hum,unsigned int light,unsigned char LightState,unsigned int earth,unsigned int humidityStatus)
{
  DynamicJsonDocument data(4096);
  data["MQ"]=mq;
  data["temp"]=tem;
  data["humi"]=hum;
  data["Lightread"]=light;
  data["LightState"]=LightState;
  data["earth"]=earth;
  data["earthstate"]=humidityStatus;
  serializeJson(data,json_string);
}
/******************************************************************************************/
void egg()
{
  //hellooooo =v=
  //I LOVE  WYW
  //开始躺平
}

