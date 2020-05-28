#include "define.h"
#include "esp_camera.h"
#include "camera_pins.h"
#include <WiFi.h>
#include <Wire.h>

const char* ssid = "Your AP SSID";
const char* password = "Your AP Password";

#ifdef LEFT_CAMERA
IPAddress local_IP(192, 168, 110, 254);
#elif RIGHT_CAMERA
IPAddress local_IP(192, 168, 110, 253);
#endif

IPAddress gateway(192, 168, 110, 1);

IPAddress subnet(255, 255, 255, 0);

void startCameraServer();

/*
typedef enum {
    FRAMESIZE_QQVGA,    // 160x120
    FRAMESIZE_QQVGA2,   // 128x160
    FRAMESIZE_QCIF,     // 176x144
    FRAMESIZE_HQVGA,    // 240x176
    FRAMESIZE_QVGA,     // 320x240
    FRAMESIZE_CIF,      // 400x296
    FRAMESIZE_VGA,      // 640x480
    FRAMESIZE_SVGA,     // 800x600
    FRAMESIZE_XGA,      // 1024x768
    FRAMESIZE_SXGA,     // 1280x1024
    FRAMESIZE_UXGA,     // 1600x1200
    FRAMESIZE_QXGA,     // 2048*1536
    FRAMESIZE_INVALID
} framesize_t;
*/

#include <VL53L1X.h>

unsigned int ToF = 0;
VL53L1X sensor;

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"

int16_t ax = 0, ay = 0, az = 0;
int16_t gx = 0, gy = 0, gz = 0;
unsigned long mpu_update_time = 0;
double roll = 0, pitch = 0, yaw = 0;
double halfT = 0.002f;
MPU6050 accelgyro;


volatile unsigned int M1_POS = 90;
volatile unsigned int M2_POS = 90;

void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 180) {
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / valueMax) * min(value, valueMax);
  ledcWrite(channel, duty);
}

TaskHandle_t ServoTask;
void ServoTaskcode( void * pvParameters){
  while(true){
    //Serial.print(M1_POS);Serial.print(",");Serial.println(M2_POS);
    ledcAnalogWrite(2, M1_POS);
    ledcAnalogWrite(4, M2_POS); 
    //controlServos();
    delay(20); 
  }
}

int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;
void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
 
  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration(){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;
 
  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  while (1){
    int ready=0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);
 
    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);
 
    meansensors();
    Serial.println("...");
 
    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;
 
    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;
 
    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;
 
    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);
 
    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);
 
    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);
 
    if (ready==6) break;
    //if (ready==3) break;
  }
}

bool int_flag = false;
static void IRAM_ATTR FUNC1(void *arg)
{
  int_flag = true;
}


void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 12;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, /*FRAMESIZE_VGA*/FRAMESIZE_SVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  WiFi.begin(ssid, password);  
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  Wire.begin(15,13);

#ifdef LEFT_CAMERA
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    //while (1);
  }
  else{
    sensor.setDistanceMode(VL53L1X::Long);
    sensor.setMeasurementTimingBudget(50000);
    sensor.startContinuous(50);
  }

#elif RIGHT_CAMERA
  accelgyro.initialize();
  if(!accelgyro.testConnection()){
    Serial.println("Failed to detect and initialize MPU6050!");
    mpu_update_time = micros();
  }
  else{
    meansensors();
    calibration();
  }
  ledcSetup(2, 50, 16); //channel, freq, resolution
  ledcAttachPin(M1, 2); // pin, channel
  ledcSetup(4, 50, 16);
  ledcAttachPin(M2, 4);  // Ai-Thinker: pins 2 and 12  
  xTaskCreatePinnedToCore(ServoTaskcode,"ServoTask",10000,NULL,1,&ServoTask,0);
/*
 gpio_pad_select_gpio(GPIO_NUM_16);
 gpio_set_direction(GPIO_NUM_16, GPIO_MODE_INPUT);
 err = gpio_isr_handler_add(GPIO_NUM_16, &FUNC1, (void *) 16);
 if (err != ESP_OK) {
  Serial.printf("handler add failed with error 0x%x \r\n", err);
 }
 err = gpio_set_intr_type(GPIO_NUM_16, GPIO_INTR_NEGEDGE);
 if (err != ESP_OK) {
  Serial.printf("set intr type failed with error 0x%x \r\n", err);
 }
 gpio_pulldown_en(GPIO_NUM_16);
 gpio_pullup_dis(GPIO_NUM_16);
 gpio_intr_enable(GPIO_NUM_16);
 */
#endif

  pinMode(RED_LED,OUTPUT);
  digitalWrite(RED_LED,LOW);      
}

int count = 0;
void loop() { 

#ifdef LEFT_CAMERA
  ToF = sensor.read();
  Serial.print(ToF);
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();  
#elif RIGHT_CAMERA
  if(int_flag){
    Serial.print(count++);
    Serial.println("Detect");
    int_flag = false;
  }
  
  if(accelgyro.testConnection()){


    double dt = (micros() - mpu_update_time) / 1000000.0f;
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    mpu_update_time = micros();

    halfT = dt / 2.0f;
    
    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz); Serial.print("\t");
    Serial.print(roll,4); Serial.print("\t");
    Serial.print(pitch,4); Serial.print("\t");
    Serial.print(yaw,4); Serial.print("\t");
    Serial.println(dt,4);
   
  }
#endif

/*
    bool coded[18] = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0};
    for(int i = 0;i < 18;++i){
      if(coded[i]){
        digitalWrite(LASER2, HIGH); 
      }
      else{
        digitalWrite(LASER2, LOW); 
      }
      delay(20);
    }

    delay(1000);
    */
}
