#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/State.h>

#include <NewPing.h>
#include <Adafruit_NeoPixel.h>

#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define SONAR_NUM 2

#define PIXEL_PIN 6
#define PIXEL_COUNT 7 //or 87

Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
uint8_t   brightness;
uint16_t  brg     = 200;
uint32_t  RED     = strip.Color(brg,  0,  0);
uint32_t  GREEN   = strip.Color(  0,brg,  0);
uint32_t  BLUE    = strip.Color(  0,  0,brg);
uint32_t  YELLOW  = strip.Color(brg,brg,  0);
uint32_t  WHITE   = strip.Color(brg,brg,brg);
uint32_t  BLACK   = strip.Color(  0,  0,  0);

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(2, 3, MAX_DISTANCE), // Each sensor'se trigger pin, echo pin, and max distance to ping. 
  NewPing(4, 5, MAX_DISTANCE)
};

ros::NodeHandle  nh;

int value = 0;
bool ascend = true;
bool roverArmed = false;

void stateCallback(const mavros_msgs::State& msg){
  roverArmed = msg.armed;
}

sensor_msgs::Range range_msg;
ros::Publisher pub_range1( "/ultrasound1", &range_msg);
ros::Publisher pub_range2( "/ultrasound2", &range_msg);
ros::Subscriber<mavros_msgs::State> sub("/mavros/state", &stateCallback);
char frameid[] = "/ultrasound";

float getRange_Ultrasound(uint8_t index){
  return (float(sonar[index].ping_cm())/100.0);
}

void setup()
{
  initLED();
  
  pinMode(13, OUTPUT);
  
  nh.initNode();
  nh.advertise(pub_range1);
  nh.advertise(pub_range2);
  nh.subscribe(sub);
  
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 0.0;
  range_msg.max_range = 3.0;
}

long range_time = 0;
long led_time = 0;
bool led_on = true;

void loop(){
  if(nh.connected()){
    if ( millis() >= range_time ){
      int r =0;
  
      range_msg.range = getRange_Ultrasound(0);
      range_msg.header.stamp = nh.now();
      pub_range1.publish(&range_msg);
  
      range_msg.range = getRange_Ultrasound(1);
      range_msg.header.stamp = nh.now();
      pub_range2.publish(&range_msg);
      
      range_time =  millis() + 50;
      if(roverArmed){
        colorWipe(BLUE);
      } else blinking(YELLOW, 250);
    }
  } else colorWipe(RED);
  
  nh.spinOnce();
}

void colorWipe(uint32_t color){
  for(int i=0; i<strip.numPixels(); i++){
    strip.setPixelColor(i, color);
    strip.show();
    delay(1);
  }
}

void blinking(uint32_t color, uint32_t wait){
  if ( millis() >= led_time ){
    if(led_on){
      colorWipe(BLACK);
      digitalWrite(13, LOW);
      led_on = false;
    }
    else{
      colorWipe(color);
      digitalWrite(13, HIGH);
      led_on = true;
    }

    led_time = millis + wait;
  }
}

void initLED(){
  strip.begin();
  strip.show();
  strip.setBrightness(50);
  blinking(RED, 150);
}
