#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/State.h>

#include <NewPing.h>

#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define SONAR_NUM 2

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(2, 3, MAX_DISTANCE), // Each sensor'se trigger pin, echo pin, and max distance to ping. 
  NewPing(4, 5, MAX_DISTANCE)
};

ros::NodeHandle  nh;

int value = 0;
bool ascend = true;
bool roverArmed = true;

void stateCallback(const mavros_msgs::State& msg){
  if(msg.armed)digitalWrite(13, HIGH);
  else digitalWrite(13, LOW);
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

void loop()
{
  if ( millis() >= range_time ){
    int r =0;

    range_msg.range = getRange_Ultrasound(0);
    range_msg.header.stamp = nh.now();
    pub_range1.publish(&range_msg);

    range_msg.range = getRange_Ultrasound(1);
    range_msg.header.stamp = nh.now();
    pub_range2.publish(&range_msg);
    
    range_time =  millis() + 50;
  }

  nh.spinOnce();
}
