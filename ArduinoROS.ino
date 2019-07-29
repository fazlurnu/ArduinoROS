#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

#include <NewPing.h>

#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define SONAR_NUM 2

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(4, 5, MAX_DISTANCE), // Each sensor'se trigger pin, echo pin, and max distance to ping. 
  NewPing(6, 7, MAX_DISTANCE)
};

ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
ros::Publisher pub_range1( "/ultrasound1", &range_msg);
ros::Publisher pub_range2( "/ultrasound2", &range_msg);

char frameid[] = "/ultrasound";

float getRange_Ultrasound(uint8_t index){
  return (float(sonar[index].ping_cm())/100.0);
}

void setup()
{
  nh.initNode();
  nh.advertise(pub_range1);
  nh.advertise(pub_range2);
  
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 0.0;
  range_msg.max_range = 3.0;
}


long range_time;

void loop()
{
  
  //publish the adc value every 50 milliseconds
  //since it takes that long for the sensor to stablize
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
