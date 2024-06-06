/*
 * irsensors.h
 *
 * Created: 5/5/2024 9:42:15 PM
 * Author: jlb
 */ 

#ifndef IRSENSORS_H_
#define IRSENSORS_H_

//Map Sensor Number to ADC Channel
#define SENSOR1 0	//Sensor connected to channel 0
#define SENSOR2 1	//Sensor connected to channel 1
#define SENSOR3 2	//Sensor connected to channel 2
#define SENSOR4 3	//Sensor connected to channel 3
#define SENSOR5 4	//Sensor connected to channel 4
#define SENSOR6 5	//Sensor connected to channel 5

//threshold
//Value = ((1.5v*1023)/5v)=306
#define SENSOR_THRES 750 //Sensor threshold

float read_sensors();

#endif /* IRSENSORS_H_ */