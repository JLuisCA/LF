/*
 * line_follower.c
 *
 * Created: 6/5/2024 7:46:57 PM
 * Author : DoniM
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include "motors.h"
#include "adc.h"
#include "leds.h"
#include "irsensors.h"

////////Variables generales///////
float setpoint;
float Kp = 2.5;	
float Ki =  5.5;	
float Kd =  0.001;	
int32_t errorI = 0;	
int32_t errorP  =0;		
float control;

////implentacion del PID/////
float pid(float cur_value,float req_value)
{
	float pid;
	float error;

	error = req_value - cur_value;
	pid = (Kp * error)  + (Ki * errorI) + (Kd * (error - errorP));

	errorI += error;                  // La integral es la suma simple del error
	errorP = error;                    // Guarda el error anterior

	return pid;
}

int main(void)
{
    led_init();
    test_leds();
	motors_init();
	adc_init();
	//Leyendo el punto del sensor anterior
	float setpoint_anterior=0.0;
	
    while (1) 
    {
		////////////// La funcion que tiene el set point es con la suma ponderada sacar el valor de los dos leds de intermedio
		////////////// Lo cual al hacer la suma ponderada de 1 al 6, la mitad es 3.5 teniendo tanto el sensor 3 y 4 prendidos detectando el valor
		setpoint=read_sensors();
		
		///////Si el sensor no encuentra ninguna linea regresa a el valor anterior
		if(setpoint==0xFF)
		{
			setpoint=setpoint_anterior;
		}
		
		/////////// El algoritmo de control es decirle a PID, que cuando  se encuentre en medio del setpoint o el 3.5  regrese a tomar las funciones vistas en el PID
		control = pid(setpoint,3.5);
		
		//////La variable control va de el limite de los motores, al tener la funcion del motor derecho e izquierdo
		////// y cada motor al valer 255, se hace la suma 255+255=510 teniendo en cuenta el funcionamiento o la 
		//////potencia de ambos motores
		if(control > 510)
		control = 510;
		if(control < -510)
		control = -510;
		//Esta funcion es para el sensor izquierdo al no tener la linea este debe controlar su velocidad para corregir
		if(control >= 0.0)
		{
			if(control>255)
			motor_left(MOTOR_BACKWARD,control-255);
			else
			motor_left(MOTOR_FORWARD,255-control);
			
			motor_right(MOTOR_BACKWARD,255);
		}
		//Esta funcion es para el sensor derecha al no tener la linea este debe controlar su velocidad para corregir
		if(control <= 0.0)
		{
			if(control<-255)
			motor_right(MOTOR_FORWARD,-(control+255));
			else
			motor_right(MOTOR_BACKWARD,255+control);
			
			motor_left(MOTOR_FORWARD,255);
		}
		
		setpoint_anterior=setpoint;
		
    }
}