#include <gui/screen1_screen/Screen1View.hpp>
#include "FreeRTOS.h"
#include "queue.h"


extern QueueHandle_t pin1;
extern QueueHandle_t pin3;
extern QueueHandle_t pin6;
extern QueueHandle_t pin7;
extern QueueHandle_t pin8;
extern QueueHandle_t pin9;
extern QueueHandle_t pin11;
extern QueueHandle_t pin12;
extern QueueHandle_t pin13;
extern QueueHandle_t pin14;
extern QueueHandle_t pin15;
extern QueueHandle_t pin16;



Screen1View::Screen1View()
{

}

void Screen1View::setupScreen()
{
    Screen1ViewBase::setupScreen();
}

void Screen1View::tearDownScreen()
{
    Screen1ViewBase::tearDownScreen();
}


void Screen1View::PIN1_exe(){
	uint8_t pin1_state;
	if(xQueuePeek(pin1, &pin1_state, 0) == pdPASS){
		pin1_state ^= 1;
		pin1_state = pin1_state & 1;
		xQueueOverwrite(pin1,&pin1_state);
	}
}

void Screen1View::PIN3_exe(){
	uint8_t pin3_state;
	if(xQueuePeek(pin3, &pin3_state, 0) == pdPASS){
		pin3_state ^= 1;
		pin3_state = pin3_state & 1;
		xQueueOverwrite(pin3,&pin3_state);
	}
}


void Screen1View::PIN6_exe(){
	uint8_t pin6_state;
	if(xQueuePeek(pin6, &pin6_state, 0) == pdPASS){
		pin6_state ^= 1;
		pin6_state = pin6_state & 1;
		xQueueOverwrite(pin6,&pin6_state);
	}
}

void Screen1View::PIN7_exe(){
	uint8_t pin7_state;
	if(xQueuePeek(pin7, &pin7_state, 0) == pdPASS){
		pin7_state ^= 1;
		pin7_state = pin7_state & 1;
		xQueueOverwrite(pin7,&pin7_state);
	}
}

void Screen1View::PIN8_exe(){
	uint8_t pin8_state;
	if(xQueuePeek(pin8, &pin8_state, 0) == pdPASS){
		pin8_state ^= 1;
		pin8_state = pin8_state & 1;
		xQueueOverwrite(pin8,&pin8_state);
	}
}

void Screen1View::PIN9_exe(){
	uint8_t pin9_state;
	if(xQueuePeek(pin9, &pin9_state, 0) == pdPASS){
		pin9_state ^= 1;
		pin9_state = pin9_state & 1;
		xQueueOverwrite(pin9,&pin9_state);
	}
}

void Screen1View::PIN11_exe(){
	uint8_t pin11_state;
	if(xQueuePeek(pin11, &pin11_state, 0) == pdPASS){
		pin11_state ^= 1;
		pin11_state = pin11_state & 1;
		xQueueOverwrite(pin11,&pin11_state);
	}
}

void Screen1View::PIN12_exe(){
	uint8_t pin12_state;
	if(xQueuePeek(pin12, &pin12_state, 0) == pdPASS){
		pin12_state ^= 1;
		pin12_state = pin12_state & 1;
		xQueueOverwrite(pin12,&pin12_state);
	}
}

void Screen1View::PIN13_exe(){
	uint8_t pin13_state;
	if(xQueuePeek(pin13, &pin13_state, 0) == pdPASS){
		pin13_state ^= 1;
		pin13_state = pin13_state & 1;
		xQueueOverwrite(pin13,&pin13_state);
	}
}


void Screen1View::PIN14_exe(){
	uint8_t pin14_state;
	if(xQueuePeek(pin14, &pin14_state, 0) == pdPASS){
		pin14_state ^= 1;
		pin14_state = pin14_state & 1;
		xQueueOverwrite(pin14,&pin14_state);
	}
}

void Screen1View::PIN15_exe(){
	uint8_t pin15_state;
	if(xQueuePeek(pin15, &pin15_state, 0) == pdPASS){
		pin15_state ^= 1;
		pin15_state = pin15_state & 1;
		xQueueOverwrite(pin15,&pin15_state);
	}
}


void Screen1View::PIN16_exe(){
	uint8_t pin16_state;
	if(xQueuePeek(pin16, &pin16_state, 0) == pdPASS){
		pin16_state ^= 1;
		pin16_state = pin16_state & 1;
		xQueueOverwrite(pin16,&pin16_state);
	}
}



void Screen1View::update_values_exe(){
	extern QueueHandle_t Sensor_Queue;

	struct sensor{
	    float voltage;
	    float current;
	};

	struct sensor SensorValues;

	if(xQueueReceive(Sensor_Queue, &SensorValues, pdMS_TO_TICKS(10)) == pdPASS){
		SensorValues.voltage = 10.856;
		Unicode::snprintfFloat(Voltage_textBuffer, VOLTAGE_TEXT_SIZE, "%.2f V",SensorValues.voltage);
		Unicode::snprintfFloat(Current_textBuffer, CURRENT_TEXT_SIZE, "%.2f A",SensorValues.current);
		Voltage_text.invalidate();
		Current_text.invalidate();
	}
}
