#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>
#include "FreeRTOS.h"

extern uint8_t g_CurrentTaskName;

Model::Model() : modelListener(0)
{

}

void Model::tick()
{
	g_CurrentTaskName = 3;
}
