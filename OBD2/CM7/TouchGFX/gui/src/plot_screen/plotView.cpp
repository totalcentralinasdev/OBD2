#include <gui/plot_screen/plotView.hpp>
#include "FreeRTOS.h"
#include "queue.h"

extern QueueHandle_t Sensor_Queue;
extern QueueHandle_t Sampling_size;

struct sensor {
	float voltage;
	float current;
};

struct sensor Sensor_data;
int current_update = 1, voltage_update = 1;

plotView::plotView() :
		plotview_current_callback(this, &plotView::dynamicGraph_ClickHandler), plotview_voltage_callback(
				this, &plotView::dynamicGraph_ClickHandler) {

}

void plotView::setupScreen() {
	plotViewBase::setupScreen();
	uint8_t sample_size = 2;
	xQueueOverwrite(Sampling_size, &sample_size);
	dynamicGraph_Current.setClickAction(plotview_current_callback);
	dynamicGraph_Voltage.setClickAction(plotview_voltage_callback);
}

void plotView::tearDownScreen() {
	plotViewBase::tearDownScreen();
}

void plotView::plot_update_exe() {
	if (xQueueReceive(Sensor_Queue, &Sensor_data, pdMS_TO_TICKS(10)) == pdPASS) {
		if (voltage_update) {
			dynamicGraph_Voltage.addDataPoint(Sensor_data.voltage);
		}
		if (current_update) {
			dynamicGraph_Current.addDataPoint(Sensor_data.current);
		}
	}
}

void plotView::dynamicGraph_ClickHandler(const GraphWrapAndOverwrite<100> &b,
		const ClickEvent &e) {
	if (e.getType() == touchgfx::ClickEvent::RELEASED) {
		if (&b == &dynamicGraph_Current) {
			current_update++;
			if (current_update > 1) {
				current_update = 0;
			}
		} else {
			if (&b == &dynamicGraph_Voltage) {
				voltage_update++;
				if (voltage_update > 1) {
					voltage_update = 0;
				}
			}
		}

	}

}
