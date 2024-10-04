// #pragma once
#ifndef TASKCLASS_CPP
#define TASKCLASS_CPP

#include "taskClass.hpp"
#include "esp_mac.h"
extern "C" {

void taskFunctionWrapper(void* taskClass) {
	Task *caller = (Task*)taskClass;
	caller->taskFunction();
}

Task::Task(const char* name, uint32_t stack_size, UBaseType_t prio) :
taskName(name), stackSize(stack_size), priority(prio), taskHandle(NULL)
{ }
Task::~Task() {
	vTaskSuspend(taskHandle);
	vTaskDelete(taskHandle);
}
void Task::start() {
	if (taskHandle == NULL) {
		xTaskCreate(
			// taskFunctionWrapper,
			[] (void * args) {((Task*)args)->taskFunction();},
			taskName,
			stackSize,
			this,
			priority,
			&taskHandle
		);
	}
	else {
		vTaskResume(taskHandle);
	}
}
void Task::stop() {
	vTaskSuspend(taskHandle);
}
}
#endif
