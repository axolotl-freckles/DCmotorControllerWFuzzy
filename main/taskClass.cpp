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
_taskName(name), _stackSize(stack_size), _priority(prio), _taskHandle(NULL)
{ }
Task::~Task() {
	vTaskSuspend(_taskHandle);
	vTaskDelete(_taskHandle);
	_taskHandle = nullptr;
}
void Task::start() {
	if (_taskHandle == NULL) {
		xTaskCreate(
			// taskFunctionWrapper,
			[] (void * args) {((Task*)args)->taskFunction();},
			_taskName,
			_stackSize,
			this,
			_priority,
			&_taskHandle
		);
	}
	else {
		vTaskResume(_taskHandle);
	}
}
void Task::stop() {
	vTaskSuspend(_taskHandle);
}
}
#endif
