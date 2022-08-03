#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <NIDAQmx.h>
#include "ftconfig.h"
#include <string>

#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

#define MAX_VALUES 6
#define DAQNAME "Dev4/"
//#define CALIB_FILE "FT21423.cal"
#define CALIB_FILE "FT13162.cal"
#define CONCAT_DEVICE(p)    (char*)((DAQNAME + std::string(p)).c_str())

int initATINano();
int32 CVICALLBACK initDAQ();
int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void* callbackData);
int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void* callbackData);
void doAction(float* data);
void StartForceThread();
void StopForceThread();
void getData(float (&data)[MAX_VALUES]);