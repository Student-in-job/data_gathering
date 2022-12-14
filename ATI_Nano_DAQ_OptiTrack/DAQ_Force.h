#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <NIDAQmx.h>
#include "ftconfig.h"
#include <string>

#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else
#define DAQNAME "Dev2/"
#define MAX_VALUES 6
#define CALIB_FILE "FT13162.cal"

#define CONCAT_DEVICE(p)    (char*)((DAQNAME + std::string(p)).c_str())

int   initATINano();
int32 CVICALLBACK initDAQ();
int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void* callbackData);
int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void* callbackData);
void  StartForceThread();
void  StopForceThread();
void  doAction(float* data);
void  getData(float (&data)[MAX_VALUES]);
//bool Pause(char original);