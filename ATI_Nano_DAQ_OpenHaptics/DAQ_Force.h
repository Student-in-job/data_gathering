#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <NIDAQmx.h>
#include "ftconfig.h"
#include <string>

#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

#define MAX_VALUES 6
#define DAQNAME "Dev2/"
//#define CALIB_FILE "FT21423.cal"
#define CALIB_FILE "FT13162.cal"
#define CONCAT_DEVICE(p)    (char*)((DAQNAME + std::string(p)).c_str())

// Uncomment next define to receive 1000 Hz asynchronous read
//#define ASYNC_READ 1;

int   initATINano();
void  removeBias();
int32 CVICALLBACK initDAQ();
void  StartForceThread();
void  StopForceThread();
#ifdef ASYNC_READ
int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void* callbackData);
int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void* callbackData);
void  doAction(float* data);
#else
void  getData(float (&data)[MAX_VALUES]);
#endif