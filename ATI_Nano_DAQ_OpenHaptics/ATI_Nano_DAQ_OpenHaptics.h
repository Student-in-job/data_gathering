#pragma once

#ifdef  _WIN64
#pragma warning (disable:4996)
#endif

#if defined(WIN32)
# include <conio.h>
#else
# include "conio.h"
#endif

#include <string.h>
#include <stdlib.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <fstream>
#include <iostream>
#include <chrono>
#include <time.h>
#include <vector>
#include <array>

HDCallbackCode HDCALLBACK deviceCallback(void* data);
int  initHD(void);
void DisplayData(float* position, float* data, double elapsed_time);
void FlushData();
bool Pause(char original);
void Info(std::string message);
void Error(std::string message);