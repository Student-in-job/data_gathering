#include "DAQ_Force.h"
#include <thread>

TaskHandle   taskHandle = 0;
Calibration* cal;		// struct containing calibration information
float64	     voltage[1000];
int	         FTCount = 0;
float        FT[MAX_VALUES];            // This array will hold the resultant force/torque vector.
double       sensorCalibSum[MAX_VALUES];
int          sensorCalibCount = 0;
int          biasNum = 10;
bool         biased;
bool         started = false;
bool         stoped = false;

int initATINano()
{
	if (CALIB_FILE == "")
	{
		printf("Calibration file not defined\n");
		return 0;
	}

	short sts;
	cal = createCalibration((char*)CALIB_FILE, 1);
	if (cal == NULL)
	{
		printf("\nSpecified calibration could not be loaded.\n");
		return -1;
	}
	sts = SetForceUnits(cal, (char*)"N");
	switch (sts)
	{
	case 0: break;	// successful completion
	case 1: printf("Invalid Calibration struct\n"); return -1;
	case 2: printf("Invalid force units\n"); return -1;
	default: printf("Unknown error\n"); return -1;
	}
	sts = SetTorqueUnits(cal, (char*)"N-m");
	switch (sts)
	{
	case 0: break;	// successful completion
	case 1: printf("Invalid Calibration struct\n"); return -1;
	case 2: printf("Invalid torque units\n"); return -1;
	default: printf("Unknown error\n"); return -1;
	}
	biased = true;

	printf("\nCalibration of sensor: Success\n");
	for (int index = 0; index < MAX_VALUES; index++)
		sensorCalibSum[index] = 0.0f;
	return 0;
}

int32 CVICALLBACK initDAQ()
{
	int32       error = 0;
	char        errBuff[2048] = { '\0' };

	DAQmxErrChk(DAQmxCreateTask("forceRead", &taskHandle));
	for (int index = 0; index < MAX_VALUES; index++)
		DAQmxErrChk(DAQmxCreateAIVoltageChan(taskHandle, CONCAT_DEVICE("ai" + static_cast<char>(index)), "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL));
	//DAQmxErrChk(DAQmxCreateAIVoltageChan(taskHandle, CONCAT_DEVICE("ai0"), "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL));
	//DAQmxErrChk(DAQmxCreateAIVoltageChan(taskHandle, CONCAT_DEVICE("ai1"), "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL));
	//DAQmxErrChk(DAQmxCreateAIVoltageChan(taskHandle, CONCAT_DEVICE("ai2"), "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL));
	//DAQmxErrChk(DAQmxCreateAIVoltageChan(taskHandle, CONCAT_DEVICE("ai3"), "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL));
	//DAQmxErrChk(DAQmxCreateAIVoltageChan(taskHandle, CONCAT_DEVICE("ai4"), "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL));
	//DAQmxErrChk(DAQmxCreateAIVoltageChan(taskHandle, CONCAT_DEVICE("ai5"), "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL));

	DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle, "", 1000.0, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 1));

	DAQmxErrChk(DAQmxRegisterEveryNSamplesEvent(taskHandle, DAQmx_Val_Acquired_Into_Buffer, 1, 0, EveryNCallback, NULL));
	DAQmxErrChk(DAQmxRegisterDoneEvent(taskHandle, 0, DoneCallback, NULL));

	/*********************************************/
	// DAQmx Start Code
	/*********************************************/
	DAQmxErrChk(DAQmxStartTask(taskHandle));

	//printf("Generating voltage continuously. Press Enter to interrupt\n");
	started = true;
	while(!stoped)
	{ }
	started = false;

Error:
	if (DAQmxFailed(error))
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
	if (taskHandle != 0)
	{
		/*********************************************/
		// DAQmx Stop Code
		/*********************************************/
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
	}
	if (DAQmxFailed(error))
		printf("DAQmx Error: %s\n", errBuff);
	return 1;
}

int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void* callbackData)
{
	int32       error = 0;
	char        errBuff[2048] = { '\0' };
	static int  totalRead = 0;
	int32       read = 0;

	/*********************************************/
	// DAQmx Read Code
	/*********************************************/
	DAQmxErrChk(DAQmxReadAnalogF64(taskHandle, 1, 10.0, DAQmx_Val_GroupByScanNumber, voltage, MAX_VALUES, &read, NULL));
	if (read > 0) {
		if (sensorCalibCount < biasNum) {
			for (int index = 0; index < MAX_VALUES; index++) {
				sensorCalibSum[index] = sensorCalibSum[index] + voltage[index];
			}
			sensorCalibCount++;
		}
		else {
			float volFloat[MAX_VALUES];
			for (int index = 0; index < MAX_VALUES; index++) {
				volFloat[index] = voltage[index];
			}
			if (sensorCalibCount == biasNum) {
				float mean[MAX_VALUES];
				for (int index = 0; index < MAX_VALUES; index++) {
					mean[index] = sensorCalibSum[index] / float(sensorCalibCount);
					sensorCalibSum[index] = 0.0f;
				}
				sensorCalibCount++;
				Bias(cal, mean);
				biased = false;
				printf("Bias calculation completed\n");
			}
			else if (sensorCalibCount > biasNum)
			{
				ConvertToFT(cal, volFloat, FT);
				doAction(FT);
			}
		}
	}

Error:
	if (DAQmxFailed(error)) {
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
		/*********************************************/
		// DAQmx Stop Code
		/*********************************************/
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
		printf("DAQmx Error: %s\n", errBuff);
		stoped = true;
	}
	return 0;
}

int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void* callbackData)
{
	int32   error = 0;
	char    errBuff[2048] = { '\0' };

	// Check to see if an error stopped the task.
	DAQmxErrChk(status);

Error:
	if (DAQmxFailed(error)) {
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
		DAQmxClearTask(taskHandle);
		printf("DAQmx Error: %s\n", errBuff);
	}
	return 0;
}
/// <summary>
/// Starts reading of data in separate thread
/// </summary>
void StartForceThread()
{
	if (started)
		return;
	started = false;
	std::thread forceThread(initDAQ);
	forceThread.detach();
}
/// <summary>
/// Stops current running thread
/// </summary>
void StopForceThread()
{
	stoped = true;
	while (started){}
}

void getData(float(&data)[MAX_VALUES])
{
	int32       error = 0;
	char        errBuff[2048] = { '\0' };
	static int  totalRead = 0;
	int32       read = 0;

	/*********************************************/
	// DAQmx Read Code
	/*********************************************/
	DAQmxErrChk(DAQmxReadAnalogF64(taskHandle, 1, 10.0, DAQmx_Val_GroupByScanNumber, voltage, MAX_VALUES, &read, NULL));

	if (read > 0)
	{
		float volFloat[MAX_VALUES];
		for (int index = 0; index < MAX_VALUES; index++) {
			volFloat[index] = voltage[index];
			ConvertToFT(cal, volFloat, data);
		}
	}

Error:
	if (DAQmxFailed(error)) {
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
		DAQmxClearTask(taskHandle);
		printf("DAQmx Error: %s\n", errBuff);
		stoped = true;
	}
}