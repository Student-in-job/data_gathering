// ATI_Nano_DAQ_OpenHaptics.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include "ATI_Nano_DAQ_OpenHaptics.h"
#include "DAQ_Force.h"

#define FILE "./data"
#define EXTENSION ".csv"

bool chosen;
extern bool started;
char r = '1';
std::ofstream myfile;
bool first = true;
std::chrono::time_point<std::chrono::steady_clock> previous, current;
extern bool biased;

extern void doAction(float* data) {}

int main()
{
    while (!chosen) {
        printf("Choose the action, which you would like to do:\n");
        printf("\t [1] : Display data.\n");
        printf("\t [2] : Write data to a file.\n");
        printf("\t [3] : Display data and write them to file.\n");
        r = getchar();
        switch (r) {
        case '1': chosen = true; break;
        case '2': chosen = true; break;
        case '3': chosen = true; break;
        }
    }
    int choice = (int)r - 48;
    if ((choice & 2) == 2) {
        const std::time_t now = std::time(nullptr);
        char mbstr[100];
        std::strftime(mbstr, sizeof(mbstr), "_%Y-%m-%d_%H-%M-%S", std::localtime(&now));
        std::string filename = FILE + std::string(mbstr) + EXTENSION;
        myfile.open(filename, std::ios::out);
    }
    if ((initATINano() == -1))
        return 0;
    StartForceThread();
    if ((initHD() == -1))
        return 0;
    
    if (myfile.is_open()) {
        myfile.close();
        printf("\n\n Data saved to file.\n");
    }
    StopForceThread();
}

/******************************************************************************
 Initializes haptics.
******************************************************************************/
int initHD(void)
{
    HDErrorInfo error;
    HDSchedulerHandle handlePosition;

    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        return -1;
    }

    printf("Found device model: %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));

    /* Schedule the main callback that will render forces to the device. */
    handlePosition = hdScheduleAsynchronous(deviceCallback, 0, HD_MAX_SCHEDULER_PRIORITY);

    while (biased) {}
    //hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();

    /* Check for errors and abort if so. */
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        return -1;
    }

    while (!_getch())
    {
        /* Periodically check if the gravity well callback has exited. */
        if (!hdWaitForCompletion(handlePosition, HD_WAIT_CHECK_STATUS))  // Checks if a callback is still scheduled for execution.
        {
            fprintf(stderr, "Press any key to stop.\n");
            break;
        }
    }

    /* For cleanup, unschedule callback and stop the scheduler. */
    hdStopScheduler();          // Typically call this as a frst step for cleanup and shutdown of devices
    hdUnschedule(handlePosition); // removing the associated callback from the scheduler.
    hdDisableDevice(hHD);       // Disables a device. The handle should not be used afterward

    return 0;
}

/*******************************************************************************
 Servo callback.
 Called every servo loop tick.  Simulates a gravity well, which sucks the device
 towards its center whenever the device is within a certain range.
*******************************************************************************/
HDCallbackCode HDCALLBACK deviceCallback(void* data)
{
    HDErrorInfo error;
    hduVector3Dd position;
    hduVector3Dd force;

    HHD hHD = hdGetCurrentDevice();  // Gets the handle of the current device

    /* Begin haptics frame.  ( In general, all state-related haptics calls
       should be made within a frame. ) */
    hdBeginFrame(hHD);

    /* Get the current position of the device. */
    hdGetDoublev(HD_CURRENT_POSITION, position);

    memset(force, 0, sizeof(hduVector3Dd));

    float FT[MAX_VALUES] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    getData(FT);
    double time_seconds;

    if (first)
    {
        first = false;
        previous = std::chrono::high_resolution_clock::now();
        time_seconds = 0.0f;
    }
    else
    {
        current = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed_time = current - previous;
        time_seconds = elapsed_time.count() / 1000;
    }
    float p[3] = { position[0], position[1], position[2] };
    previous = current;

    int choice = (int)r - 48;
    if ((choice & 1) == 1)
        DisplayData(p, FT, time_seconds);
    if ((choice & 2) == 2)
        WriteData(p, FT, time_seconds);
    

    //std::cout << position[0] << "\t" << position[1] << "\t" << position[2] << std::endl;

    /* End haptics frame. */
    hdEndFrame(hHD);

    /* Check for errors and abort the callback if a scheduler error
       is detected. */
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error,
            "Error getting data from device.\n");

        if (hduIsSchedulerError(&error))
        {
            return HD_CALLBACK_DONE;
        }
    }

    /* Signify that the callback should continue running, i.e. that
       it will be called again the next scheduler tick. */
    return HD_CALLBACK_CONTINUE;
}

void DisplayData(float* position, float* data, double elapsed_time)
{
    printf("\nResult:\n");
    for (int i = 0; i < 3; i++)
        printf("%9.4f", position[i]);
    for (int i = 0; i < MAX_VALUES; i++)
        printf("%9.6f ", data[i]);
    printf("%9.6f ", elapsed_time);
}

void WriteData(float* position, float* data, double elapsed_time)
{
    if (!myfile.is_open())
        return;

    char mbstr[100];
    for (int i = 0; i < 3; i++)
        myfile << position[i] << ',';
    for (int i = 0; i < MAX_VALUES; i++)
        myfile << data[i] << ',';
    myfile << elapsed_time;
    myfile << std::endl;

}