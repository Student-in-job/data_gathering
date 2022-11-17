#include <string.h>
#include <stdlib.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <conio.h>
#include <fstream>
#include <iostream>
#include <chrono>
#include <time.h>

#define FILE                  "./data"
#define EXTENSION             ".csv"
#define TARGET_SERVOLOOP_RATE 1000
#define OFFSET_POSITION       1
#define OFFSET                -47

bool            chosen;
char            r = '1';
std::ofstream   myfile;
HDErrorInfo     error;
HHD             hHD;

// ------------- Forward declarations -----------------
int initHD(void);
int calibrateHD(void);
HDCallbackCode HDCALLBACK deviceCallback(void* data);
void DisplayData(float* position, float* data);
void WriteData(float* position, float* data);

int main()
{
    const std::time_t now = std::time(nullptr);
    char mbstr[100];
    std::strftime(mbstr, sizeof(mbstr), "_%Y-%m-%d_%H-%M-%S", std::localtime(&now));
    std::string filename = FILE + std::string(mbstr) + EXTENSION;
    myfile.open(filename, std::ios::out);

    if ((initHD() == -1))
        return 0;

    if (myfile.is_open()) {
        myfile.close();
        printf("\n\n Data saved to file.\n");
    }
}

int calibrateHD(void)
{
    int supportedCalibrationStyles;
    int calibrationStyle;

    printf("Calibration\n");
    printf("Found %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));
    /* Choose a calibration style.  Some devices may support multiple types of
       calibration.  In that case, prefer auto calibration over inkwell
       calibration, and prefer inkwell calibration over reset encoders. */
    hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
    if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET)
    {
        calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL)
    {
        calibrationStyle = HD_CALIBRATION_INKWELL;
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_AUTO)
    {
        calibrationStyle = HD_CALIBRATION_AUTO;
    }

    if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET)
    {
        printf("Please prepare for manual calibration by\n");
        printf("placing the device at its reset position.\n\n");
        printf("Press any key to continue...\n");

        while (!_getch()) {}

        hdUpdateCalibration(calibrationStyle);
        if (hdCheckCalibration() == HD_CALIBRATION_OK)
        {
            printf("Calibration complete.\n\n");
        }
        if (HD_DEVICE_ERROR(error = hdGetError()))
        {
            hduPrintError(stderr, &error, "Reset encoders reset failed.");
            return -1;
        }
    }
}


/******************************************************************************
 Initializes haptics.
******************************************************************************/
int initHD(void)
{
    hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        return -1;
    }

    printf("Found device model: %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));

    if (calibrateHD() == -1)
    {
        fprintf(stderr, "\nFailed to calibrate the device.\n");
        return -1;
    }
    bool paused = true;
    char stopchar = 't';
    fprintf(stderr, "\nTo start data gathering press any key.\n");
    while (paused)
    {
        stopchar = getchar();
        if (stopchar != 't')
            paused = false;
    }

    /* Schedule the main callback that will render forces to the device. */
    HDSchedulerHandle hServoCallback = hdScheduleAsynchronous(deviceCallback, 0, HD_MAX_SCHEDULER_PRIORITY);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        std::cerr << error << std::endl;
        std::cerr << "\nFailed to schedule servoloop callback\n" << std::endl;
        hdDisableDevice(hHD);
        return -1;
    }

    hdSetSchedulerRate(TARGET_SERVOLOOP_RATE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        std::cerr << error << std::endl;
        std::cerr << "\nFailed to set servoloop rate\n" << std::endl;
        hdDisableDevice(hHD);
        return -1;
    }

    hdDisable(HD_FORCE_OUTPUT);

    //hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();

    /* Check for errors and abort if so. */
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "\nFailed to start scheduler\n");
        fprintf(stderr, "\nPress any key to quit.\n");
        return -1;
    }

    while (!_getch())
    {
        /* Periodically check if the gravity well callback has exited. */
        if (!hdWaitForCompletion(hServoCallback, HD_WAIT_CHECK_STATUS))  // Checks if a callback is still scheduled for execution.
        {
            fprintf(stderr, "Press any key to stop.\n");
            break;
        }
    }

    /* For cleanup, unschedule callback and stop the scheduler. */
    hdStopScheduler();          // Typically call this as a frst step for cleanup and shutdown of devices
    hdUnschedule(hServoCallback); // removing the associated callback from the scheduler.
    hdDisableDevice(hHD);       // Disables a device. The handle should not be used afterward

    return 0;
}

/*******************************************************************************
 Servo callback.
*******************************************************************************/
HDCallbackCode HDCALLBACK deviceCallback(void* data)
{
    hduMatrix previous;
    double transformMatrix[16];
    HDErrorInfo error;
    hduVector3Dd position;
    hduVector3Dd force;

    HHD hHD = hdGetCurrentDevice();  // Gets the handle of the current device

    /* Begin haptics frame.  ( In general, all state-related haptics calls
       should be made within a frame. ) */
    hdBeginFrame(hHD);

    /* Get the current position of the device. */
    hdGetDoublev(HD_CURRENT_POSITION, position);
    hdGetDoublev(HD_CURRENT_TRANSFORM, transformMatrix);

    float p[3] = { position[0], position[1], position[2] };
    previous = hduMatrix(transformMatrix);
    float tm[16] = { 0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0 };
    for (int i = 0; i < 12; i++)
        tm[i] = transformMatrix[i];

    DisplayData(p, tm);
    WriteData(p, tm);

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

void DisplayData(float* position, float* data)
{
    printf("\nResult:\n");
    for (int i = 0; i < 3; i++)
        printf("%9.4f", position[i]);
    for (int i = 0; i < 16; i++)
        printf("%9.6f ", data[i]);
}

void WriteData(float* position, float* data)
{
    if (!myfile.is_open())
        return;

    for (int i = 0; i < 3; i++)
        myfile << position[i] << ',';
    myfile << std::endl;
    myfile << std::endl;
    for (int i = 0; i < 16; i++)
    {
        if (i % 4 == 0)
            myfile << std::endl;
        myfile << data[i] << ',';
    };
    myfile << std::endl;
    myfile << std::endl;
    myfile << std::endl;
}