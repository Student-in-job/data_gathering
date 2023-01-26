// ATI_Nano_DAQ_OpenHaptics.cpp : This file contains the 'main' function. Program execution begins and ends there.
#include "ATI_Nano_DAQ_OpenHaptics.h"
#include "DAQ_Force.h"

#define FILE                  "./data"
#define EXTENSION             ".csv"
#define TARGET_SERVOLOOP_RATE 1000
#define OFFSET_POSITION       1
#define OFFSET                -47
#define GRAVITY_ACCELERATION  9.8067
#define MASS                  0.026
#define GM_POSITION           2

bool            chosen;
char            r = '1';
std::ofstream   myfile;
bool            first = true;
HDErrorInfo     error;
HHD             hHD; 
std::chrono::time_point<std::chrono::steady_clock> previous, current;
std::vector<std::array<float, 10>> dataPoints;

extern bool started;

extern void doAction(float* data) {}

int main()
{
    while (!chosen) {
        Info("Choose the action, which you would like to do:\n");
        Info("\t [1] : Display data.\n");
        Info("\t [2] : Write data to a file.\n");
        Info("\t [3] : Display data and write them to file.\n");
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
    
    if ((choice & 2) == 2)
        FlushData();

    if (myfile.is_open()) {
        myfile.close();
        Info("\n\n Data saved to file.\n");
    }
    StopForceThread();
}

int calibrateHD(void)
{
    int supportedCalibrationStyles;
    int calibrationStyle;

    Info("\n \t ---------- Calibration ----------\n");
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
        Info("Please prepare for manual calibration by\n");
        Info("placing the device at its reset position.\n\n");
        Info("Press 't' key and Enter to continue...\n");

        while (Pause('t')) {}

        hdUpdateCalibration(calibrationStyle);
        if (hdCheckCalibration() == HD_CALIBRATION_OK)
        {
            Info("\n\t ---------- Calibration complete. ----------\n\n");
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

    Info("\nTo start data gathering press 't' key and Enter to continue....\n");
    while (Pause('t')) {}
    
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

    std::array<float, 10> point = { p[0], p[1], p[2], FT[0], FT[1], FT[2], FT[3], FT[4], FT[5], (float)time_seconds };
    dataPoints.push_back(point);

    int choice = (int)r - 48;
    if ((choice & 1) == 1)
        DisplayData(p, FT, time_seconds);
    
    /* End haptics frame. */
    hdEndFrame(hHD);

    /* Check for errors and abort the callback if a scheduler error
       is detected. */
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Error getting data from device.\n");

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
    std::cout << "Result:" << std::endl;
    for (int i = 0; i < 3; i++)
        if (i == OFFSET_POSITION)
            std::cout << position[i] - OFFSET << " ";
        else
            std::cout << position[i] << " ";
    for (int i = 0; i < MAX_VALUES; i++)
        std::cout << data[i] << " ";
    std::cout << elapsed_time << std::endl;
}

void FlushData()
{
    if (!myfile.is_open())
        return;

    std::array<float, 10> point;
    for (int i = 0; i < dataPoints.size(); i++)
    {
        point = dataPoints[i];
        for (int j = 0; j < 9; j++)
            if (j == OFFSET_POSITION)
                myfile << point[j] - OFFSET << ',';
            else
                myfile << point[j] << ',';
        myfile << point[9] << std::endl;
    }
}

bool Pause(char original)
{
    char stopchar = getchar();
    if (stopchar == original)
        return false;
    else
        return true;
}

// Redefine method to show message on different output
void Info(std::string message)
{
    std::cout << message << std::endl;
}
// Redefine method to show error message
void Error(std::string message)
{
    std::cout << message << std::endl;
}