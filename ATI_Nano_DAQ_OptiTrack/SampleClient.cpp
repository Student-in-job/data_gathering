/*

SampleClient.cpp

This program connects to a NatNet server, receives a data stream, and writes that data stream
to an cvs file.  The purpose is to illustrate using the NatNetClient class.

Usage [optional]:

	SampleClient [ServerIP] [LocalIP] [OutputFilename]

	[ServerIP]			IP address of the server (e.g. 192.168.0.107) ( defaults to local machine)
	[OutputFilename]	Name of points file (pts) to write out.  defaults to Client-output.pts

*/

#include "SampleClient.h"
#include "DAQ_Force.h"

#include <iostream>
#define SERVER_IP_ADDRESS "192.168.0.63"

// Extern declaration from DAQ_Force.h
extern bool started;
extern bool biased;
extern void doAction(float* data);

static const ConnectionType kDefaultConnectionType = ConnectionType_Multicast;

NatNetClient* g_pClient = NULL;
std::ofstream   g_outputFile;
sNatNetClientConnectParams g_connectParams;
int g_analogSamplesPerMocapFrame = 0;
sServerDescription g_serverDescription;
std::string fName = "output.csv";
bool chosen = false;
char r = '1';
float forceData[MAX_VALUES] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
bool first = true;
std::chrono::time_point<std::chrono::steady_clock> previous, current;

int main( int argc, char* argv[] )
{
    while (!chosen) {
        std::cout << "Choose the action, which you would like to do:" << std::endl;
        std::cout << "\t [1] : Display data."<< std::endl;
        std::cout << "\t [2] : Write data to a file." << std::endl;
        std::cout << "\t [3] : Display data and write them to file." << std::endl;
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
        std::string filename = fName + std::string(mbstr) + ".csv";

        // Create data file for writing received stream into
        g_outputFile.open(filename, std::ios::out);
    }
    if ((initATINano() == -1))
        return 0;

    StartForceThread();
    while (biased) {}

    // print version info
    unsigned char ver[4];
    NatNet_GetVersion( ver );
    printf( "NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3] );

    g_connectParams.connectionType = ConnectionType_Multicast;
    g_connectParams.serverAddress = SERVER_IP_ADDRESS;
    g_connectParams.localAddress = SERVER_IP_ADDRESS;
    g_connectParams.serverCommandPort = NATNET_DEFAULT_PORT_COMMAND;
    g_connectParams.serverDataPort = NATNET_DEFAULT_PORT_DATA;
    g_connectParams.multicastAddress = NATNET_DEFAULT_MULTICAST_ADDRESS;

    // Install logging callback
    NatNet_SetLogCallback( MessageHandler );

    // create NatNet client
    g_pClient = new NatNetClient();

    std::cout << "Connectiong to a server:" << SERVER_IP_ADDRESS << std::endl;
    int iResult;

    // Connect to Motive
    iResult = ConnectClient();
    if (iResult != ErrorCode_OK)
    {
        std::cout << "Error initializing client. See log for details. Exiting." << std::endl;
        return 1;
    }
    else
    {
        std::cout << "Client initialized and ready" << std::endl;
    }

    // Send/receive test request
    void* response;
    int nBytes;
    std::cout << "[SampleClient] Sending Test Request" << std::endl;
	iResult = g_pClient->SendMessageAndWait("TestRequest", &response, &nBytes);
	if (iResult == ErrorCode_OK)
	{
        std::cout << "[SampleClient] Received: " << (char*)response << std::endl;
	}

	// Retrieve Data Descriptions from Motive
    std::cout << "[SampleClient] Requesting Data Descriptions..." << std::endl;
	sDataDescriptions* pDataDefs = NULL;
	iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
    if (iResult != ErrorCode_OK || pDataDefs == NULL)
	{
		std::cout<< "[SampleClient] Unable to retrieve Data Descriptions."<< std::endl;
	}

    // set the frame callback handler
    g_pClient->SetFrameReceivedCallback(DataHandler, g_pClient);	// this function will receive data from the server

	// Ready to receive marker stream!
    std::cout << "Client is connected to server and listening for data..." << std::endl;
    std::cout << "Press Q to quit" << std::endl;
	bool bExit = false;
	while ( const int c = getch() )
	{
		switch(c)
		{
			case 'q':
				bExit = true;		
				break;	
			default:
				break;
		}
		if(bExit)
			break;
	}

	// Done - clean up.
	if (g_pClient)
	{
		g_pClient->Disconnect();
		delete g_pClient;
		g_pClient = NULL;
	}

    StopForceThread();
    if (g_outputFile.is_open())
    {
        g_outputFile.close();
    }

    return ErrorCode_OK;
}

// Establish a NatNet Client connection
int ConnectClient()
{
    // Release previous server
    g_pClient->Disconnect();

    // Init Client and connect to NatNet server
    int retCode = g_pClient->Connect( g_connectParams );
    if (retCode != ErrorCode_OK)
    {
        std::cout << "Unable to connect to server.  Error code: " << retCode << ". Exiting." << std::endl;
        return ErrorCode_Internal;
    }
    else
    {
        // connection succeeded

        void* pResult;
        int nBytes = 0;
        ErrorCode ret = ErrorCode_OK;

        // get mocap frame rate
        ret = g_pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            float fRate = *((float*)pResult);
            std::cout << "Mocap Framerate : " << fRate << std::endl;
        }
        else
            std::cout << "Error getting frame rate." << std::endl;

        // get # of analog samples per mocap frame of data
        ret = g_pClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            g_analogSamplesPerMocapFrame = *((int*)pResult);
            std::cout << "Analog Samples Per Mocap Frame : " << g_analogSamplesPerMocapFrame << std::endl;
        }
        else
            std::cout << "Error getting Analog frame rate." << std::endl;
    }

    return ErrorCode_OK;
}

// DataHandler receives data from the server
// This function is called by NatNet when a frame of mocap data is available
void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData)
{
    NatNetClient* pClient = (NatNetClient*) pUserData;
    float position[3] = { data->RigidBodies[0].x, data->RigidBodies[0].y, data->RigidBodies[0].z };
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
    previous = current;

    int choice = (int)r - 48;
    if ((choice & 1) == 1)
        DisplayData(position, forceData, time_seconds);
    if ((choice & 2) == 2)
        WriteData(position, forceData, time_seconds);
}


// MessageHandler receives NatNet error/debug messages
void NATNET_CALLCONV MessageHandler( Verbosity msgType, const char* msg )
{
    // Optional: Filter out debug messages
    if ( msgType < Verbosity_Info )
    {
        return;
    }

    printf( "\n[NatNetLib]" );

    switch ( msgType )
    {
        case Verbosity_Debug:
            printf( " [DEBUG]" );
            break;
        case Verbosity_Info:
            printf( "  [INFO]" );
            break;
        case Verbosity_Warning:
            printf( "  [WARN]" );
            break;
        case Verbosity_Error:
            printf( " [ERROR]" );
            break;
        default:
            printf( " [?????]" );
            break;
    }

    printf( ": %s\n", msg );
}

///TODO: Making some data reading 6 values - force and torque
void doAction(float* data){
    for (int i = 0; i < MAX_VALUES; i++)
        forceData[i] = data[i];
}

void DisplayData(float* position, float* data, double elapsed_time)
{
    std::cout << std::endl;
    std::cout<<"Result:"<<std::endl;
    for (int i = 0; i < 3; i++)
        std::cout << position[i] << '\t';
    for (int i = 0; i < MAX_VALUES; i++)
        std::cout << data[i] << '\t';
    std::cout << elapsed_time;
}

void WriteData(float* position, float* data, double elapsed_time)
{
    if (!g_outputFile.is_open())
        return;

    for (int i = 0; i < 3; i++)
        g_outputFile << position[i] << ',';
    for (int i = 0; i < MAX_VALUES; i++)
        g_outputFile << data[i] << ',';
    g_outputFile << elapsed_time;
    g_outputFile << std::endl;
}

#ifndef _WIN32
char getch()
{
    char buf = 0;
    termios old = { 0 };

    fflush( stdout );

    if ( tcgetattr( 0, &old ) < 0 )
        perror( "tcsetattr()" );

    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;

    if ( tcsetattr( 0, TCSANOW, &old ) < 0 )
        perror( "tcsetattr ICANON" );

    if ( read( 0, &buf, 1 ) < 0 )
        perror( "read()" );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;

    if ( tcsetattr( 0, TCSADRAIN, &old ) < 0 )
        perror( "tcsetattr ~ICANON" );

    //printf( "%c\n", buf );

    return buf;
}
#endif
