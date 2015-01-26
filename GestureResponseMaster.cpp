#include "Client.h"

#include <GL/glut.h>

#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <math.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <pthread.h>

#define SEND_IP "10.2.255.255"  // broadcast address for IVS network (update if needed)
#define BUFLEN 512
#define PORT 25885

#include "../boost_1_53_0/boost/format.hpp"
#include "../boost_1_53_0/boost/lexical_cast.hpp"

#define NELEMS(x)  (sizeof(x) / sizeof(x[0]))

using namespace std;
using namespace ViconDataStreamSDK::CPP;
using boost::format;

#define output_stream if(!LogFile.empty()) ; else std::cout 

// ***VICON***

// Make a new client
Client MyClient;
std::string HostName = "141.219.28.17:801";
//std::string HostName = "localhost:801";

namespace
{
	std::string Adapt( const bool i_Value )
	{
		return i_Value ? "True" : "False";
	}

	std::string Adapt( const Direction::Enum i_Direction )
	{
		switch( i_Direction )
		{
			case Direction::Forward:
				return "Forward";
			case Direction::Backward:
				return "Backward";
			case Direction::Left:
				return "Left";
			case Direction::Right:
				return "Right";
			case Direction::Up:
				return "Up";
			case Direction::Down:
				return "Down";
			default:
				return "Unknown";
		}
	}

	std::string Adapt( const DeviceType::Enum i_DeviceType )
	{
		switch( i_DeviceType )
		{
			case DeviceType::ForcePlate:
				return "ForcePlate";
			case DeviceType::Unknown:
			default:
				return "Unknown";
		}
	}

	std::string Adapt( const Unit::Enum i_Unit )
	{
		switch( i_Unit )
		{
			case Unit::Meter:
				return "Meter";
			case Unit::Volt:
				return "Volt";
			case Unit::NewtonMeter:
				return "NewtonMeter";
			case Unit::Newton:
				return "Newton";
			case Unit::Kilogram:
				return "Kilogram";
			case Unit::Second:
				return "Second";
			case Unit::Ampere:
				return "Ampere";
			case Unit::Kelvin:
				return "Kelvin";
			case Unit::Mole:
				return "Mole";
			case Unit::Candela:
				return "Candela";
			case Unit::Radian:
				return "Radian";
			case Unit::Steradian:
				return "Steradian";
			case Unit::MeterSquared:
				return "MeterSquared";
			case Unit::MeterCubed:
				return "MeterCubed";
			case Unit::MeterPerSecond:
				return "MeterPerSecond";
			case Unit::MeterPerSecondSquared:
				return "MeterPerSecondSquared";
			case Unit::RadianPerSecond:
				return "RadianPerSecond";
			case Unit::RadianPerSecondSquared:
				return "RadianPerSecondSquared";
			case Unit::Hertz:
				return "Hertz";
			case Unit::Joule:
				return "Joule";
			case Unit::Watt:
				return "Watt";
			case Unit::Pascal:
				return "Pascal";
			case Unit::Lumen:
				return "Lumen";
			case Unit::Lux:
				return "Lux";
			case Unit::Coulomb:
				return "Coulomb";
			case Unit::Ohm:
				return "Ohm";
			case Unit::Farad:
				return "Farad";
			case Unit::Weber:
				return "Weber";
			case Unit::Tesla:
				return "Tesla";
			case Unit::Henry:
				return "Henry";
			case Unit::Siemens:
				return "Siemens";
			case Unit::Becquerel:
				return "Becquerel";
			case Unit::Gray:
				return "Gray";
			case Unit::Sievert:
				return "Sievert";
			case Unit::Katal:
				return "Katal";

			case Unit::Unknown:
			default:
				return "Unknown";
		}
	}
}

void viconExit()
{
    MyClient.DisableSegmentData();
//    MyClient.DisableMarkerData();
//    MyClient.DisableUnlabeledMarkerData();
//    MyClient.DisableDeviceData();

	// TODO: Disconnect seems to cause a hang. -Scott Kuhl
    // Disconnect and dispose
    int t = clock();
    std::cout << " Disconnecting..." << std::endl;
    MyClient.Disconnect();
    int dt = clock() - t;
    double secs = (double) (dt)/(double)CLOCKS_PER_SEC;
    std::cout << " Disconnect time = " << secs << " secs" << std::endl;
}

void viconInit()
{
    // Connect to a server
    std::cout << "Connecting to " << HostName << " ..." << std::flush;
	int attemptConnectCount = 0;
	const int MAX_CONNECT_ATTEMPTS=2;
    while( !MyClient.IsConnected().Connected && attemptConnectCount < MAX_CONNECT_ATTEMPTS)
    {
		attemptConnectCount++;
		bool ok = false;
		ok =( MyClient.Connect( HostName ).Result == Result::Success );
		if(!ok)
			std::cout << "Warning - connect failed..." << std::endl;
		std::cout << ".";
		sleep(1);
    }
	if(attemptConnectCount == MAX_CONNECT_ATTEMPTS)
	{
		printf("Giving up making connection to Vicon system\n");
		return;
	}
    std::cout << std::endl;

    // Enable some different data types
    MyClient.EnableSegmentData();
    //MyClient.EnableMarkerData();
    //MyClient.EnableUnlabeledMarkerData();
    //MyClient.EnableDeviceData();

    std::cout << "Segment Data Enabled: "          << Adapt( MyClient.IsSegmentDataEnabled().Enabled )         << std::endl;
    std::cout << "Marker Data Enabled: "           << Adapt( MyClient.IsMarkerDataEnabled().Enabled )          << std::endl;
    std::cout << "Unlabeled Marker Data Enabled: " << Adapt( MyClient.IsUnlabeledMarkerDataEnabled().Enabled ) << std::endl;
    std::cout << "Device Data Enabled: "           << Adapt( MyClient.IsDeviceDataEnabled().Enabled )          << std::endl;

    // Set the streaming mode
    //MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPull );
    // MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch );
    MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ServerPush );

    // Set the global up axis
    MyClient.SetAxisMapping( Direction::Forward, 
                             Direction::Left, 
                             Direction::Up ); // Z-up
    // MyClient.SetGlobalUpAxis( Direction::Forward, 
    //                           Direction::Up, 
    //                           Direction::Right ); // Y-up

    Output_GetAxisMapping _Output_GetAxisMapping = MyClient.GetAxisMapping();
    std::cout << "Axis Mapping: X-" << Adapt( _Output_GetAxisMapping.XAxis ) 
			  << " Y-" << Adapt( _Output_GetAxisMapping.YAxis ) 
			  << " Z-" << Adapt( _Output_GetAxisMapping.ZAxis ) << std::endl;

    // Discover the version number
    Output_GetVersion _Output_GetVersion = MyClient.GetVersion();
    std::cout << "Version: " << _Output_GetVersion.Major << "." 
			  << _Output_GetVersion.Minor << "." 
			  << _Output_GetVersion.Point << std::endl;

}



// an atexit() callback:
void exitCallback()
{
	viconExit();
	return;
}

/*void keyboard(unsigned char key, int x, int y)
{
	if (key == 27 || key == 'q')  // escape key, exit program
		exit(0);
}*/

// end of Vicon stuff /////////////////////////////////////////////////////////////////////////////


// DGR variables
pthread_t senderThread;
struct sockaddr_in si_other;
int slen, s;
char buf [BUFLEN];
int so_broadcast = 1;

string ipAddress;
int port;

const GLdouble SCREEN_WIDTH = (1920*6)/8.0;  
const GLdouble SCREEN_HEIGHT = (1080.0*4)/8.0;
const float screenAspectRatio = SCREEN_WIDTH/SCREEN_HEIGHT;

typedef struct trackable {
  float x;
  float y;
  float z;
} trackable;

const int dataHertz = 100;
const int numTrackedObjects = 4;

ofstream outputFile;
bool simulation = true;
vector<string> trackNames;

vector<string> objectsToTrack;
string dataToSend;
bool drawingOn = false;
string flagObject;
int switchDrawingCtr = 0;

int totalCtr = 0;
// Artist performance variable **CUSTOMIZABLE**
const int UPDATE_COUNTER = 120;  // A new line will be recorded
                                 //(Vicon Update Rate / UPDATE_COUNTER) * number_of_tracked_objects
                                 // times/second.
                                 // So, assuming a Vicon update rate of about 100 Hz and if
                                 // UPDATE_COUNTER is set to 40 and 2 objects are being tracked, a new line is recorded
                                 // ~5 times per second.
                                 // The higher this value, the nicer the lines will look, but the more quickly
                                 // the buffer will fill up. Tweak this value to find a
                                 // good balance.

void error(const char *msg) {
  perror(msg);
  exit(1);
}

void closeProgram() {
  exit(0);
}

void display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glutSwapBuffers();
}

bool recording = false;

void gtfo() {
  if (!simulation) outputFile.close();
  exit(0);
}

void keyboard(unsigned char key, int x, int y) {
  if (key == 'x') gtfo();
  else if (key == ' ') recording = !recording;
}

int gargc;
char** gargv;

void sender() {
  if (simulation) { // read from data dump
    string line;
    ifstream inputFile(gargv[1]);
    if (inputFile.is_open()) {
      while (inputFile.good()) {
        getline(inputFile, line);
        sprintf(buf, "%s", line.c_str());
        if (sendto(s, buf, BUFLEN, 0, (struct sockaddr*)&si_other,
          slen) == -1) error("ERROR sendto()");
        usleep(1000);
      }
    } else {
      printf("Unable to open file\n");
      exit(1);
    }
    inputFile.close();
  } else { // live tracking w/ Vicon
    outputFile.open(gargv[4]);
    flagObject = gargv[5];
    for (int i = 6; i < gargc; i++) objectsToTrack.push_back(string(gargv[i]));
    //vector<format> formatters;
    //for (int i = 0; i < objectsToTrack.size(); i++) formatters.push_back(format("%1%~%2%~%3%~%4%"));
    while (true) {
      totalCtr++;
      if (MyClient.GetFrame().Result != Result::Success )
        printf("WARNING: Inside display() and there is no data from Vicon...\n");
      if (switchDrawingCtr > 0) switchDrawingCtr--;
      Output_GetSegmentGlobalTranslation flagTranslate = MyClient.GetSegmentGlobalTranslation(flagObject, flagObject);
      if (flagTranslate.Translation[2] > 2.0 && switchDrawingCtr <= 0) {
        switchDrawingCtr = 360;
        drawingOn = !drawingOn;
      }
      if (drawingOn && totalCtr % UPDATE_COUNTER == 0) {
        for (int i = 0; i < objectsToTrack.size(); i++) {
          dataToSend.clear();
          Output_GetSegmentGlobalTranslation globalTranslate = MyClient.GetSegmentGlobalTranslation(objectsToTrack[i], objectsToTrack[i]);
          Output_GetSegmentGlobalRotationEulerXYZ globalRotation = MyClient.GetSegmentGlobalRotationEulerXYZ(objectsToTrack[i], objectsToTrack[i]);
          dataToSend = objectsToTrack[i];
          dataToSend.append("~");
          dataToSend.append(boost::lexical_cast<string>((float)globalTranslate.Translation[0] / 1000.0f));
          dataToSend.append("~");
          dataToSend.append(boost::lexical_cast<string>((float)globalTranslate.Translation[1] / 1000.0f));
          dataToSend.append("~");
          dataToSend.append(boost::lexical_cast<string>((float)globalTranslate.Translation[2] / 1000.0f));
//          formatters[i] % objectsToTrack[i];
//          formatters[i] % (globalTranslate.Translation[0] / 1000);
//          formatters[i] % (globalTranslate.Translation[1] / 1000);
//          formatters[i] % (globalTranslate.Translation[2] / 1000);
//          dataToSend.append(formatters[i].str());
          outputFile << dataToSend << "\n";
          dataToSend.append("\n");
          if (sendto(s, dataToSend.c_str(), dataToSend.length(), 0, (struct sockaddr*)&si_other, slen) == -1) {
            perror ("ERROR sendto()");
          }
//printf("I sent %s\n", dataToSend.c_str());
        } // end for loop thru objectsToTrack
      } // end ifDrawingOn
    }
  } // end live tracking w/ Vicon
}

int main(int argc, char** argv) {
  if (argc < 2) {
    printf("USAGE:\n");
    printf("Playback mode:    GestureResponseMaster input_filename ip_address port\n");
    printf("Live tracking:    GestureResponseMaster FALSE ip_address port output_filename flag_object objects_to_track\n");
    return 1;
  }

  gargc = argc;
  gargv = argv;

  ipAddress = string(argv[2]);
  port = atoi(argv[3]);

  // socket setup
  slen=sizeof(si_other);
  if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) error("ERROR socket - audio");
  setsockopt(s, SOL_SOCKET, SO_BROADCAST, &so_broadcast, sizeof(so_broadcast));
  memset((char *) &si_other, 0, sizeof(si_other));
  si_other.sin_family = AF_INET;
  si_other.sin_port = htons(port);
  if (inet_aton(ipAddress.c_str(), &si_other.sin_addr) == 0) {
    fprintf(stderr, "inet_aton() failed\n");
    exit(1);
  }

  atexit(exitCallback);
  viconInit(); // Vicon initialization

  // OGL stuff
  /*glutInit(&argc, argv);
  glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);
  glutInitWindowSize(200, 200);
  glutInitWindowPosition(0, 0);
  glutCreateWindow(argv[0]);
  glutKeyboardFunc(keyboard);
  glClearColor(1.0,1.0,1.0, 1);
  glutDisplayFunc(display); */
  // end OGL stuff

  if (strcmp(argv[1], "FALSE") == 0) simulation = false;
  else simulation = true;

  /*if (pthread_create(&senderThread, NULL, sender, NULL) != 0) {
      perror("Can't start thread, terminating\n");
      return 1;
  } */

  //glutMainLoop();

  if (simulation) { // read from data dump
    string line;
    ifstream inputFile(gargv[1]);
    if (inputFile.is_open()) {
      while (inputFile.good()) {
        getline(inputFile, line);
        sprintf(buf, "%s", line.c_str());
        if (sendto(s, buf, BUFLEN, 0, (struct sockaddr*)&si_other,
          slen) == -1) error("ERROR sendto()");
        usleep(1000);
      }
    } else {
      printf("Unable to open file\n");
      exit(1);
    }
    inputFile.close();
  } else { // live tracking w/ Vicon
    outputFile.open(gargv[4]);
    flagObject = gargv[5];
    for (int i = 6; i < gargc; i++) objectsToTrack.push_back(string(gargv[i]));
    //vector<format> formatters;
    //for (int i = 0; i < objectsToTrack.size(); i++) formatters.push_back(format("%1%~%2%~%3%~%4%"));
    while (true) {
      if (MyClient.GetFrame().Result != Result::Success )
        printf("WARNING: Inside display() and there is no data from Vicon...\n");
      if (switchDrawingCtr > 0) switchDrawingCtr--;
      Output_GetSegmentGlobalTranslation flagTranslate = MyClient.GetSegmentGlobalTranslation(flagObject, flagObject);
      if (flagTranslate.Translation[2] > 2000.0 && switchDrawingCtr <= 0) {
        switchDrawingCtr = 360;
        drawingOn = !drawingOn;
        if (drawingOn) printf("Drawing has switched from OFF to ON\n");
        else printf("Drawing has switched from ON to OFF\n");
      }
      if (drawingOn) {
        for (int i = 0; i < objectsToTrack.size(); i++) {
          dataToSend.clear();
          Output_GetSegmentGlobalTranslation globalTranslate = MyClient.GetSegmentGlobalTranslation(objectsToTrack[i], objectsToTrack[i]);
          Output_GetSegmentGlobalRotationEulerXYZ globalRotation = MyClient.GetSegmentGlobalRotationEulerXYZ(objectsToTrack[i], objectsToTrack[i]);
          dataToSend = objectsToTrack[i];
          dataToSend.append("~");
          dataToSend.append(boost::lexical_cast<string>((float)globalTranslate.Translation[0] / -1000.0f));
          dataToSend.append("~");
          dataToSend.append(boost::lexical_cast<string>((float)globalTranslate.Translation[1] / 1000.0f * 1.5f));
          dataToSend.append("~");
          dataToSend.append(boost::lexical_cast<string>((float)globalTranslate.Translation[2] / 1000.0f * 3.5f - 2.0f));
//          formatters[i] % objectsToTrack[i];
//          formatters[i] % (globalTranslate.Translation[0] / 1000);
//          formatters[i] % (globalTranslate.Translation[1] / 1000);
//          formatters[i] % (globalTranslate.Translation[2] / 1000);
//          dataToSend.append(formatters[i].str());
          outputFile << dataToSend << "\n";
          dataToSend.append("\n");
          if (sendto(s, dataToSend.c_str(), dataToSend.length(), 0, (struct sockaddr*)&si_other, slen) == -1) {
            perror ("ERROR sendto()");
          }
//printf("I sent %s\n", dataToSend.c_str());
        } // end for loop thru objectsToTrack
      } else { // end ifDrawingOn
        dataToSend = "DUMMYDATA\n";
        if (sendto(s, dataToSend.c_str(), dataToSend.length(), 0, (struct sockaddr*)&si_other, slen) == -1) {
          perror ("ERROR sendto()");
        }
        usleep(10000);
      } // end else of ifDrawingOn
    }
  } // end live tracking w/ Vicon

  return 0;
}
