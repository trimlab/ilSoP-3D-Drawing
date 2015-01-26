// Artistic Rendering using Vicon
// Author: James Walker jwwalker a+ mtu d0+ edu

#include <GL/glut.h>

#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include "../boost_1_53_0/boost/lexical_cast.hpp"

#define BUFLEN 512
#define NPACK 10
#define PORT 25884

using namespace std;

const GLdouble SCREEN_WIDTH = (1920*3);  
const GLdouble SCREEN_HEIGHT = 1080;
const float screenAspectRatio = SCREEN_WIDTH/SCREEN_HEIGHT;

//ofstream outputFile;
bool simulation;

typedef struct trackable {
  float x;
  float y;
  float z;
} trackable;

typedef struct myline {
  float x1, x2;
  float y1, y2;
  float z1, z2;
  float r, g, b;
} myline;

int bufferHead = -1;
const int bufferSeconds = 5;
const int dataHertz = 100;
//const int bufferSize = bufferSeconds * dataHertz;
const int bufferSize = 20;
int numTrackedObjects;

const bool SIMULATION = true;
vector<string> trackNames;
map<string, vector<trackable> > trackHistory; 
vector<float> averageDistances;

map<string, vector<trackable> > afterImages;
//vector<particle> particles; -- disabled; I think these would just get in the way for drawing purposes.

pthread_t simulatorThread;

// Artist performance variables **CUSTOMIZABLE**
const bool LIMIT_BUFFER = true;  // Do we limit the size of our lines buffer?
                                 // If yes: Avoid the problem of the computer running out of memory,
                                 //         but once the limit is reached, old lines will disappear as
                                 //         new ones are drawn.
                                 // If no: The number of lines that can be drawn is "theoretically" unlimited,
                                 //        but of course once the available memory fills up, the program will
                                 //        crash. For this reason, it is recommended to leave LIMIT_BUFFER at "true".

const int ART_BUFFER_SIZE = 200000; // How many lines can be held in memory at one time.
                                 // Make the number too small, and old lines will start to disappear quickly.
                                 // Make the number too big, and the system's performance will degrade.
                                 // Tweak this value to try to achieve an effective balance.
                                 // Here is the equation for how quickly lines will start to disappear based on
                                 // this value:
                                 //
                                 // X = ART_BUFFER_SIZE / (Vicon update rate / (UPDATE_COUNTER/2) * <number of tracked objects>)
                                 //
                                 // where X = number of seconds before the buffer fills up.
                                 // If ART_BUFFER_SIZE = 200,000; Vicon update = 100Hz; UPDATE_COUNTER = 40;
                                 // and you are tracking 4 objects, then it will be 10,000 seconds, or just short of 167
                                 // minutes, before the buffer fills.

const double COLOR_CHANGE = 0.0003f; // The program is configured so that the color of drawn lines changes over time.
                                 // This number controls how quickly the line color changes. The rate of change is
                                 // expressed by the following equation:
                                 //
                                 // X = 1 / COLOR_CHANGE / 60
                                 //
                                 // where X is the number of seconds until the color has completely changed, and the
                                 // pattern repeats. If COLOR_CHANGE = .0003, the color goes through one complete
                                 // cycle about once per minute.

const float LINE_THICKNESS = 5.0f; // Line thickness varies on user's distance from the screen, but this variable
                                   // controls the "base" thickness of the lines. Higher value = fatter lines.
                                   // Adjust to suit your aesthetic taste.

// Artist performance variables **NOT CUSTOMIZABLE--DON'T ALTER THESE**
map<string, vector<myline> > lines;
int lineBufferHead = -1;
map<string, myline> currentLine;
bool drawingOn = true;
double lineRed = 1.0;
double lineGreen = 0.5;
double lineBlue = 0.0;
double lineRedDir = -1.0;
double lineGreenDir = 1.0;
double lineBlueDir = 1.0;

// DGR vars
int s, milliseconds;
struct timespec req;
pthread_t receiverThread;
struct sockaddr_in si_me, si_other;
int slen;

double ortho_left;
double ortho_right;
double ortho_bottom;
double ortho_top;

bool receivedPacket = false;
int framesPassed = 0;

vector<string> &split(const string &s, char delim, vector<string> &elems) {
  stringstream ss(s);
  string item;
  while (getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}

vector<string> split(const string &s, char delim) {
  vector<string> elems;
  return split(s, delim, elems);
}

void error(const char *msg) {
  perror(msg);
  exit(1);
}

void closeProgram() {
  close(s);
  exit(0);
}

float absFloat(float f) {
  if (f >= 0) return f;
  else return -f;
}

float computeAverage(vector<float> values) {
  float avg = 0.0f;
  for (int i = 0; i < values.size(); i++) {
    avg += values[i];
  }
  avg /= values.size();
  return avg;
}

float compute3dDistance(trackable t1, trackable t2) {
  float xd = t1.x - t2.x;
  float yd = t1.y - t2.y;
  float zd = t1.z - t2.z;
  return sqrt(xd*xd + yd*yd + zd*zd);
}

float computeAverageDistance(vector<trackable> t) {
  vector<float> distances;
  for (int i = 0; i < t.size() - 1; i++) {
    for (int j = i + 1; j < t.size(); j++) {
      distances.push_back(compute3dDistance(t[i], t[j]));
    }
  }
  return computeAverage(distances);
}

const int numAfterImages = 24;

void addAfterImage(string key, trackable addMe) {
  if (afterImages[key].size() < numAfterImages) {
    afterImages[key].push_back(addMe);
  } else {
    afterImages[key].erase(afterImages[key].end()-1);
    afterImages[key].insert(afterImages[key].begin(), addMe);
  } /* */
}

trackable calculateVelocity(string key) {
  trackable retData;
  retData.x = 0;
  retData.y = 0;
  retData.z = 0;
  float xVel = 0;
  float yVel = 0;
  float zVel = 0;
  if (bufferHead >= 6) {
    for (int t = bufferHead; t > bufferHead - 5; t--) {
      xVel += trackHistory[key][t].x - trackHistory[key][t-1].x;
      yVel += trackHistory[key][t].y - trackHistory[key][t-1].y;
      zVel += trackHistory[key][t].z - trackHistory[key][t-1].z;
    }
    retData.x = xVel / 5.0f;
    retData.y = yVel / 5.0f;
    retData.z = zVel / 5.0f;
  }
  return retData;
}

float calculateAverageVelocity() {
  trackable retData;
  retData.x = 0.0f;
  retData.y = 0.0f;
  retData.z = 0.0f;
  for (int i = 0; i < trackNames.size(); i++) {
    trackable runningAvg = calculateVelocity(trackNames[i]);
    retData.x += absFloat(runningAvg.x);
    retData.y += absFloat(runningAvg.y);
    retData.z += absFloat(runningAvg.z);
  }
  retData.x /= (float)numTrackedObjects;
  retData.y /= (float)numTrackedObjects;
  retData.z /= (float)numTrackedObjects;
  return ((retData.x + retData.y + retData.z) / 3.0f);
}

int getTmpBufferHead(string effName) {
  int tmpBufferHead = bufferHead - 1;
  if (tmpBufferHead < 0) {
    if (trackHistory[effName].size() == bufferSize) tmpBufferHead = bufferSize - 1;
    else tmpBufferHead = 0;
  }
  return tmpBufferHead;
}

trackable getColors(string effName) {
  trackable color;
  color.x = color.y = color.z = 1.0f;
  int tmpBufferHead = getTmpBufferHead(effName);
  if (averageDistances.size() > 0) {
    color.x = averageDistances[tmpBufferHead] / 2.0f;
    color.z = 1.0f - averageDistances[tmpBufferHead] / 2.0f;
    if (color.x > color.z) color.y = color.x - color.z;
    else color.y = color.z - color.x;
    if (color.x > 1.0f) color.x = 1.0f;
    if (color.z < 0.0f) color.z = 0.0f;
    if (color.y > 1.0f) color.y = 1.0f;
  }
  return color;
}

float cubeRotationA = 0.0f;
float cubeRotationB = 0.0f;
float cubeRotationC = 0.0f;
float cubeColorR = 0.3f;
float cubeColorG = 0.6f;
float cubeColorB = 0.9f;

int executionCtr = 0;
int totalCtr = 0;

void averageDistanceHelper() {
            executionCtr++;
            if (trackNames.size() == numTrackedObjects) {
              vector<trackable> points;
              for (int i = 0; i < trackNames.size(); i++) {
                points.push_back(trackHistory[trackNames[i]][bufferHead]);
              }
              if (averageDistances.size() < bufferSize) {
                averageDistances.push_back(computeAverageDistance(points));
              } else {
                averageDistances[bufferHead] = computeAverageDistance(points);
              }
            }
            bufferHead++;
}

void receiver() {
  char buf[BUFLEN];
  vector<string> splitLine;
  while (true) {
    if (recvfrom(s, buf, BUFLEN, 0, (struct sockaddr*)&si_other,
      &slen) == -1) error("ERROR recvfrom()");
    receivedPacket = true;
    framesPassed = 0;
    string itrmdt(buf);
    splitLine = split(itrmdt, '~');
    if (splitLine.size() == 4) {  // valid input line
      trackable newTrackData;
      newTrackData.x = atof(splitLine[1].c_str());
      newTrackData.y = atof(splitLine[2].c_str());
      newTrackData.z = atof(splitLine[3].c_str());
      if (trackHistory.count(splitLine[0]) == 0) {
        trackNames.push_back(splitLine[0]);
        myline newcline;
        newcline.x1 = newcline.x2 = newTrackData.x;
        newcline.y1 = newcline.y2 = newTrackData.y;
        newcline.z1 = newcline.z2 = newTrackData.z;
        currentLine[splitLine[0]] = newcline;
      }
      if (bufferHead >= bufferSize) bufferHead = 0;
      if (trackHistory[splitLine[0]].size() < bufferSize) {
        trackHistory[splitLine[0]].push_back(newTrackData);
      } else {
        trackHistory[splitLine[0]][bufferHead] = newTrackData;
      }
      if (executionCtr % 3 == 0) addAfterImage(splitLine[0], newTrackData);
      

      // add particles
      /*if (executionCtr % 50 == 0) {
        trackable velocityData = calculateVelocity(splitLine[0]);
        trackable color = getColors(splitLine[0]);
        if (velocityData.x != 0 && velocityData.y != 0 && velocityData.z != 0) {
          particle newParticle;
          newParticle.x = newTrackData.x;
          newParticle.y = newTrackData.y;
          newParticle.z = newTrackData.z;
          newParticle.x_vel = velocityData.x;
          newParticle.y_vel = velocityData.y;
          newParticle.z_vel = velocityData.z;
          newParticle.colorR = color.x;
          newParticle.colorG = color.y;
          newParticle.colorB = color.z;
          newParticle.colorA = 1.0f;
          particles.push_back(newParticle);
        }
      } */

      // ADD LINE RECORDING FOR ARTIST VERSION
      if (drawingOn /*&& totalCtr % UPDATE_COUNTER == 0*/ &&
         (newTrackData.x != 0 || newTrackData.y != 0 || newTrackData.z != 0))
      {

        currentLine[splitLine[0]].r = lineRed;
        currentLine[splitLine[0]].g = lineGreen;
        currentLine[splitLine[0]].b = lineBlue;

        currentLine[splitLine[0]].x1 = currentLine[splitLine[0]].x2;
        currentLine[splitLine[0]].y1 = currentLine[splitLine[0]].y2;
        currentLine[splitLine[0]].z1 = currentLine[splitLine[0]].z2;

        currentLine[splitLine[0]].x2 = newTrackData.x;
        currentLine[splitLine[0]].y2 = newTrackData.y;
        currentLine[splitLine[0]].z2 = newTrackData.z;

        myline newLine;
        newLine.x1 = currentLine[splitLine[0]].x1; newLine.x2 = currentLine[splitLine[0]].x2;
        newLine.y1 = currentLine[splitLine[0]].y1; newLine.y2 = currentLine[splitLine[0]].y2;
        newLine.z1 = currentLine[splitLine[0]].z1; newLine.z2 = currentLine[splitLine[0]].z2;
        newLine.r = currentLine[splitLine[0]].r;
        newLine.g = currentLine[splitLine[0]].g;
        newLine.b = currentLine[splitLine[0]].b;

        lineBufferHead++;
        if (lineBufferHead >= ART_BUFFER_SIZE) lineBufferHead = 0;
        if (lines[splitLine[0]].size() < ART_BUFFER_SIZE) lines[splitLine[0]].push_back(newLine);
        else lines[splitLine[0]][lineBufferHead] = newLine;
      }
      // END LINE RECORDING FOR ARTIST VERSION

      if (!simulation) {  // counting for live tracking
        totalCtr++;
        if (trackNames.size() > 0) {
          if (totalCtr % trackNames.size() == 0) {
            averageDistanceHelper();
          }
        }
      }

    // end check for valid input line
    } else if (simulation) { // counting for data dump reading
      totalCtr++;
      averageDistanceHelper();
      // compute average proximity

// DEBUG CODE
//if (logger) {
//  outputFile << "trackNames.size() is " << boost::lexical_cast<string>(trackNames.size()) << "\n";
//  outputFile << "numTrackedObjects is " << boost::lexical_cast<string>(numTrackedObjects) << "\n";
//}
// END DEBUG CODE

    }
  } // end receive loop
}

string effName;

void display() {

  // auto close
  framesPassed++;
  if (receivedPacket) {
    if (framesPassed > 180) {
      //if (!simulation && outputFile.is_open()) outputFile.close(); 
      exit(0);
    }
  } else {
    if (framesPassed > 900) {
      //if (!simulation && outputFile.is_open()) outputFile.close(); 
      exit(0);
    }
  }

  // color changing
  lineRed += COLOR_CHANGE * lineRedDir;
  lineGreen += COLOR_CHANGE * lineGreenDir;
  lineBlue += COLOR_CHANGE * lineBlueDir;
  if (lineRed <= 0) lineRedDir = 1.0;
  else if (lineRed >= 1) lineRedDir = -1.0;
  if (lineGreen <= 0) lineGreenDir = 1.0;
  else if (lineGreen >= 1) lineGreenDir = -1.0;
  if (lineBlue <= 0) lineBlueDir = 1.0;
  else if (lineBlue >= 1) lineBlueDir = -1.0;

  // display

//  glEnable(GL_LIGHTING) ;
//  glEnable(GL_LIGHT0);
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_NORMALIZE);
  glEnable(GL_DEPTH_TEST);

  glClearColor(0, 0, 0, 0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glFrustum(ortho_left, ortho_right, ortho_bottom, ortho_top, 0.1, 5000);
//  gluPerspective(45, screenAspectRatio, .1, 5000);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(0,4,1,
            0,0,1,
            0,0,1);

  glTranslatef(0.0f, 3.5f, 0.0f);

  // draw huge wireframe cubes that spin in response to user movement
  // disabled; doesn't seem useful for a drawing application.
  /*glBlendFunc(GL_ONE, GL_ZERO);
  glLineWidth(5.0f);
  float avgVel = calculateAverageVelocity();
  cubeRotationA += (avgVel * 3);
  cubeRotationB += (avgVel * 5);
  cubeRotationC += (avgVel * 7);
  cubeColorR += 0.01f;
  cubeColorG += 0.03f;
  cubeColorB += 0.05f;
  if (cubeColorR > 1.0f) cubeColorR = 0.0f;
  if (cubeColorG > 1.0f) cubeColorG = 0.0f;
  if (cubeColorB > 1.0f) cubeColorB = 0.0f;
  glColor4f(cubeColorR, cubeColorG, cubeColorB, 1.0f);
  glPushMatrix();
  glRotatef(cubeRotationA, 0.0f, 0.0f, 1.0f);
  glutWireSphere(6.0, 8, 8);
  glPopMatrix();
  glColor4f(cubeColorB, cubeColorR, cubeColorG, 1.0f);
  glPushMatrix();
  glRotatef(cubeRotationB, 0.0f, 1.0f, 0.0f);
  glutWireSphere(6.0, 8, 8);
  glPopMatrix();
  glColor4f(cubeColorG, cubeColorB, cubeColorR, 1.0f);
  glPushMatrix();
  glRotatef(cubeRotationC, 1.0f, 0.0f, 0.0f);
  glutWireSphere(6.0, 8, 8);
  glPopMatrix();
  glLineWidth(1.0f); */
  glBlendFunc(GL_SRC_COLOR, GL_DST_COLOR);

  for (int i = 0; i < trackNames.size(); i++) {
    glPushMatrix();

    // basic display
    effName = trackNames[i];

    int tmpBufferHead = getTmpBufferHead(effName);
    trackable color = getColors(effName);

    if (trackHistory[effName][tmpBufferHead].z != 0) {
      glTranslatef(trackHistory[effName][tmpBufferHead].x,
        trackHistory[effName][tmpBufferHead].y,
        trackHistory[effName][tmpBufferHead].z);
      glColor3f(color.x, color.y, color.z);
      glutSolidSphere(0.1, 12, 12);
      glPopMatrix();
    }
    // display afterimages (trail)
    float runningSize = 0.1f;
    float runningAlpha = 1.0f;
    if (afterImages[effName].size() == numAfterImages) {
      for (int a = 0; a < numAfterImages; a++) {
        if (afterImages[effName][a].z != 0) {
          glPushMatrix();
          glTranslatef(afterImages[effName][a].x,
            afterImages[effName][a].y,
            afterImages[effName][a].z);
          glColor4f(color.x, color.y, color.z, runningAlpha);
          glutSolidSphere(runningSize, 8, 8);
          glPopMatrix();
          runningSize -= 0.005f;
          runningAlpha -= 0.05f;
        }
      }
    }
  } // end loop thru trackNames

  // draw lines
  for (int t = 0; t < trackNames.size(); t++) {
    effName = trackNames[t];
    for (int i = 0; i < lines[effName].size(); i++) {
      myline cline = lines[effName][i];
      float lineWidth = (cline.y1 + 2) * LINE_THICKNESS * 1.5f;
      glLineWidth(lineWidth);
      glColor3f(cline.r, cline.g, cline.b);
      glBegin(GL_LINES);
      glVertex3f(cline.x1, cline.y1, cline.z1);
      glVertex3f(cline.x2, cline.y2, cline.z2);
      glEnd();
    } // end loop thru lines
  }

  // display particles
  /*for (int i = particles.size() - 1; i >= 0; i--) {
    particles[i].colorA -= 0.0025f;
    if (particles[i].colorA <= 0) particles.erase(particles.begin() + i);
    else {
      particles[i].x += particles[i].x_vel;
      particles[i].y += particles[i].y_vel;
      particles[i].z += particles[i].z_vel;
      glPushMatrix();
      glTranslatef(particles[i].x, particles[i].y, particles[i].z);
      glColor4f(particles[i].colorR, particles[i].colorG,
        particles[i].colorB, particles[i].colorA);
      glutSolidCube(0.07);
      glPopMatrix();
    }
  } */

  glutSwapBuffers();
  glutPostRedisplay();
}

int main(int argc, char** argv) {
  if (argc < 7) {
    printf("USAGE: GestureResponseSlave left right bottom top num_tracked_objects simulation\n");
    return 1;
  }

  ortho_left = atof(argv[1]); //-5.0;
  ortho_right = atof(argv[2]); //0.0;
  ortho_bottom = atof(argv[3]); //-5.0;
  ortho_top = atof(argv[4]); //5.0;
  numTrackedObjects = atoi(argv[5]);
  simulation = (strcmp(argv[6], "FALSE") != 0);
  //if (!simulation) outputFile.open(argv[6]);

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
  glutInitWindowSize(SCREEN_WIDTH, SCREEN_HEIGHT);
  glutInitWindowPosition(0, 0);
  glutCreateWindow("Gesture Responder Slave Node");
  glShadeModel(GL_SMOOTH);
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_COLOR, GL_DST_COLOR);
  glutDisplayFunc(display);

  // socket stuff
  slen=sizeof(si_other);
  if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) error("ERROR socket");
  memset((char *) &si_me, 0, sizeof(si_me));
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(PORT);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(s, (struct sockaddr*)&si_me, sizeof(si_me)) == -1) error("ERROR bind");

  // listen for updates
  if (pthread_create(&receiverThread, NULL, receiver, NULL) != 0) {
    perror("Can't start thread, terminating");
    return 1;
  }

  glutMainLoop();

  return 0;
}
