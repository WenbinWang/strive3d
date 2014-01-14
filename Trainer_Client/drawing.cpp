/*
 *  Copyright (C) 2011 Justin Stoecker
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

/* 
 * This source file uses some basic datagram socket communication code from
 * Brian "Beej Jorgensen" Hall's Guide to Network Programming:
 * http://beej.us/guide/bgnet/output/html/multipage/clientserver.html#datagram
 */


#include "drawing.h"
#define ROBOVIS_HOST "localhost"
#define ROBOVIS_PORT "32769"
#define TEST_DURATION 10000

Drawing::Drawing()
{
  angle = 0.0;
  connect();
  rd = new Rvdraw();
}


void Drawing::swapBuffers(const string* setName) {
  int bufSize = -1;
  unsigned char* buf = rd->newBufferSwap(setName, &bufSize);
  sendto(sockfd, buf, bufSize, 0, p->ai_addr, p->ai_addrlen);
  delete[] buf;
}

void Drawing::drawLine(float x1, float y1, float z1, float x2, float y2, float z2, float thickness, float r, float g, float b,
    const string* setName) {
  float pa[3] = {x1,y1,z1};
  float pb[3] = {x2,y2,z2};
  float color[3] = {r,g,b};

  int bufSize = -1;
  unsigned char* buf = rd->newLine(pa, pb, thickness, color, setName, &bufSize);
  sendto(sockfd, buf, bufSize, 0, p->ai_addr, p->ai_addrlen);
  delete[] buf;
}

void Drawing::drawCircle(float x, float y, float radius, float thickness, float r, float g, float b, const string* setName) {
  float center[2] = {x,y};
  float color[3] = {r,g,b};

  int bufSize = -1;
  unsigned char* buf = rd->newCircle(center, 2.0f, 2.5f, color, setName, &bufSize);
  sendto(sockfd, buf, bufSize, 0, p->ai_addr, p->ai_addrlen);
  delete[] buf;
}

void Drawing::drawSphere(float x, float y, float z, float radius, float r, float g, float b, const string* setName) {
  float center[3] = {x,y,z};
  float color[3] = {r,g,b};

  int bufSize = -1;
  unsigned char* buf = rd->newSphere(center, radius, color, setName, &bufSize);
  sendto(sockfd, buf, bufSize, 0, p->ai_addr, p->ai_addrlen);
  delete[] buf;
}

void Drawing::drawPoint(float x, float y, float z, float size, float r, float g, float b, const string* setName) {
  float center[3] = {x,y,z};
  float color[3] = {r,g,b};

  int bufSize = -1;
  unsigned char* buf = rd->newPoint(center, size, color, setName, &bufSize);
  sendto(sockfd, buf, bufSize, 0, p->ai_addr, p->ai_addrlen);
  delete[] buf;
}

void Drawing::drawPolygon(const float* v, int numVerts, float r, float g, float b,
    float a, const string* setName) {
  float color[4] = {r,g,b,a};

  int bufSize = -1;
  unsigned char* buf = rd->newPolygon(v, numVerts, color, setName, &bufSize);
  sendto(sockfd, buf, bufSize, 0, p->ai_addr, p->ai_addrlen);
  delete[] buf;
}

//void drawAnnotation(const string* text, float x, float y, float z, float r,
//    float g, float b, const string* setName) {
//  float color[3] = {r,g,b};
//  float pos[3] = {x,y,z};
//
//  int bufSize = -1;
//  unsigned char* buf = newAnnotation(text, pos, color, setName, &bufSize);
//  sendto(sockfd, buf, bufSize, 0, p->ai_addr, p->ai_addrlen);
//  delete[] buf;
//}

float Drawing::maxf(float a, float b) {
  return a > b ? a : b;
}

void Drawing::renderAnimatedShapes() {
  angle += 0.05;

  string n1("animated.points");
  for (int i = 0; i < 60; i++) {
    float p = i / 60.0f;
    float height = maxf(0, sin(angle + p * 18));
    drawPoint(-9 + 18 * p, p * 12 - 6, height, 5, 0,0,0,&n1);
  }

  float bx = cos(angle) * 2;
  float by = sin(angle) * 2;
  float bz = cos(angle) + 1.5f;

  string n2("animated.spinner");
  drawLine(0,0,0,bx,by,bz,5,1,1,0,&n2);
  drawLine(bx,by,bz,bx,by,0,5,1,1,0,&n2);
  drawLine(0,0,0,bx,by,0,5,1,1,0,&n2);

  string n3("animated.annotation");
  char tbuf[4];
  int result = snprintf(tbuf, 4, "%.1f", bz);
  string aText(tbuf);
  //drawAnnotation(&aText, bx, by, bz, 0, 1, 0, &n3);

  string staticSets("animated");
  swapBuffers(&staticSets);
}

void Drawing::renderStaticShapes() {
  
  // draw 3D coordinate axes
  string n1("static.axes");
  drawLine(0,0,0,3,0,0,3,1,0,0,&n1);
  drawLine(0,0,0,0,3,0,3,0,1,0,&n1);
  drawLine(0,0,0,0,0,3,3,0,0,1,&n1);

  // draw 1 meter lines on field
  string n2("static.lines.field");
  drawLine(-9,-6,0,9,-6,0,1,0.6f,0.9f,0.6f,&n2);
  drawLine(-9,6,0,9,6,0,1,0.6f,0.9f,0.6f,&n2);
  for (int i = 0; i <= 18; i++)
    drawLine(-9+i,-6,0,-9+i,6,0,1,0.6f,0.9f,0.6f,&n2);

  // draw some circles
  string n3("static.circles");
  drawCircle(-5,0,3,2,0,0,1,&n3);
  drawCircle(5,0,3,2,0,0,1,&n3);

  // draw some spheres
  string n4("static.spheres");
  drawSphere(-5,0,2,0.5f,1,0,0.5f,&n4);
  drawSphere(5,0,2,0.5f,1,0,0.5f,&n4);

  // draw a polygon
  string n5("static.polygons");
  float v[] = {0,0,0, 1,0,0, 1,1,0, 0,3,0, -2,-2,0};
  drawPolygon(v, 4, 1, 1, 1, 0.5f, &n5); 

  string staticSets("static");
  swapBuffers(&staticSets);
}

void Drawing::runTest() {
  renderStaticShapes();
  for (int i = 0; i < TEST_DURATION / 17; i++) {
    renderAnimatedShapes();
    usleep(16000);
  }
}


int Drawing::connect(){
  
    
    int rv;
    int numbytes;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;

    if ((rv = getaddrinfo(ROBOVIS_HOST, ROBOVIS_PORT, &hints, &servinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return 1;
    }

    // loop through all the results and make a socket
    for(p = servinfo; p != NULL; p = p->ai_next) {

        if ((sockfd = socket(p->ai_family, p->ai_socktype,
                p->ai_protocol)) == -1) {
            perror("socket");
            continue;
        }

        break;
    }

    if (p == NULL) {
        fprintf(stderr, "failed to bind socket\n");
        return 2;
    }
  
}

Drawing::~Drawing()
{
    freeaddrinfo(servinfo);
    close(sockfd);
}

