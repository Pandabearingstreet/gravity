#include "eigen3/Eigen/Dense"
#include <math.h>
#include <unistd.h>
#include <ncurses.h>
#include <stdlib.h>
#include <vector>
#include <cassert>

using Eigen::Vector3d;

std::string zPallete = "MQW#BNqpHERmKdgAGbX8@SDO$PUkwZyF69heT0a&xV%Cs4fY52Lonz3ucJjvItr}{li?1][7<>=)(+*|!/;:-,";

int MAXSIZE = 5000;
float SPEEDLIMIT = 100;
float dt = 10;

void quit() { endwin(); }
void clearscr() {
    for (int j = 0; j <= LINES; j++) {
      for (int k = 0; k <= COLS; k++) {
        mvprintw(j, k, " ");
      }
    }
}

class thing {
public:
  Vector3d position;
  Vector3d velocity;
  float radius;
  float mass;
  int color;
  thing(Vector3d pos, Vector3d vel, float rad, float m, int c) {
    position = pos;
    velocity = vel;
    radius = rad;
    mass = m;
    color = c;    
  }
  void accelerate(Vector3d impuls) {
    Vector3d incomingVel = impuls / mass;
    velocity = (incomingVel + velocity).array()/( 1 + (incomingVel.array()*velocity.array())/(SPEEDLIMIT*SPEEDLIMIT));
  }
  void updatePos() {
    position += velocity;      
  }
};

void drawThings(std::vector<thing> things,  char texture) {

  struct {
    bool operator()(thing a, thing b) const {
      return a.position[2] < b.position[2];
    }
  } zPositionsLess;

  std::sort(things.begin(), things.end(), zPositionsLess);
  
  for (size_t i = 0; i < things.size(); i++) {
    thing curThing = things[i];
    // find closest int coords to pos
    int LINEpos = curThing.position[0] / MAXSIZE * LINES;
    int COLpos = curThing.position[1] / MAXSIZE * COLS;
    int Zpos = curThing.position[2] / MAXSIZE * zPallete.size();
    // set color
    color_set(curThing.color, 0);
    float zSizeOffset = curThing.position[2] / MAXSIZE;
    // TODO, radius
    int LINErad = (curThing.radius * zSizeOffset) / MAXSIZE * LINES;
    int COLrad = (curThing.radius * zSizeOffset) / MAXSIZE * COLS;    

    if (texture == 'O') {
      if (size_t(Zpos) < zPallete.size()){
	texture = zPallete.at(Zpos);
      }
    }
    
    
    mvaddch(LINEpos, COLpos, texture); // draw center
    //draw a circle around, j^2+k^2 <r
    for (int j = -LINErad; j <= LINErad; j++) {
      for (int k = -COLrad; k <= COLrad; k++) {
	if (j*j + k*k  < sqrt(COLrad*LINErad)) {
	  mvaddch(LINEpos + j, COLpos + k, texture);     
	}
      }
    }    
  }
}

void checkBoundaries(std::vector<thing> *things) {
  for (size_t i = 0; i < things->size(); i++) {
    thing& curThing = things->at(i);
    if (curThing.position[0] <= 10 || curThing.position[0] > MAXSIZE-10) {
      curThing.velocity[0] *= -1;
      curThing.updatePos();
      curThing.velocity[0] *= 0.8;
      }
    if (curThing.position[1] <= 10 || curThing.position[1] > MAXSIZE-10) {
      curThing.velocity[1] *= -1;
      curThing.updatePos();
      curThing.velocity[1] *= 0.8;
    }
    if (curThing.position[2] <= 20 || curThing.position[2] > MAXSIZE - 20) {
      curThing.velocity[2] *= -1;
      curThing.updatePos();
      curThing.velocity[2] *= 0.8;
    }      
  }  
}
void checkCollisions(std::vector<thing> *things) {
  for (size_t i = 0; i < things->size(); i++) {
    for (size_t j = i + 1; j < things->size(); j++) {
      
      thing& curThing = things->at(i);
      thing& compThing = things->at(j);
      Vector3d dpos = curThing.position - compThing.position;
      float distanceSqrd = dpos.dot(dpos);
      if (distanceSqrd < (curThing.radius + compThing.radius) *
                             (curThing.radius + compThing.radius)) {
                
	Vector3d dvel = curThing.velocity - compThing.velocity;

	dpos.normalize();
	float velocityAlongNormalVector = dpos.dot(dvel);

	float e = 0.8; //coeficient of efficiency

	float impulsMag = -(1 + e) * velocityAlongNormalVector;
        impulsMag /= (1 / curThing.mass + 1 / compThing.mass);
	Vector3d impuls = dpos * impulsMag;
        curThing.accelerate(impuls);
	compThing.accelerate(-impuls);
      }
      
    }
  }
}
void gravitate(std::vector<thing> *things) {
  for (size_t i = 0; i < things->size(); i++) {
    for (size_t j = i+1; j < things->size(); j++) {
      
      thing& curThing = things->at(i);
      thing& compThing = things->at(j);
      Vector3d dpos = compThing.position - curThing.position;
      float distance = dpos.norm();
      if (distance * 0.8 < curThing.radius + compThing.radius) {
        distance = (curThing.radius + compThing.radius) * 0.8;        
      }
      
      float gForce =
	(9.81 * curThing.mass * compThing.mass) / (distance * distance * distance);
      Vector3d directionalImpuls = gForce * dt * dpos;
      curThing.accelerate(directionalImpuls);
      compThing.accelerate(-directionalImpuls);
    }
  }
}
  
void moveThings(std::vector<thing> *things) {
  for (size_t i = 0; i < things->size(); i++) {
    thing& curThing = things->at(i);    
    curThing.updatePos();
    }
}
int main() {
  initscr();
  atexit(quit);
  curs_set(0);
  start_color();  
  init_pair(1, COLOR_RED, 0);
  init_pair(2, COLOR_GREEN, 0);
  init_pair(3, COLOR_BLUE, 0);
  init_pair(4, COLOR_YELLOW, 0);
  init_pair(5, COLOR_CYAN, 0);
  init_pair(6, COLOR_MAGENTA, 0);
  int oldLines = LINES;


  std::vector<thing> things = {thing({350, 1450, 1000}, {0, 0, 0}, 700, 700, 1),
                               thing({1000, 100, 1900}, {+10, +50, -500}, 50, 500,2),
                               thing({950, 2955, 850}, {0, -50, 0}, 400, 400, 3),
			       thing({2500, 2500, 2500}, {0,0, 0}, 700, 17000, 4),
			       thing({100, 100, 2500}, {0,0, -50}, 500, 500, 5),
                               thing({4050, 195, 400}, {+50,-50, 0}, 400, 400, 6)};

  while (TRUE) {

    // check LINES for Change, to clear screen on resize
    if (oldLines == LINES) {
      clearscr();
    }
    oldLines = LINES;
    
    drawThings(things, ' ');
    checkBoundaries(&things);
    //checkCollisions(&things);
    gravitate(&things);
    moveThings(&things);
    drawThings(things, 'O');
    refresh();
    usleep(dt*1000);
  }  
  return 1;  
  }  
