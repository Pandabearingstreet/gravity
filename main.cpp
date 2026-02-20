#include <math.h>
#include <unistd.h>
#include <ncurses.h>
#include <stdlib.h>
#include <vector>
#include <cassert>
int MAXSIZE = 1000;
float SPEEDLIMIT = 100;
float dt = 10;


float dotP(std::vector<float> a, std::vector<float> b) {
  assert(a.size() == b.size() && "vectors need to be of equal length for dotproduct");
  float sum = 0;
  for (size_t i = 0; i < a.size(); i++) {
    sum += a[i] * b[i];
  }
  return sum;
}
float magnitudeV(std::vector<float> a) {
  float sum = 0;  
  for (size_t i = 0; i < a.size(); i++) {
    sum += a[i];
  }
  return sum;  
}

std::vector<float> normalizeV(std::vector<float> a) {
  float m = magnitudeV(a);
  for (size_t i = 0; i < a.size(); i++) {
     a[i] /= m;
  }
  return a;  
}



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
  std::vector<float> position;
  std::vector<float> velocity;
  float radius;
  float mass;
  int color;
  thing(std::vector<float> pos, std::vector<float> vel, float rad, float m, int c) {
    position = pos;
    velocity = vel;
    radius = rad;
    mass = m;
    color = c;    
  }
  void accelerate(std::vector<float> impuls) {
    std::vector<float> incomingVel = {(impuls[0] / mass),(impuls[1] / mass)};
    velocity[0] = (incomingVel[0] + velocity[0])/(1+ (incomingVel[0]*velocity[0])/(SPEEDLIMIT*SPEEDLIMIT));
    velocity[1] = (incomingVel[1] + velocity[1])/(1+ (incomingVel[1]*velocity[1])/(SPEEDLIMIT*SPEEDLIMIT));
    }
  void updatePos() {
    position[0] += velocity[0];
    position[1] += velocity[1];
    }
};

void drawThings(std::vector<thing> things,  char texture) {
  for (size_t i = 0; i < things.size(); i++) {
    thing curThing = things[i];
    // find closest int coords to pos
    int LINEpos = curThing.position[0] / MAXSIZE * LINES;
    int COLpos = curThing.position[1] / MAXSIZE * COLS;
    // set color
    color_set(curThing.color, 0);

    // TODO, radius
    int LINErad = curThing.radius / MAXSIZE * LINES;
    int COLrad = curThing.radius /MAXSIZE *COLS;
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
    if (curThing.position[0] <= 5 || curThing.position[0] > MAXSIZE-5) {
      curThing.velocity[0] *= -1;
      curThing.updatePos();
      curThing.velocity[0] *= 0.5;
      }
    if (curThing.position[1] <= 5 || curThing.position[1] > MAXSIZE-5) {
      curThing.velocity[1] *= -1;
      curThing.updatePos();
      curThing.velocity[1] *= 0.5;
    }
  }  
}
void checkCollisions(std::vector<thing> *things) {
  for (size_t i = 0; i < things->size(); i++) {
    for (size_t j = i + 1; j < things->size(); j++) {
      
      thing& curThing = things->at(i);
      thing& compThing = things->at(j);
      float dx = curThing.position[0] - compThing.position[0];
      float dy = curThing.position[1] - compThing.position[1];
      float distanceSqrd = dx*dx + dy*dy;
      if (distanceSqrd < (curThing.radius + compThing.radius) *
                             (curThing.radius + compThing.radius)) {
                
	float dvx = curThing.velocity[0] - compThing.velocity[0];
	float dvy = curThing.velocity[1] - compThing.velocity[1];

	std::vector<float> n = normalizeV({dx, dy});
	float velocityAlongNormalVector = dotP(n, {dvx, dvy});

	float e = 0.5; //coeficient of efficiency

	float impulsMag = -(1 + e) * velocityAlongNormalVector;
        impulsMag /= (1 / curThing.mass + 1 / compThing.mass);
	std::vector<float> impuls = {n[0]*impulsMag, n[1]*impulsMag};        
	std::vector<float> negimpuls = {-impuls[0], -impuls[1]};
        curThing.accelerate(impuls);
	compThing.accelerate(negimpuls);
      }
      
    }
  }
}
void gravitate(std::vector<thing> *things) {
  for (size_t i = 0; i < things->size(); i++) {
    for (size_t j = i+1; j < things->size(); j++) {
      
      thing& curThing = things->at(i);
      thing& compThing = things->at(j);
      float dx = compThing.position[0] - curThing.position[0];
      float dy = compThing.position[1] - curThing.position[1];
      float distance = sqrt(dx*dx + dy*dy);
      if (distance < curThing.radius + compThing.radius) {
        distance = curThing.radius + compThing.radius;
      }
      
      float gForce =
	(9.81 * curThing.mass * compThing.mass) / (distance * distance * distance);
      std::vector<float> directionalImpuls = {gForce * dx * dt, gForce * dy * dt};
      curThing.accelerate(directionalImpuls);
      compThing.accelerate({-directionalImpuls[0], -directionalImpuls[1]});
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
  int i = 0;
  std::vector<thing> things = {thing({300, 450}, {0,0}, 70, 700, 1),
			       thing({100, 100}, {0,+50}, 50, 500, 2),
                               thing({950, 955}, {0,-50}, 10, 100, 3)};


  while (i < 30000) {    
    drawThings(things, ' ');
    checkBoundaries(&things);
    //checkCollisions(&things);
    gravitate(&things);
    moveThings(&things);
    drawThings(things, 'O');
    refresh();
    usleep(dt*5000);
    i++;    
  }  
  return 1;  
  }  
