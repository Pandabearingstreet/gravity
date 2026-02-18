#include <unistd.h>
#include <ncurses.h>
#include <stdlib.h>
#include <string>
#include <vector>
std::string pallete = "MQW#BNqpHERmKdgAGbX8@SDO$PUkwZyF69heT0a&xV%Cs4fY52Lonz3ucJjvItr}{li?1][7<>=)(+*|!/;:-,";

int MAXSIZE = 100;



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
  thing(std::vector<float> pos, std::vector<float> vel, float rad, float m) {
    this->position = pos;
    this->velocity = vel;
    this->radius = rad;
    this->mass = m;
  };
  void accelerate(float x, float y) {
    this->velocity[0] += x;
    this->velocity[1] += y;
  };
  void updatePos() {
    this->position[0] += this->velocity[0];
    this->position[1] += this->velocity[1];
  };
};

void drawThings(std::vector<thing> things) {
  for (int i = 0; i < things.size(); i++) {
    thing curThing = things[i];
    // find closest int coords to pos
    int LINEpos = curThing.position[0] / MAXSIZE * LINES;
    int COLpos = curThing.position[1] / MAXSIZE * COLS;
    // TODO, radius
    mvprintw(LINEpos, COLpos, "O");
  };
};
void moveThings(std::vector<thing> *things) {
  for (int i = 0; i < things->size(); i++) {
    things->at(i).updatePos();
    mvprintw(0,0,"moved thing %d", i);
    };
}
int main() {
  initscr();
  atexit(quit);
  curs_set(0);
  int i = 0;
  std::vector<thing> things = {thing({30, 50}, {0.5,-0.1}, 1, 1), thing({20, 50}, {0.1,-0.1}, 1, 1),
                               thing({70, 70}, {-0.1,0.1}, 1, 1)};
  
  
  while (i < 3000) {
    clearscr();
    drawThings(things);
    moveThings(&things);
    refresh();
    usleep(50000);
    i++;    
  }  
  return 1;  
  }  
