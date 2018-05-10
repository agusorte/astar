#ifndef __ASTAR_H__
#define __ASTAR_H__

#include <queue>
#include <limits>
#include <cmath>
#include <string.h>

// my namespace pathplanning

namespace PP {
// represents a single pixel/grid in the map
class Nodea 
{
  public:
    int idx;     // index in the flattened grid
    float cost;  // cost of traversing this pixel final cost will be the time passed

    Nodea(int i, float c) : idx(i),cost(c) {}

};


// class that compute A* algorithm
class Astar
{
  public:

    // contructstors 
  	Astar();
    //dest
    ~Astar();
  
  	// width and height of the grid
  	Astar(int w, int h);

    // we enable diagonal motion
  	void setDiagonalMovement(bool enable);

    //grid map or weights of the map
    void setMap(float* &map);
  	
    // start, goal:    index of start/goal in flattened grid
    // diag_ok:        if true, allows diagonal moves (8-conn.)
    // paths (output): for each node, stores previous node in path
    // cost_all comulative cost of the path
    // return true if path found
    bool findPath(const float* weights, const int h, const int w,
      const int start, const int goal, bool diag_ok,int* paths,
      float& cost_all);

    // just the version overloaded
    bool findPath(const int start,const int goal,
      int* paths, float& cost_all);

  


  private:

    // L_\inf norm (diagonal distance)
    // when we have infinity
    float linf_norm(int i0, int j0, int i1, int j1) {
    return std::max(std::abs(i0 - i1), std::abs(j0 - j1));
    };

    // L_1 norm (manhattan distance)
    float l1_norm(int i0, int j0, int i1, int j1) {
    return std::abs(i0 - i1) + std::abs(j0 - j1);
    };

    // grid dimentions
    int mwidth;
    int mheight;
    // grid pointer
    float* mgrid;
    // results path
    bool mdiagonal_ok;


  
};

}

#endif // __ASTAR_H__
