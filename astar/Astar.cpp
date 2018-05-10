#include "Astar.h"
#include "Operations.h"
#include <iostream> 

namespace PP // pathplannin namespace
{

Astar::Astar()
{
  delete mgrid;

}
Astar::~Astar()
{

}

Astar::Astar(int w, int h)
{
  mwidth = w;
  mheight = h;

  mgrid = new float[w*h];
 
  // infinity values for obstacles
  const float INF = std::numeric_limits<float>::infinity();

  // we fill pointers
  memset(mgrid ,INF, sizeof(mgrid));  
   


}

void Astar::setDiagonalMovement(bool enable)
{
	mdiagonal_ok = enable;
}

void Astar::setMap(float* &map)
{
  mgrid = map;

}



bool Astar::findPath(const float* weights, const int h, const int w, const int start, const int goal, bool diag_ok, int* paths, float& cost_all)
{

  const float INF = std::numeric_limits<float>::infinity();

  Nodea start_node(start, 0.);
  Nodea goal_node(goal, 0.);

  float* costs = new float[h * w];

  // paths = new int[w*h];
  // memset(paths, -1, sizeof(paths));  

 // mcosts = new float[h * w];

  for (int i = 0; i < h * w; ++i)
    costs[i] = INF;
  costs[start] = 0.;

  std::priority_queue<Nodea> nodes_to_visit;
  nodes_to_visit.push(start_node);

  int* nbrs = new int[8];

  bool solution_found = false;
      float new_cost;

  while (!nodes_to_visit.empty()) 
  {
    // .top() doesn't actually remove the node
    Nodea cur = nodes_to_visit.top();

    if (cur == goal_node) {
      solution_found = true;
      break;
    }

    nodes_to_visit.pop();

    int row = cur.idx / w;
    int col = cur.idx % w;
    // check bounds and find up to eight neighbors: 
    //top to bottom, left to right
    nbrs[0] = (diag_ok && row > 0 && col > 0)          ? cur.idx - w - 1   : -1;
    nbrs[1] = (row > 0)                                ? cur.idx - w       : -1;
    nbrs[2] = (diag_ok && row > 0 && col + 1 < w)      ? cur.idx - w + 1   : -1;
    nbrs[3] = (col > 0)                                ? cur.idx - 1       : -1;
    nbrs[4] = (col + 1 < w)                            ? cur.idx + 1       : -1;
    nbrs[5] = (diag_ok && row + 1 < h && col > 0)      ? cur.idx + w - 1   : -1;
    nbrs[6] = (row + 1 < h)                            ? cur.idx + w       : -1;
    nbrs[7] = (diag_ok && row + 1 < h && col + 1 < w ) ? cur.idx + w + 1   : -1;

    float heuristic_cost;
    for (int i = 0; i < 8; ++i) 
    {
      if (nbrs[i] >= 0) 
      {

        // we add more time 
        // the sum of the cost so far and the cost of this move
        new_cost = costs[cur.idx] + weights[nbrs[i]];

        //BASED in the constraints
        // only if we compute for diagonal data
        // speed is sqrt(2) + elevation time
        if(i== 0 || i== 2 || i== 5 || i== 7)   
          new_cost+= sqrt(2)-1;

        // check cost
        if (new_cost < costs[nbrs[i]]) 
        {
          // estimate the cost to the goal based on legal moves
          if (diag_ok) {
            heuristic_cost = linf_norm(nbrs[i] / w, nbrs[i] % w,
                                       goal    / w, goal    % w);
          }
          else
           {
            heuristic_cost = l1_norm(nbrs[i] / w, nbrs[i] % w,
                                     goal    / w, goal    % w);
          }

          // paths with lower expected cost are explored first
          // this case seconds
          float priority = new_cost + heuristic_cost;
          nodes_to_visit.push(Nodea(nbrs[i], priority));

          costs[nbrs[i]] = new_cost;
          paths[nbrs[i]] = cur.idx;
        }
      }
    }
  }

  
  cost_all = new_cost;
  
  // free memory
  delete[] costs;
  delete[] nbrs;

  return solution_found;

}


bool Astar::findPath(const int start,const int goal, int* paths, float& costs)
{
	return findPath(mgrid,mwidth,mheight,start,goal,mdiagonal_ok,paths, costs);
}

}//namespace PP