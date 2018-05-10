

#ifndef __OPERATIONS_H__
#define __OPERATIONS_H__


// comparison nodes for our A* alg
namespace PP {
bool operator<(const Nodea &n1, const Nodea &n2) {
  return n1.cost > n2.cost;
}

bool operator==(const Nodea &n1, const Nodea &n2) {
  return n1.idx == n2.idx;
}

}// namespace pathplanner

#endif //__OPERATIONS_H__