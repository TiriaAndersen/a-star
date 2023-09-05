#include <queue>
#include <limits>
#include <cmath>
#include <iostream>
using namespace std;

// represents a single pixel
class Node {
  public:
    int idx;     // index in the flattened grid
    float cost;  // cost of traversing this pixel
    float prob;  // probability of finding another POI 

    Node(int i, float c, float p) : idx(i),cost(c),prob(p) {}
};

// the top of the priority queue is the greatest element by default,
// but we want the smallest, so flip the sign
bool operator<(const Node &n1, const Node &n2) {
  if(n1.cost==n2.cost)
  {
    return n1.prob < n2.prob;
  }
  else
  {
    return n1.cost > n2.cost;
  }
}

bool operator==(const Node &n1, const Node &n2) {
  return n1.idx == n2.idx;
}

// See for various grid heuristics:
// http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html#S7
// L_\inf norm (diagonal distance)
float linf_norm(int i0, int j0, int i1, int j1) {
  return max(abs(i0 - i1), abs(j0 - j1));
}

// L_1 norm (manhattan distance)
float l1_norm(int i0, int j0, int i1, int j1) {
  return abs(i0 - i1) + abs(j0 - j1);
}

// Print formatted grid
void print_grid(float* grid, int h, int w){
  cout << endl;
  for (int i=0; i <=(h*w-1); i++){
    if (i%w == 0) {cout << endl;}
    cout <<  grid[i] << " ";
  }
  cout << endl;
}

// weights:        flattened h x w grid of costs
// h, w:           height and width of grid
// d:              unit distance between grid cells
// start, goal:    index of start/goal in flattened grid
// diag_ok:        if true, allows diagonal moves (8-conn.)
// paths (output): for each node, stores previous node in path
extern "C" bool astar(
      const float* weights, const int h, const int w, const int d,
      const int start, const int goal, bool diag_ok,
      int* paths) {

  const float INF = numeric_limits<float>::infinity();

  Node start_node(start, 0., 0);
  Node goal_node(goal, 0., 0);

  float* costs = new float[h * w];
  for (int i = 0; i < h * w; ++i)
    costs[i] = INF;
  costs[start] = 0.;

  float* priorities = new float[h * w];

  priority_queue<Node> nodes_to_visit;
  nodes_to_visit.push(start_node);

  int* nbrs = new int[8];

  bool solution_found = false;
  while (!nodes_to_visit.empty()) {
    // .top() doesn't actually remove the node
    Node cur = nodes_to_visit.top();
    
    cout << endl << "Popped: " << cur.idx << " cost: " << cur.cost << endl;

    if (cur == goal_node) {
      solution_found = true;
      break;
    }

    nodes_to_visit.pop();

    int row = cur.idx / w;
    int col = cur.idx % w;
    // check bounds and find up to eight neighbors: top to bottom, left to right
    nbrs[0] = (diag_ok && row > 0 && col > 0)          ? cur.idx - w - 1   : -1;
    nbrs[1] = (row > 0)                                ? cur.idx - w       : -1;
    nbrs[2] = (diag_ok && row > 0 && col + 1 < w)      ? cur.idx - w + 1   : -1;
    nbrs[3] = (col > 0)                                ? cur.idx - 1       : -1;
    nbrs[4] = (col + 1 < w)                            ? cur.idx + 1       : -1;
    nbrs[5] = (diag_ok && row + 1 < h && col > 0)      ? cur.idx + w - 1   : -1;
    nbrs[6] = (row + 1 < h)                            ? cur.idx + w       : -1;
    nbrs[7] = (diag_ok && row + 1 < h && col + 1 < w ) ? cur.idx + w + 1   : -1;

    float heuristic_cost;
    for (int i = 0; i < 8; ++i) {
      if (nbrs[i] >= 0) {
        // the sum of the cost so far and the cost of this move
        float new_cost = costs[cur.idx] + (d - weights[nbrs[i]]);
        if (new_cost < costs[nbrs[i]]) {
          // estimate the cost to the goal based on legal moves
          if (diag_ok) {
            heuristic_cost = linf_norm(nbrs[i] / w, nbrs[i] % w,
                                       goal    / w, goal    % w);
          }
          else {
            heuristic_cost = l1_norm(nbrs[i] / w, nbrs[i] % w,
                                     goal    / w, goal    % w);
          }

          // paths with lower expected cost are explored first
          // float priority = new_cost + heuristic_cost;
          float priority = new_cost;
          nodes_to_visit.push(Node(nbrs[i], priority, weights[nbrs[i]]));

          priorities[nbrs[i]] = priority;
          costs[nbrs[i]] = new_cost;
          paths[nbrs[i]] = cur.idx;
        }
      }
    }

    // print backpointer grid
    cout << endl << "backpointers:" << endl;
    for (int i=0; i <=15; i++){
      if (i%4 == 0) {cout << endl;}
      cout <<  paths[i] << " ";
    }
    cout << endl;

    // print cost grid
    cout << endl << "costs:";
    print_grid(costs, h, w);

    // print priority grid
    // cout << endl << "priorities:";
    // print_grid(priorities, h, w);
    
  }

  delete[] costs;
  delete[] nbrs;

  return solution_found;
}

int main()
{
    // Parameters.
  int h = 4;
  int w = 4;
  int d = 1;
  int start = 12;
  int goal = 3;
  bool diag_ok = true;
  int paths[w*h];

  // print grid index
  for (int i=0; i <=15; i++){
    if (i%4 == 0) {cout << endl;}
    cout << i << " ";
  }
  cout << endl;

  // Print weights.
  float weights[16] = {1, 0.5, 0.5, 1, 0.5, 0.5, 0, 0, 0.2};
  print_grid(weights, h, w);
  // for (int i=0; i <=15; i++){
  //   if (i%4 == 0) {cout << endl;}
  //   cout <<  weights[i] << " ";
  // }

  // Run A*
  bool solution_found = astar(weights, h, w, d, start, goal, diag_ok, paths);
  cout << "Solution found? "  << solution_found << endl;

  // print solution path (remember, index starts at 0 in the top-left corner)
  float total_cost = 0;
  int path_idx = goal;
  cout << goal << " ";
  while (path_idx != start){
    cout << paths[path_idx]  << " ";
    path_idx = paths[path_idx];
    total_cost += d - weights[path_idx];
  }
  cout << endl << "total cost: " << total_cost << endl;

}
