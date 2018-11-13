#include "myNode.h"

//myNode::myNode() {
//  pos[0] = x;
//  pos[1] = y;
//}

float *myNode::getPos() {
  float pos[2];
  pos[0] = x;
  pos[1] = y;
  return pos;
}

// compares the x and y coordinates of the given node to the current node
bool myNode::operator==(const myNode& n) { return (x == n.x && y == n.y); }

