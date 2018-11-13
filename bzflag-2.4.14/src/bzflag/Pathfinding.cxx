/* bzflag
 *
 * This file was created by Cassandra Largosa.
 */

#include "Pathfinding.h"

Tile::Tile(float x, float z, int offset)
{
  tid[0] = (int)(x / SCALING_FACT);
  tid[1] = (int)(z / SCALING_FACT);

  id[0] = tid[0] + offset;
  id[1] = tid[1] + offset;

  reachablePoint[0] = x;
  reachablePoint[1] = z;
  reachablePoint[2] = 0.0f;
}

Tile::Tile(const float p[3], int offset)
{
  tid[0] = (int)(p[0] / SCALING_FACT);
  tid[1] = (int)(p[1] / SCALING_FACT);

  id[0] = tid[0] + offset;
  id[1] = tid[1] + offset;

  reachablePoint[0] = p[0];
  reachablePoint[1] = p[1];
  reachablePoint[2] = p[3];
}

Tile::Tile(int x, int z, int offset)
{
  id[0] = x;
  id[1] = z;

  tid[0] = x - offset;
  tid[1] = z - offset;

  reachablePoint[0] = (tid[0] * SCALING_FACT) + (SCALING_FACT / 2);
  reachablePoint[1] = (tid[1] * SCALING_FACT) + (SCALING_FACT / 2);
  reachablePoint[2] = 0.0f;
}

Tile::Tile(){}

const float * Tile::get()
{
  return reachablePoint;
}

const int * Tile::getId()
{
  return id;
}

const std::vector<Tile*> Tile::getConnections()
{
  return connections;
}

void Tile::connect(Tile* neighbor)
{
  bool canConnect = ((neighbor->id[0] == id[0] && (neighbor->id[1] == id[1] + 1 || neighbor->id[1] == id[1] - 1)) ||
                     (neighbor->id[1] == id[1] && (neighbor->id[0] == id[0] + 1 || neighbor->id[0] == id[0] - 1)));

  if (!canConnect)
  {
    for (int i = 0; i < (int)connections.size() && !canConnect; i++)
    {
      std::vector<Tile*>::iterator iter = std::find(begin(neighbor->connections), end(neighbor->connections), connections[i]);
      if (iter != end(neighbor->connections)) canConnect = true;
    }
  }

  canConnect = canConnect && !neighbor->obstucted() && !obstucted();

  std::vector<Tile*>::iterator iter = std::find(begin(connections), end(connections), neighbor);
  if (iter != end(connections)) canConnect = false;

  if (canConnect)
  {
    connections.push_back(neighbor);
    neighbor->connections.push_back(this);
  }
}

bool Tile::obstucted()
{
  float radius = SCALING_FACT / 2;
  return World::getWorld()->inBuilding(reachablePoint, radius, BZDBCache::tankHeight);
}

bool Tile::obstucted(const float p[2])
{
  float radius = SCALING_FACT / 2;
  return World::getWorld()->inBuilding(p, radius, BZDBCache::tankHeight);
}

Grid::Grid()
{
  sideLength = (int)(ceil(2 * BZDBCache::worldSize) / SCALING_FACT);
  if (sideLength <= 0) return;
  tiles.resize(sideLength);

  for (int x = 0; x < sideLength; x++)
  {
    tiles[x].resize(sideLength);
    for (int z = 0; z < sideLength; z++)
    {
      tiles[x][z] = Tile(x, z, (int)(sideLength / 2));

      if (x - 1 >= 0)
      {
        tiles[x][z].connect(&tiles[x-1][z]);
        if (z - 1 >= 0)
        {
          tiles[x][z].connect(&tiles[x-1][z-1]);
        }
        if (z + 1 < sideLength)
        {
          tiles[x][z].connect(&tiles[x-1][z+1]);
        }
      }
      if (z - 1 >= 0)
      {
        tiles[x][z].connect(&tiles[x][z-1]);
      }
    }
  }
}

Tile* Grid::mapToTile(const float p[2])
{
  Tile* t = inTile(p);

  if (t->obstucted())
  {
    float ddif = SCALING_FACT / sqrtf(2);
    float testPoints[8][2] = {
      {p[0] - SCALING_FACT, p[1]},
      {p[0] + SCALING_FACT, p[1]},
      {p[0], p[1] - SCALING_FACT},
      {p[0], p[1] + SCALING_FACT},
      {p[0] - ddif, p[1] - ddif},
      {p[0] - ddif, p[1] + ddif},
      {p[0] + ddif, p[1] - ddif},
      {p[0] + ddif, p[1] + ddif}
    };
    
    for (int i = 0; i < 8; i++)
    {
      if (testPoints[i][0] >= 0 && testPoints[i][1] >= 0 && testPoints[i][0] < BZDBCache::worldSize && testPoints[i][1] < BZDBCache::worldSize &&
          !Tile::obstucted(testPoints[i]) && !inTile(testPoints[i])->obstucted())
      {
        t = inTile(testPoints[i]);
      }
    }
  }

  return t;
}

Tile * Grid::inTile(const float p[2])
{
  int tid[2];

  tid[0] = (int)(p[0] / SCALING_FACT) + (int)(sideLength / 2);
  tid[1] = (int)(p[1] / SCALING_FACT) + (int)(sideLength / 2);

  return &tiles[tid[0]][tid[1]];
}

int Grid::getX()
{
  return sideLength;
}

int Grid::getZ()
{
  return sideLength;
}

Node::Node(Tile * cur, Tile * prev, float cost, float estimate, NodeStatus stat)
{
  t = cur;
  previous = prev;
  costSoFar = cost;
  estamatedTotalCost = estimate;
  status = stat;
}

Node::Node()
{
  t = nullptr;
  previous = nullptr;
  costSoFar = 0.0;
  estamatedTotalCost = 0.0;
  status = Unvisited;
}

bool Node::betterNode(Node testNode, Node compNode)
{
  return testNode.estamatedTotalCost < compNode.estamatedTotalCost;
}

std::vector<Tile> Node::aStar(Grid* graph, Tile* start, Tile* end)
{
  std::priority_queue<Node, std::vector<Node>, CompareNode> openList;
  std::vector<std::vector<Node>> closedList(graph->getX(), std::vector<Node>(graph->getZ(), Node()));
  Node startNode = Node(start, nullptr, 0.0, heuristic(start, end), Open);
  closedList[start->getId()[0]][start->getId()[1]] = startNode;
  openList.push(startNode);
  Node current = Node();

  while (!openList.empty())
  {
    current = openList.top();
    openList.pop();
    if (current.t == end) break;
    Node closedSlot = closedList[current.t->getId()[0]][current.t->getId()[1]];
    if (closedSlot.status != Unvisited)
    {
      if (closedSlot.status == Closed && !Node::betterNode(current, closedSlot)) continue;
      if (closedSlot.status == Open && Node::betterNode(closedSlot, current)) continue;
    }

    current.status = Closed;
    closedList[current.t->getId()[0]][current.t->getId()[1]] = current;

    std::vector<Tile*> connections = current.t->getConnections();

    for (int i = 0; i < (int)connections.size(); i++)
    {
      Node next = Node();
      next.t = connections[i];
      next.previous = current.t;
      next.costSoFar = current.costSoFar + moveCost(current.t, next.t);
      closedSlot = closedList[next.t->getId()[0]][next.t->getId()[1]];

      if (closedSlot.status != Unvisited)
      {
        if (closedSlot.costSoFar <= next.costSoFar) continue;
        next.estamatedTotalCost = next.costSoFar + closedSlot.estamatedTotalCost - closedSlot.costSoFar;
      }
      else next.estamatedTotalCost = next.costSoFar + heuristic(next.t, end);

      next.status = Open;

      closedList[next.t->getId()[0]][next.t->getId()[1]] = next;
      openList.push(next);
    }
  }
  std::vector<Tile> path;
  path.clear();

  if (current.t != end) return path;

  Node smoothStart = current;
  Node smoothEnd = closedList[current.previous->getId()[0]][current.previous->getId()[1]];
  while (current.t != start)
  {
    current = closedList[current.previous->getId()[0]][current.previous->getId()[1]];
    if (smoothStart.t->getId()[0] != current.t->getId()[0] || smoothStart.t->getId()[1] != current.t->getId()[1])
    {
      path.push_back(*smoothStart.t);
      smoothStart = smoothEnd;
    }
    smoothEnd = current;
  }

  std::reverse(path.begin(), path.end());
  return path;
}

float heuristic(Tile* here, Tile* goal)
{
  float h = 0;
  h += hypotf(goal->getId()[0] - here->getId()[0], goal->getId()[1] - here->getId()[1]); // Euclidean distance
  return h;
}

float moveCost(Tile* t1, Tile* t2)
{
  float cost = 0.0;
  if (t1->getId()[0] == t2->getId()[0] || t1->getId()[1] == t2->getId()[1]) cost += 1.0; else cost += sqrtf(2);
  return cost;
}
