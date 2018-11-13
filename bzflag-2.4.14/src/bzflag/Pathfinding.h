/* bzflag
 *
 * This file was created by Cassandra Largosa.
 */

#ifndef	TILE_H
#define	TILE_H

#include "common.h"
#include <queue>
#include "BZDBCache.h"
#include "World.h"

#ifndef	SCALING_FACT
#define	SCALING_FACT BZDBCache::tankRadius
#endif

class Tile {
  public:
                                       Tile(float x, float z, int offset);
                                       Tile(const float p[3], int offset);
                                       Tile(int x, int z, int offset);
                                       Tile();
    const float*                       get();
    const int*                         getId();
    const std::vector<Tile*>           getConnections();
    void                               connect(Tile* neighbor);
    bool                               obstucted();
    static bool                        obstucted(const float p[2]);
  private:
    int                                id[2];
    int                                tid[2];
    float                              reachablePoint[3];
    std::vector<Tile*>                 connections;
};

class Grid {
  public:
                                       Grid();

    Tile*                              mapToTile(const float p[2]);
    Tile*                              inTile(const float p[2]);
    int                                getX();
    int                                getZ();
  private:
    std::vector<std::vector<Tile>>     tiles;
    int                                sideLength;
};

enum NodeStatus
{
  Open = 0,
  Closed = 1,
  Unvisited = 2
};

float heuristic(Tile* here, Tile* goal);
float moveCost(Tile* t1, Tile* t2);

class Node {
  public:
                                       Node(Tile* cur, Tile* prev, float cost, float estimate, NodeStatus stat);
                                       Node(); // make a Null node

    static bool                        betterNode(Node testNode, Node compNode);
    static std::vector<Tile>           aStar(Grid* graph, Tile* start, Tile* end);
  private:
    Tile*                              t;
    Tile*                              previous;
    float                              costSoFar;
    float                              estamatedTotalCost;
    NodeStatus                         status;
};

struct CompareNode {
  bool operator()(const Node &lhs, const Node &rhs) const {
    return Node::betterNode(rhs, lhs);
  }
};
#endif // TILE_H
