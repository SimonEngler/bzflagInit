#pragma once

#ifndef	BZF_ASTARNODE_H
#define	BZF_ASTARNODE_H

#include <vector>
using std::vector;
#include "BZDBCache.h"

#define SCALE	BZDBCache::tankRadius
// A node of the A* search graph

// status is unseen (not yet visited), on the open list or on the closed list
enum AStarStatus {unseen, open1, closed};

// Stores a node in the A* search graph
class AStarNode
{
public:
	AStarNode(void);
	~AStarNode(void);

	bool operator==(const AStarNode& n) const { return (x==n.x && y==n.y); };
	bool operator<(const AStarNode& n) const { return (getTotalCost() < n.getTotalCost()); };
	AStarNode(const float location[3]);
	AStarNode(int xi, int yi);
	bool isAccessible(void);
	static bool isAccessible(int x, int y);
	double getHeuristic(const AStarNode * goalNode) const;
	inline int getX(void) const { return x; }
	inline int getY(void) const { return y; }
	inline void setX(int newX) { x = newX; }
	inline void setY(int newY) { y = newY; }
	inline int getPrevX(void) const { return prevX; }
	inline int getPrevY(void) const { return prevY; }
	inline void setPrevX(int newX) { prevX = newX; }
	inline void setPrevY(int newY) { prevY = newY; }
	inline float getScaledX(void) const { return x * SCALE; }
	inline float getScaledY(void) const { return y * SCALE; }
	inline double getCostSoFar(void) const { return costSoFar; }
	inline void setCostSoFar(double newCostSoFar) { costSoFar = newCostSoFar; }
	inline double getTotalCost(void) const { return totalCost; }
	inline void setTotalCost(double newTotalCost) { totalCost = newTotalCost; }
	inline AStarStatus getStatus(void) const { return status; }
	inline void setStatus(AStarStatus newStatus) { status = newStatus; }
private:
	int x, y;
	int prevX, prevY;
	double costSoFar;
	double totalCost;
	AStarStatus status;
};


#endif // BZF_ASTARNODE_H
