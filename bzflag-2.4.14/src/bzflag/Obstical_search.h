//
//  Obstical_search.hpp
//  BZFlag
//
//  Created by aidan akamine on 10/7/18.
//  Copyright © 2018 BZFlag. All rights reserved.
//

#ifndef Obstical_search_H
#define Obstical_search_H

#include <stdio.h>
#include "World.h"
#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <stdio.h>
#include "BZDBCache.h"
#include <array>
#include <vector>



class Obstical_search
{
  class node
  {
  public:
    // current position
    int xPos;
    int yPos;
    // total distance already travelled to reach the node
    int level;
    // priority=level+remaining distance estimate
    int priority;  // smaller: higher priority
    int dir;
    node * parent;
    node(int xp, int yp, int d, int p)
    {
      xPos=xp;
      yPos=yp;
      level=d;
      priority=p;
      parent = NULL;
      dir = 8;
      
    }
    ~node()
    {
      
    }
    
    int getxPos() const {return xPos;}
    int getyPos() const {return yPos;}
    int getLevel() const {return level;}
    int getPriority() const {return priority;}
    void setxPos(int x) {this->xPos = x;}
    void setyPos(int y) {this->yPos = y;}
    void setPriority(int priority) {this->priority = priority;}
    
    
    void updatePriority(const int & xDest, const int & yDest)
    {
      //priority = level+ estimate(xDest, yDest)*10; //A*
      setPriority(getLevel() + (10 * estimate(xDest, yDest) ));
      
    }
    
    void nextLevel(const int & i) // i: direction
    {
      level+=(dir==8?(i%2==0?10:14):10);
    }
    
    // Estimation function for the remaining distance to the goal.
    int estimate(const int & xDest, const int & yDest)
    {
      int xd, yd, d;
      xd=xDest-getxPos();
      yd=yDest-getyPos();

      d=static_cast<int>(sqrt(xd*xd+yd*yd));
      
      return(d);
    }
  };
  
  
public:
  
  
  int **map;
  int **closed_nodes_map; // map of closed (tried-out) nodes
  int **open_nodes_map; // map of open (not-yet-tried) nodes
  int **dir_map; // map of directions
  int xdim;// horizontal size of the map
  int ydim;// vertical size size of the map
  int dx[8];
  int dy[8];
  std::vector<std::array<int, 3>>  path;
  
  static const int dir = 8;
  
  Obstical_search(int xdim, int ydim){
    this->xdim = 2 *xdim;
    this->ydim = 2 *ydim;
    dx[0] = 1; dx[1] = 1; dx[2] = 0; dx[3] = -1; dx[4] = -1; dx[5] =-1; dx[6] = 0; dx[7] = 1;
    dy[0] = 0; dy[1] = 1; dy[2] = 1; dy[3] = 1; dy[4] = 0; dy[5] =-1; dy[6] = -1; dy[7] = -1;
    this->map = new int*[this->xdim];
    this->closed_nodes_map = new int*[this->xdim];
    this->open_nodes_map = new int*[this->xdim];
    this->dir_map = new int*[this->xdim];
    
    for(int i = 0; i < this->xdim; i++){
      this->map[i] = new int[this->ydim];
      this->closed_nodes_map[i] = new int[this->ydim];
      this->open_nodes_map[i] = new int[this->ydim];
      this->dir_map[i] = new int[this->ydim];
    }
  }
  
  friend bool operator<(const node & a, const node & b)
  {
    return a.getPriority() > b.getPriority();
  }
  
  void addObstical (int x, int y) {
    map[x][y] = 1;
  }
  
  void initialize_map() {
    
    for (int i =0; i<xdim-1; i++){
      for(int j =0; j<ydim-1; j++){
	map[i][j] = 0;
      }
    }
  }

  void read_path(std::string route, int startx, int starty)
  {
    if(route.length() > 0 ) {
      int j; char c;
      int x = startx;
      int y = starty;
      
      for(int i=0;i<route.length();i++)
      {
	
	//note * i requested branden mccabe's assistance on how to use the vectors and arrays
	std::array<int, 3>temp;
	c =route.at(i);
	j=atoi(&c);
	x=x+dx[j];
	y=y+dy[j];
	temp[0] = x;
	temp[1] = y;
	temp[2] = 0;
	path.push_back(temp);
	//path.insert(temp);
      }
    }
  }
  
  
  std::string find_path(int & xStart, int & yStart,
				int & xFinish, int & yFinish)
  {
    std::priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
    int pqi; // pq index
    node* n0;
    node* m0;
    int i, j, x, y, xdx, ydy;
    char c;
    pqi=0;
    
    // reset the node maps
    for(y=0;y<xdim;y++)
    {
      for(x=0;x<ydim;x++)
      {
	closed_nodes_map[x][y]=0;
	open_nodes_map[x][y]=0;
      }
    }
    
    // create the start node and push into list of open nodes
    n0=new node(xStart, yStart, 0, 0);
    n0->updatePriority(xFinish, yFinish);
    pq[pqi].push(*n0);
    open_nodes_map[xStart][yStart]=n0->getPriority(); // mark it on the open nodes map
    
    // A* search
    while(!pq[pqi].empty())
    {
      // get the current node w/ the highest priority
      // from the list of open nodes
      n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
		  pq[pqi].top().getLevel(), pq[pqi].top().getPriority());
      
      x=n0->getxPos(); y=n0->getyPos();
      
      pq[pqi].pop(); // remove the node from the open list
      open_nodes_map[x][y]=0;
      // mark it on the closed nodes map
      closed_nodes_map[x][y]=1;
      
      // quit searching when the goal state is reached
      //if((*n0).estimate(xFinish, yFinish) == 0)
      if(x==xFinish && y==yFinish)
      {
	// generate the path from finish to start
	// by following the directions
	std::string path="";
	while(!(x==xStart && y==yStart))
	{
	  j=dir_map[x][y];
	  c='0'+(j+dir/2)%dir;
	  path=c+path;
	  x+=dx[j];
	  y+=dy[j];
	}
	
	// garbage collection
	delete n0;
	// empty the leftover nodes
	while(!pq[pqi].empty()) pq[pqi].pop();

	return path;
      }
      
      // generate moves (child nodes) in all possible directions
      for(i=0;i<dir;i++)
      {
	xdx=x+dx[i]; ydy=y+dy[i];
	
	if(!(xdx<0 || xdx>xdim-1 || ydy<0 || ydy>ydim-1 || map[xdx][ydy]==1
	     || closed_nodes_map[xdx][ydy]==1))
	{
	  // generate a child node
	  m0=new node( xdx, ydy, n0->getLevel(),
		      n0->getPriority());
	  m0->nextLevel(i);
	  m0->updatePriority(xFinish, yFinish);
	  
	  // if it is not in the open list then add into that
	  if(open_nodes_map[xdx][ydy]==0)
	  {
	    open_nodes_map[xdx][ydy]=m0->getPriority();
	    pq[pqi].push(*m0);
	    // mark its parent node direction
	    dir_map[xdx][ydy]=(i+dir/2)%dir;
	  }
	  else if(open_nodes_map[xdx][ydy]>m0->getPriority())
	  {
	    // update the priority info
	    open_nodes_map[xdx][ydy]=m0->getPriority();
	    // update the parent direction info
	    dir_map[xdx][ydy]=(i+dir/2)%dir;
	    
	    // replace the node
	    // by emptying one pq to the other one
	    // except the node to be replaced will be ignored
	    // and the new node will be pushed in instead
	    while(!(pq[pqi].top().getxPos()==xdx &&
		    pq[pqi].top().getyPos()==ydy))
	    {
	      pq[1-pqi].push(pq[pqi].top());
	      pq[pqi].pop();
	    }
	    pq[pqi].pop(); // remove the wanted node
	    
	    // empty the larger size pq to the smaller one
	    if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
	    while(!pq[pqi].empty())
	    {
	      pq[1-pqi].push(pq[pqi].top());
	      pq[pqi].pop();
	    }
	    pqi=1-pqi;
	    pq[pqi].push(*m0); // add the better node instead
	  }
	  else delete m0; // garbage collection
	}
      }
      delete n0; // garbage collection
    }
    return ""; // no route found
  }
};

#endif

