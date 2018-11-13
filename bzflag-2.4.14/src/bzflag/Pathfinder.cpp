/******************************************************************************************
*                                                                                        *
*    Part of                                                                             *
*    Yet Another Graph-Search Based Planning Library (YAGSBPL)                           *
*    A template-based C++ library for graph search and planning                          *
*    Version 1.0                                                                         *
*    ----------------------------------------------------------                          *
*    Copyright (C) 2010  Subhrajit Bhattacharya                                          *
*                                                                                        *
*    This program is free software: you can redistribute it and/or modify                *
*    it under the terms of the GNU General Public License as published by                *
*    the Free Software Foundation, either version 3 of the License, or                   *
*    (at your option) any later version.                                                 *
*                                                                                        *
*    This program is distributed in the hope that it will be useful,                     *
*    but WITHOUT ANY WARRANTY; without even the implied warranty of                      *
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                       *
*    GNU General Public License for more details <http://www.gnu.org/licenses/>.         *  
*                                                                                        *
*                                                                                        *
*    Contact: subhrajit@gmail.com, http://fling.seas.upenn.edu/~subhrabh/                *
*                                                                                        *
*                                                                                        *
******************************************************************************************/
//    For a detailed tutorial and download, visit 
//    http://fling.seas.upenn.edu/~subhrabh/cgi-bin/wiki/index.php?n=Projects.ProgrammingLibraries-YAGSBPL


#include <stdio.h>
// =======================
// YAGSBPL libraries
#include "yagsbpl_base.h"
#include "planners/A_star.h"

//BZ Flag Includes
#include "BZDBCache.h"
#include "World.h"
#include "playing.h"

#include "pathfinder.h"

// ============================================================
// Functions that describe the graph, placed inside a class
// Here the class "GraphFunctionContainer" is redefining the virtual functions declared in "SearchGraphDescriptorFunctionContainer"
// Thus the names of the functions are important here.

class GraphFunctionContainer : public SearchGraphDescriptorFunctionContainer<myNode,double>
{
public:
        
	int getHashBin(myNode& n) // Use the absolute value of x coordinate as hash bin counter. Not a good choice though!
	{
		return ((int)fabs(n.x));
	}

	bool isAccessible(myNode& n)
	{
          char buffer[128];
          //controlPanel->addMessage("Is accessible being called...");

          float targetLoc[3];                             // Initialize container for coordinates of a targets location.
          graphToWorld(n, targetLoc);                     // Take the graph coordinates, translate them to world coordinates and then put them in the container.

          // If the node being inspected is in a building or outside the boundaries of the world return false because it's not accessible.

          // If the inBuilding function finds that the target is in a building it returns an obstacle so just check if it exists.
          if (World::getWorld()->inBuilding(targetLoc, (BZDBCache::tankRadius / 2), BZDBCache::tankHeight)) {                     // Still not 100% on how this function works. It returns an obstacle
            sprintf(buffer, "ERROR: Obstacle found. The node is NOT okay to traverse to.");                                       // object if it finds out if the target is in a building I think.
            //controlPanel->addMessage(buffer);
            return false;
          }
          // Check if the point is outside of the world.
          else if (targetLoc[0] > BZDBCache::worldSize || targetLoc[1] > BZDBCache::worldSize) {
            return false;
          }
          else {
            sprintf(buffer, "No obstacle, the node is okay to traverse to.");
            //controlPanel->addMessage(buffer);
            return true;
          }
	}

	void getSuccessors(myNode& n, std::vector<myNode>* s, std::vector<double>* c) // Define a 8-connected graph
	{
		// This function needn't account for obstacles or size of environment. That's done by "isAccessible"
		myNode tn;                                                                                                  
		s->clear(); c->clear(); // Planner is supposed to clear these. Still, for safety we clear it again.
		for (int a=-1; a<=1; a++)                                                                                   
			for (int b=-1; b<=1; b++) {
				if (a==0 && b==0) continue;
				tn.x = n.x + a;
				tn.y = n.y + b;
				s->push_back(tn);
				c->push_back(sqrt((double)(a*a+b*b)));
			}
	}

	double getHeuristics(myNode& n1, myNode& n2)
	{
		int dx = abs(n1.x - n2.x);
		int dy = abs(n1.y - n2.y);
		return (sqrt((double)(dx*dx + dy*dy))); // Euclidean distance as heuristics
	}

	// -------------------------------
	// constructors
	GraphFunctionContainer (int cr, std::vector<int> ranges) 
		{ circleRadius = cr; Xmin = ranges[0]; Xmax = ranges[1]; Ymin = ranges[2]; Ymax = ranges[3]; }
	GraphFunctionContainer () 
		{ circleRadius = 1; Xmin = -2; Xmax = 2; Ymin = -2; Ymax = 2; }

private:

	int circleRadius;
	int Xmin, Xmax, Ymin, Ymax;

};


/*
    This function defines a way for us to translate the real in game world size into a more manageable
    graph size to be used for A* search pathfinding.

    Arguments: It takes in a vector container that will hold the max ranges of the world.
    Returns:   Void and a vector array containing the ranges of the graph.
                Indexes: 0 = -X, 1 = X , 2 = -Y, 3 = Y 
*/
void worldToGraph(std::vector<int> ranges) {

  const float worldSize = BZDBCache::worldSize;       // Store the current world size in a temporary variable. This can be made more efficient by doing this inline with the scaling.
  int graphSize = (int) worldSize / worldScalar;      // Translate the world size to graph size. worldScalar is defined in Pathfinder.h

  ranges[0] = -1 * graphSize;                         // Translate the distance range to cartesian coordinates  
  ranges[1] = graphSize;                              // and store them into the ranges vector
  ranges[2] = -1 * graphSize;
  ranges[3] = graphSize;
}

/*
    This function defines a way for us to translate the graph that was made into game coordinates.

    Arguments: Takes in a float array container for us to put all the coordinates in.
    Returns:   Void and the coords container with the translated points.
*/
void graphToWorld(myNode& n, float coords[3]) {

  coords[0] = n.x * worldScalar;
  coords[1] = n.y * worldScalar;
  coords[2] = 0;
}

// =============================================================================
std::vector< std::vector< myNode > >  pathfinder(const float x, const float y, const float z, const float* myPos)
{
        char buffer[128];
        //controlPanel->addMessage("A_Star_Testing has begun...");

	// Profiling observation: Using int instead of double cost provides marginal improvement (~10%)
	GenericSearchGraphDescriptor<myNode,double> myGraph;
	
	// We describe the graph, cost function, heuristics, and the start & goal in this block
	// ------------------------------------------------------------------------------------

	// Create an instance of "GraphFunctionContainer_derived", and set is as the function container.
	std::vector<int> ranges(4);                      // Vector array that holds the world ranges
        worldToGraph(ranges);                            // Define the ranges by using the actual world and translating it into a grid based graph

	GraphFunctionContainer fun_cont(1, ranges);
	myGraph.func_container = &fun_cont;
	// Set other variables
	myGraph.hashTableSize = 212; // Since in this problem, "getHashBin" can return a max of value 201.
	myGraph.hashBinSizeIncreaseStep = 128; // By default it's 128. For this problem, we choose a higher value.
	
	myNode tempNode;
        // Round to nearest integer.
        tempNode.x = (int)round(myPos[0] / worldScalar); tempNode.y = (int)round(myPos[1] / worldScalar); // Start node
        myGraph.SeedNode = tempNode;

        // Round to nearest integer.
        tempNode.x = (int)round(x / worldScalar); tempNode.y = (int)round(y / worldScalar); // Goal node
	myGraph.TargetNode = tempNode;

	// ------------------------------------------------------------------------------------
	
	// Planning
	A_star_planner<myNode,double>  planner;
	planner.setParams(1.0, 10); // optional.
	planner.init(myGraph);
	planner.plan();

        std::vector< std::vector< myNode > > paths = planner.getPlannedPaths();

        /*-------------------------------------------------------------------------------------
         * The following block is purely for debugging and is non-essential.
         */

        /*
        sprintf(buffer, "My tank's position in the real world: x: %f, y: %f", myPos[0], myPos[1]);
        controlPanel->addMessage(buffer);
        sprintf(buffer, "My trget's position in the real world: x: %f, y: %f", x, y);
        controlPanel->addMessage(buffer);
        sprintf(buffer, "My trget's position in the graph: x: %f, y: %f", ( x / worldScalar ), ( y / worldScalar ));
        controlPanel->addMessage(buffer);
        sprintf(buffer, "My tank's position in the real world: x: %f, y: %f", myPos[0], myPos[1]);
        controlPanel->addMessage(buffer);
        sprintf(buffer, "My tank's position in the graph: x: %f, y: %f", ( myPos[0] / worldScalar ), ( myPos[1] / worldScalar ));
        controlPanel->addMessage(buffer);


	
	sprintf(buffer, "\nNumber of connections: %d\nPath coordinates: \n[ ", paths.size());
        controlPanel->addMessage(buffer);

	for (int a=0; a<paths[0].size(); a++){
                sprintf(buffer, "[%d, %d]; ", paths[0][a].x, paths[0][a].y);
                controlPanel->addMessage(buffer);
        }
	sprintf(buffer, " ]\n\n");
        controlPanel->addMessage(buffer);
        ---------------------------------------------------------------------------------------*/

        return paths;
}

