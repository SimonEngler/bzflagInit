#pragma once
class myNode
{
public:
          int x, y; // Profiling observation: integer coordinates,
                    //  hence operator==, makes the search significantly 
                    //  faster (almost 10 folds than double)
          //float pos[2];

          //myNode();
          bool operator==(const myNode& n); // defines if two nodes are identical
          float* getPos();
};

