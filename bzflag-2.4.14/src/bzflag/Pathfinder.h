#define worldScalar BZDBCache::tankRadius

class myNode
{
public:
  int x, y; // Profiling observation: integer coordinates, hence operator==,
            //  makes the search significantly faster (almost 10 folds than double)
  bool operator==(const myNode& n) { return (x == n.x && y == n.y); }; // This must be defined for the node
};

void worldToGraph(std::vector<int> ranges);
void graphToWorld(myNode& n, float coords[3]);

std::vector< std::vector< myNode > > pathfinder(const float x, const float y, const float z, const float* myPos);
