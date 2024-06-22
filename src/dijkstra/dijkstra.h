#ifndef dijkstra_h
#define dijkstra_h

#include "base_algorithm.h"
#include <unordered_map>
#include <queue>
#include <stack>
#include "renderbase.h"
#include <astar.h>




class Dijkstra : public BaseAlgorithm 
{
public:

    Dijkstra(){
        Initialize();
    };
    ~Dijkstra(){}


	void Initialize(void) override
    {   

    }
    //TODO
	void UpdateOneStep(void) override
    {  
        reset();
        startDijkstra();
        if(find){
            shortestPath.clear();
            findPath();
            std::cout<<"minimum cost to the goal: "<<distanceMap[getIndex(goal.x,goal.y)]<<std::endl;
        }
        else{
            std::cout << "No path found." << std::endl;
        }
    }

    //TODO
	void Render(void) override
    { 
        RenderBaseGrid(); // This function render the base grid line and start goal grid
        renderDistanceMap();
        renderPath();
    }
    bool IsTerminated(void) override 
    {
        return find;
    }

    bool ReturnThepath(void) override{
        return true;
    }
    ;

protected:
    const int moveCost = 1;
	// direction arrays 
    int dx[4] = { -1, 0, 1, 0 };
    int dy[4] = { 0, 1, 0, -1 };
    bool find = false;

    GridLocation start;
    GridLocation goal;

    // hashMap to store distances for each grid cell
    std::unordered_map<int, int> distanceMap;
    // hashMap to store ancestor for every location been visited,
   	std::unordered_map<GridLocation, GridLocation> came_from;
    std::vector<std::pair<int, int>> shortestPath;
    
    void reset(){
        find = false;
        std::vector<int> endPoint;
        std::vector<int> startPoint;
        startPoint=map->GetStartPoint();
        endPoint = map->GetEndPoint();
        start.x = startPoint[0];
        start.y = startPoint[1];
        goal.x = endPoint[0];
        goal.y = endPoint[1];
        std::cout<<"start:"<<start.x<<" , "<<start.y<<std::endl;
        std::cout<<"goal:"<<goal.x<<" , "<<goal.y<<std::endl;
        distanceMap.clear();
        came_from.clear();
        shortestPath.clear();
    }

    void renderPath(){
        if(shortestPath.size() > 0){
            for(int i = 0; i < shortestPath.size()-1; ++i){
            int cx1 = shortestPath[i].first * GRIDWIDTH + GRIDWIDTH / 2;
			int cy1 = shortestPath[i].second * GRIDHEIGHT + GRIDHEIGHT / 2;
            int cx2 = shortestPath[i+1].first * GRIDWIDTH + GRIDWIDTH / 2;
			int cy2 = shortestPath[i+1].second * GRIDHEIGHT + GRIDHEIGHT / 2;
            RenderBase::DrawLine(cy1,cx1,cy2,cx2,3.0,1.0,1.0,1.0);
            }
        }
    }

    void renderDistanceMap(){
        glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glBegin(GL_QUADS);
        int minDistance = INT_MAX;
        int maxDistance = INT_MIN;
        
        for (const auto& pair : distanceMap) {
            minDistance = std::min(minDistance, pair.second);
            maxDistance = std::max(maxDistance, pair.second);
        }
        float r = (0.68 - 0)/(maxDistance-minDistance);
        float g = (1.0 - 0.84)/(maxDistance-minDistance);
        float b = (0.84 - 0.04)/(maxDistance-minDistance);
        for (const auto& pair : distanceMap) {
            int distance = pair.second;
            if(distance<maxDistance && distance>minDistance){
                float red = (distance - minDistance)*r;
                float green = 0.67 + (maxDistance - distance)*g;
                float blue = 0.63 + (maxDistance - distance)*b;
                glColor3f(red, green, blue);  
                GridLocation curLocation = getGridFromIndex(pair.first);
                int cx = curLocation.x * GRIDWIDTH + GRIDWIDTH / 2;
                int cy = curLocation.y * GRIDHEIGHT + GRIDHEIGHT / 2;
                RenderBase::DrawSquare(cy,cx,GRIDWIDTH*0.9, GRIDHEIGHT*0.92);
            }

        }
        glEnd();
		glDisable(GL_BLEND);
        
    }
    //boundary check
	bool isInsideMap(int x, int y) const
    {       
        return (x >= 0 && x < map->GetWidth() && y >= 0 && y < map->GetHeight());
	}

	//collision check
	bool checkCollision(int x, int y) const
	{
		if(map->Get(y, x).state == 1){
			return true;
		}
		return false;
	}

    //helper function get index of map
	inline int getIndex(int x, int y) const
    {
		return map->GetWidth()*y + x;
	}
    
    GridLocation getGridFromIndex(int index) const{
        GridLocation loc;
        loc.x = index%map->GetWidth();
        loc.y = (index-loc.x)/map->GetWidth();
        return loc;
    }

    //Manhattan Distance
    virtual int heuristic(GridLocation a, GridLocation b) 
    {
        return 0;
    }

    void startDijkstra()
    {
        PriorityQueue<GridLocation, int> pq;
        pq.push(start, 0);
        came_from[start] = start;

        while(!pq.empty()){
            GridLocation curLocation = pq.top();

            if(curLocation == goal)
            {
                find = true;
                break;
            }
            
            for(int i = 0; i < 4; ++i){
                int nx = curLocation.x + dx[i];
                int ny = curLocation.y + dy[i];
                if(isInsideMap(nx, ny)){
                    if(!checkCollision(nx, ny)){
                        GridLocation newLocation(nx,ny);

                        int id = getIndex(newLocation.x,newLocation.y);
                        int newCost = distanceMap[getIndex(curLocation.x, curLocation.y)]+moveCost;

                        if(distanceMap.find(id) == distanceMap.end() ||
                            newCost < distanceMap[id])
                        {
                            distanceMap[id] = newCost;
                            pq.push(newLocation, newCost+heuristic(newLocation, goal));
                            came_from[newLocation] = curLocation;
                        }
                    }
                }
            }
        }

    }
    void findPath(){
        if (!find) {
            std::cout << "No path found." << std::endl;
        }
        else{
            std::cout << "path found." << std::endl;
            GridLocation current = goal;
            while (current != start) {
                shortestPath.push_back(std::make_pair(current.x, current.y));
                current = came_from[current];
            }
        }
    }

};


#endif /* dijkstra_h */