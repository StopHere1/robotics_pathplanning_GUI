#ifndef rrt_h
#define rrt_h

#include "base_algorithm.h"
#include <list>
#include <iterator>
#include <algorithm>
#include <vector>
#include <set>
#include <random>
constexpr int MaxIterations = 10000;
constexpr int MaxDist = 10;
class Node
{
public:
    Node() : x(0), y(0), prev(nullptr), next(nullptr) {}
    Node(int ix, int iy) : x(ix), y(iy), prev(nullptr), next(nullptr) {}
    Node(int ix, int iy, std::shared_ptr<Node> iprev) : x(ix), y(iy), prev(iprev), next(nullptr) {}
    Node(int ix, int iy, std::shared_ptr<Node> iprev, std::shared_ptr<Node> inext) : x(ix), y(iy), prev(iprev), next(inext) {}
    int x;
    int y;
    std::shared_ptr<Node> prev;
    std::shared_ptr<Node> next;
};
class RRT : public BaseAlgorithm
{
public:
    RRT()
	{
		Initialize();
	}
    ~RRT(){}

	void Initialize(void) override
    {   
        

    }

    bool curMapIsSameAsNewMap()
    {
        if(built)
            return true;
        if(curMap.GetHeight() != map->GetHeight() || curMap.GetWidth() != map->GetWidth())
        {
            curMap.CopyFrom(*map);
            built = true;
            return false;
        }
        else
        {
            for (int j = 0; j < map->GetWidth(); j++)
            {
                for(int i = 0 ; i < map->GetHeight(); i++)
                {
                    if(curMap.Get(j, i).state != map->Get(j, i).state)
                        return false;
                }
            }
        }
        return true;
    }
	void UpdateOneStep(void) override
    {
        if(!curMapIsSameAsNewMap())
        {
            curX = map->GetStartPoint()[1];
            curY = map->GetStartPoint()[0];
            start = std::make_shared<Node>(curX, curY);
            rrtNodes.push_back(start);
            goal = std::make_shared<Node>(map->GetEndPoint()[1], map->GetEndPoint()[0]);
            lookForPath();
        }
    }
    bool ReturnThepath(void) override
    {
        return false;
    }
	void Render(void) override
    {
        RenderBaseGrid(); // This function render the base grid line and start goal grid
        int curX = map->GetStartPoint()[1];
        int curY = map->GetStartPoint()[0];
        int curTicks = 0;
        ticks++;
        for (auto &node : movement)
        {
            if(curTicks == ticks)
                break;
            curTicks++;

            auto x = std::get<0>(node);
            auto y = std::get<1>(node);
            glLineWidth(5);
            RenderBase::DrawLine((curX*GRIDWIDTH)+(GRIDWIDTH/2), (curY*GRIDHEIGHT)+(GRIDHEIGHT/2), (x*GRIDWIDTH)+(GRIDWIDTH/2), (y*GRIDHEIGHT)+(GRIDHEIGHT/2), 5, 0, 0, 0);
            glLineWidth(1);
            curX = x;
            curY = y;
        }
    }
    bool IsTerminated(void) override 
    {
        return false;
    }
private:
    std::vector<std::tuple<int, int>> getPointsBetweenNodes(int x1, int y1, int x2, int y2)
    {
        std::vector<std::tuple<int, int>> points;
        
        bool steep = abs(y2 - y1) > abs(x2 - x1);
        if (steep) {
            std::swap(x1, y1);
            std::swap(x2, y2);
        }
        if (x1 > x2) {
            std::swap(x1, x2);
            std::swap(y1, y2);
        }

        int dx = abs(x2 - x1);
        int dy = abs(y2 - y1);
        int ystep = (y1 < y2) ? 1 : -1;
        int error = dx / 2;
        int y = y1;

        for (int x = x1; x <= x2; x++) {
            if (steep) {
                points.push_back(std::make_tuple(y, x));
            } else {
                points.push_back(std::make_tuple(x, y));
            }
            error -= dy;
            if (error < 0) {
                y += ystep;
                error += dx;
            }
        }

        return points;
    }

    bool checkObstacles(int x1, int y1, int x2, int y2)
    {
        for (auto& node : getPointsBetweenNodes(x1, y1, x2, y2))
        {
            if (map->Get(std::get<0>(node), std::get<1>(node)).state == 1)
            {
                return true;
            }
        }
        return false;
    }
    std::shared_ptr<Node> checkNearestNode(std::shared_ptr<Node> new_node)
    {

        std::vector<double> tempX;
        std::vector<double> tempY;

        auto near_node = std::make_shared<Node>();
        double minDistance = std::numeric_limits<double>::max();
        int corrX = 0;
        int corrY = 0;
        bool check_obstacle;

        for (auto &ii : rrtNodes)
        {

            double distance = std::sqrt(std::pow((new_node->x - ii->x), 2) + std::pow((new_node->y - ii->y), 2));
            if (distance < minDistance)
            {

                minDistance = distance;
                near_node = ii;
            }
        }

        double dx = new_node->x - near_node->x;
        double dy = new_node->y - near_node->y;
        double angle = std::atan2(dy, dx) * 180 / YsPi;

        if (minDistance > MaxDist)
        {

            corrX =  std::min(NXGRID, std::max(static_cast<int>(std::abs(near_node->x + std::cos(angle) * MaxDist)), 0));
            corrY =  std::min(NYGRID, std::max(static_cast<int>(std::abs(near_node->y + std::sin(angle) * MaxDist)), 0));
        }

        if (minDistance <= MaxDist)
        {

            corrX = new_node->x;
            corrY = new_node->y;
        }
        if (rrtNodes.size() > 0)
        {
            check_obstacle = checkObstacles(near_node->x, near_node->y, corrX, corrY);
        }

        new_node->x = corrX;
        new_node->y = corrY;

        near_node->next = new_node;
        new_node->prev = near_node;

        if (rrtNodes.size() == 0)
        {

            new_node->prev = start;
        }

        if (check_obstacle == 0)
        {
            rrtNodes.push_back(new_node);
        }

        if (((double)new_node->x == (double)this->goal->x) && ((double)new_node->y == (double)this->goal->y))
        {


            found = true;
            std::vector<std::tuple<int, int>> nav;
            nav.push_back(std::make_tuple(new_node->x, new_node->y));
            while (new_node->prev != nullptr)

            {
                std::cout << new_node->x << " :?: " << new_node->y << std::endl;
                new_node = new_node->prev;
                nav.push_back(std::make_tuple(new_node->x, new_node->y));
            }
            std::reverse(nav.begin(), nav.end());
            auto fromNode = nav[0];
            for(unsigned int i = 1; i < nav.size(); i++)
            {
                auto toNode = nav[i];
                auto points = getPointsBetweenNodes(std::get<0>(fromNode), std::get<1>(fromNode), std::get<0>(toNode), std::get<1>(toNode));
                if(std::get<0>(points[0]) == std::get<0>(toNode) && std::get<1>(points[0]) == std::get<1>(toNode))
                {
                    std::reverse(points.begin(), points.end());
                }
                auto first = points[0];
                for (auto &p : points)
                {
                    auto pX = std::get<0>(p);
                    auto pY = std::get<1>(p);
                    if(pX >= NXGRID || pX < 0 || pY >= NYGRID || pY < 0)
                        continue;
                    if(std::get<0>(first) == std::get<0>(p) && std::get<1>(first) == std::get<1>(p))
                    {
                        continue;
                    }
                    movement.push_back(p);
                }
                fromNode = toNode;
            }
        }

        return new_node;
    }
    void lookForPath()
    {

        std::random_device dev;
        std::mt19937 rng(dev());
        std::uniform_int_distribution<int> distY(0, Parameter::NYGRID);
        std::uniform_int_distribution<int> distX(0, Parameter::NXGRID);
        std::cout << "Randomly generated nodes: ";
        for (double i = 0; i < MaxIterations; i++)
        {
            auto random_node = std::make_shared<Node>(distX(rng), distY(rng));
            printf("(%d, %d) ", random_node->x, random_node->y);
            auto last_node = checkNearestNode(random_node);

            if (found)
            {
                goal->prev = last_node;
                break;
            }
        }
    }
    Space2D curMap;
    int curX;
    int curY;
    bool built = false;
    std::vector<std::tuple<int, int>> movement;
    unsigned int offset = 0;
    int ticks = 0;
    std::shared_ptr<Node> start = nullptr;
    std::shared_ptr<Node> goal = nullptr;
    bool found = false;
    std::vector<std::shared_ptr<Node>> rrtNodes;
};

#endif /* rrt_h */