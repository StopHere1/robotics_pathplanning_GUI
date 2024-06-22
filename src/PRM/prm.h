#ifndef prm_h
#define prm_h

#include "base_algorithm.h"
#include <list>
#include <iterator>
#include <algorithm>
#include <vector>
#include <set>
#include <random>
constexpr float INF = std::numeric_limits<float>::max();
constexpr int NODES = 100;
constexpr int TARGET_ID = NODES+1;
constexpr float RADIUS = 100.0f;

struct PRMNode
{
    PRMNode() : x(0), y(0), id(0) {}
    PRMNode(int ix, int iy, int iid) : x(ix), y(iy), id(iid) {}

    int x;
    int y;
    int id;
};

class Graph
{

private:
    int V;

    std::list<std::pair<int, int>> *adj;

public:

    std::vector<int> optimalNodes;
    Graph(int v) : V(v)
    {
        this->V = v;
        this->adj = new std::list<std::pair<int, int>>[this->V];
    }
    void addEdge(int vStart, int vEnd, int cost)
    {
        this->adj[vStart].push_back(std::make_pair(cost, vEnd));
    }
    void Astar(int vStart, int vGoal, std::vector<bool> visited, std::vector<float> &heuristic)
    {
        std::vector<std::tuple<int, int, int>> path;
        std::vector<float> functionFX(V, INF);

        std::set<std::pair<float, int>> AStar_set;

        functionFX[vStart] = 0 + heuristic[vStart];

        AStar_set.insert(std::make_pair(functionFX[vStart], vStart));

        while ((*(AStar_set.begin())).second != vGoal)
        {

            std::pair<float, int> nodeMin = *(AStar_set.begin());

            int nodeGraph = nodeMin.second;

            AStar_set.erase(AStar_set.begin());

            visited[nodeGraph] = true;

            for (auto i = adj[nodeMin.second].begin(); i != adj[nodeMin.second].end(); i++)
            {

                int nodeGraph_i = (*i).second;
                int nodeGraph_i_functionFX = (*i).first + heuristic[(*i).second];
                if (visited[nodeGraph_i] != true)
                {

                    if (functionFX[nodeGraph_i] > nodeGraph_i_functionFX)
                    {
                        if (functionFX[nodeGraph_i] != INF)
                        {
                            AStar_set.erase(AStar_set.find(std::make_pair(functionFX[nodeGraph_i], nodeGraph_i)));
                        }

                        functionFX[nodeGraph_i] = nodeGraph_i_functionFX;
                        AStar_set.insert(std::make_pair(functionFX[nodeGraph_i], nodeGraph_i));

                        path.push_back(std::make_tuple(functionFX[nodeGraph_i], nodeGraph_i, nodeGraph));
                    }
                }
            }
        }


        std::multiset<std::tuple<int, int>> init_mSet;
        init_mSet.insert(std::make_tuple(0, 0));
        std::vector<std::multiset<std::tuple<int, int>>> routePath(V, init_mSet);

        for (int pathV = 1; pathV < V; pathV++)
        {

            std::multiset<std::tuple<int, int>> to_routePath;

            for (auto &ii : path)
            {

                int vertexV = std::get<1>(ii);

                if (pathV == vertexV)
                {

                    to_routePath.insert(std::make_tuple(std::get<0>(ii), std::get<2>(ii)));
                }
            }

            routePath[pathV] = to_routePath;
        }


        int previous = vGoal;
        std::vector<int> optimalPath;
        optimalPath.push_back(vGoal);

        while (previous != 0)
        {

            std::set<std::tuple<int, int>> minFx;

            for (auto &ii : routePath[previous])
            {

                minFx.insert(std::make_tuple(std::get<0>(ii), std::get<1>(ii)));
            }

            auto it = minFx.begin();
            previous = std::get<1>(*it);

            int min_path_i = std::get<0>(*it);

            optimalPath.push_back(previous);
        }

        optimalNodes = optimalPath;
    }
    void computeStarA(int vStart, int vGoal, std::vector<float> heuristic)
    {
        std::vector<bool> visited(this->V, false);
        Graph::Astar(vStart, vGoal, visited, heuristic);
    }
};


class PRM : public BaseAlgorithm
{
public:
    bool built = false;
    PRM()
	{
		Initialize();
	}
    
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
            auto nodes = generateNodes();
            std::cout << "Randomly generated nodes: ";
            for(int i = 0; i < 20; i++)
            {
                std::cout << "(" << nodes[i].x << ", " << nodes[i].y << ") ";
            }
            auto knn_nodes = KNN(nodes);
            PRMNode G_h{G_h.x = map->GetEndPoint()[1], G_h.y = map->GetEndPoint()[0], G_h.id = TARGET_ID};

            int astar_nodes = knn_nodes.size();
            std::vector<float> heuristic;
            Graph g(astar_nodes);

            for (int i = 0; i < knn_nodes.size(); i++)
            {

                PRMNode a = std::get<0>(knn_nodes[i]);
                PRMNode b = std::get<1>(knn_nodes[i]);

                float dist = measureNodeDistance(a, b);
                heuristic.push_back(measureNodeDistance(a, G_h));

                g.addEdge(a.id, b.id, static_cast<int>(dist));
            }

            g.computeStarA(0, TARGET_ID, heuristic);


            std::set<std::tuple<int, int>> navSet;
            std::vector<PRMNode> nav;
            for (auto &i : g.optimalNodes)
            {

                for (unsigned int j = 0; j < knn_nodes.size(); j++)
                {
                    PRMNode a = std::get<0>(knn_nodes[j]);
                    if (i == a.id)
                    {
                        if(navSet.find(std::make_tuple(a.x, a.y)) == navSet.end())
                        {
                            navSet.insert(std::make_tuple(a.x, a.y));
                            nav.push_back(PRMNode(a.x, a.y, 0));
                        }
                    }
                }
            }
            std::reverse(nav.begin(), nav.end());
            auto fromNode = nav[0];
            for(unsigned int i = 1; i < nav.size(); i++)
            {
                auto toNode = nav[i];
                auto points = getPointsBetweenNodes(fromNode, toNode);
                if(points[0].x == toNode.x && points[0].y == toNode.y)
                {
                    std::reverse(points.begin(), points.end());
                }
                auto first = points[0];
                for (auto &p : points)
                {
                    if(first.x == p.x && first.y == p.y)
                    {
                        continue;
                    }
                    movement.push_back(std::make_tuple(p.x, p.y));
                }
                fromNode = toNode;
            }
        }
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

    bool ReturnThepath(void) override
    {
        return false;
    }
    bool IsTerminated(void) override 
    {
        return false;
    }
private:
    Space2D curMap;
    int curX;
    int curY;
    unsigned int offset = 0;
    int ticks = 0;
    std::vector<std::tuple<int, int>> movement;
    std::vector<PRMNode> generateNodes()
    {

        std::vector<PRMNode> vecNodes;
        std::vector<PRMNode> vecNodesEx;
        std::random_device rd;
        std::mt19937 gen(rd());
        vecNodes.push_back(PRMNode(map->GetStartPoint()[1], map->GetStartPoint()[0], 0));
        std::uniform_int_distribution<> distY(0, Parameter::NYGRID-1);
        std::uniform_int_distribution<> distX(0, Parameter::NXGRID-1);
        for (int ii = 1; ii < NODES+1; ii++)
        {

            vecNodes.push_back(PRMNode(distX(gen), distY(gen), ii));
        }
        vecNodes.push_back(PRMNode(map->GetEndPoint()[1], map->GetEndPoint()[0], TARGET_ID));
        return nodeObsExclusion(vecNodes);
    }

    std::vector<PRMNode> nodeObsExclusion(std::vector<PRMNode> vecNodes)
    {
        std::vector<PRMNode> vecNodesEx;
        for (auto &ii : vecNodes)
        {
            if (map->Get(ii.x,ii.y).state != 1)
            {
                vecNodesEx.push_back(ii);
            }
        }
        return vecNodesEx;
    }

    bool checkCollisionWithObs(PRMNode node1, PRMNode node2)
    {

        for (auto& node : getPointsBetweenNodes(node1, node2))
        {
            if (map->Get(node.x, node.y).state == 1)
            {
                return true;
            }
        }
        return false;
    }

    std::vector<PRMNode> getPointsBetweenNodes(PRMNode node1, PRMNode node2)
    {
        std::vector<PRMNode> points;
        int x1 = node1.x;
        int x2 = node2.x;
        int y1 = node1.y;
        int y2 = node2.y;
        
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
                points.push_back(PRMNode(y, x, 0));
            } else {
                points.push_back(PRMNode(x, y, 0));
            }
            error -= dy;
            if (error < 0) {
                y += ystep;
                error += dx;
            }
        }

        return points;
    }
    
    float measureNodeDistance(PRMNode node1, PRMNode node2)
    {
        return static_cast<float>(std::sqrt(std::pow((node1.x - node2.x), 2) + std::pow((node1.y - node2.y), 2)));
    }
    bool checkDistance(PRMNode node1, PRMNode node2)
    {

        auto dist = measureNodeDistance(node1, node2);
        return dist <= RADIUS ? true : false;
    }

    std::vector<std::tuple<PRMNode, PRMNode>> KNN(std::vector<PRMNode> nodes)
    {

        std::vector<std::tuple<PRMNode, PRMNode>> knn_nodes;

        for (unsigned int i = 0; i < nodes.size(); i++)
        {

            for (unsigned int j = 0; j < nodes.size(); j++)
            {

                if (i != j)
                {
                    bool checkColl = checkCollisionWithObs(nodes[i], nodes[j]);
                    if (checkDistance(nodes[i], nodes[j]) && checkColl != true)
                    {

                        knn_nodes.push_back({nodes[i], nodes[j]});
                    }
                }
            }
        }

        return knn_nodes;
    }
};

#endif /* prm_h */