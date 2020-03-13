#include <iostream>
#include <vector>
#include <fstream> 
#include <limits>
#include <stack>
#include <queue>
#include <map>
#include <chrono>
#include <thread>
#include <stack>
#include <cstdlib>
#include <cmath>
#include <algorithm>

using namespace std;
using namespace std::chrono_literals;

//--------------------------------------------------------------------------------------------------------------------
enum Side { Left, Right, Up, Down};
class Node
{
    public:
    pair<double,double> coordinates;
    Node*               ancestor;
    bool                canStep;
    bool                isOpened;
    bool                inRoad;
    double              heuristic;
    double              distance_from_start;

    Node(bool node,double X, double Y):
        canStep(node){
        coordinates         = make_pair(X,Y);
        distance_from_start = numeric_limits<double>::infinity();
        ancestor            = nullptr;
        inRoad              = false;
        isOpened            = false;
    };
    void    ManhattanDist   (Node * end)
    {
        this->heuristic = abs(this->coordinates.first - end->coordinates.first) + abs(this->coordinates.second - end->coordinates.second);
    }
    void    EuclideanDist   (Node * end)
    {
        this->heuristic = sqrt(pow(abs(this->coordinates.first - end->coordinates.first),2.0) + pow(abs(this->coordinates.second - end->coordinates.second),2.0));
    }
    void    AdjustNode      (Node * Ancestor)
    {
        distance_from_start = Ancestor->distance_from_start + 1;
        isOpened = true;
        ancestor = Ancestor;
    }

};
//--------------------------------------------------------------------------------------------------------------------
class Grid
{
    public:
    vector<vector<Node>> grid;
    Node*                start;
    Node*                end;
    double               PathLength;
    
    Node&   findNode            (Node * node, Side side)
    {
        switch (side)
        {
        case Left:
            return grid.at(node->coordinates.second).at(node->coordinates.first-1);
        case Right:
            return grid.at(node->coordinates.second).at(node->coordinates.first+1);
        case Up:
            return grid.at(node->coordinates.second-1).at(node->coordinates.first);
        case Down:
            return grid.at(node->coordinates.second+1).at(node->coordinates.first);
        default:
            break;
        }
    } 
    void    ReadGrid            (ifstream & is)
    {
        int x = 0;
        int y = 0;
        char c;
        vector<Node> help_vector;
        while (is.get(c))
        {
            if (c != 'X' && c != '\n' && c != ' ')
                break;
            if (c == 'X')
            {
                Node newNode = Node(false,x,y);
                help_vector.push_back(newNode);
                x++;
            }
            else if (c == ' ')
            {
                Node newNode = Node(true,x,y);
                help_vector.push_back(newNode);
                x++;
            }
            else if (c == '\n')
            {
                y++;
                x = 0;
                grid.push_back(help_vector);
                help_vector.clear();
            }
        }

    }
    void    setStartEnd         (ifstream & is)
    {
        int start1,start2,end1,end2;
        is.ignore(numeric_limits<streamsize>::max(),' ');
        is >> start1;
        cout << "Start x: " << start1 << ", y: ";
        is.ignore(numeric_limits<streamsize>::max(),' ');
        is >> start2;
        cout << start2 << endl;
        is.ignore(numeric_limits<streamsize>::max(),' ');
        is >> end1;
        cout << "End x: " << end1 << ", y: ";
        is.ignore(numeric_limits<streamsize>::max(),' ');
        is >> end2;
        cout << end2 << endl;
        is.close();
        start = &(grid.at(start2).at(start1));
        end = &(grid.at(end2).at(end1));
        start->distance_from_start = 0;
    }
    void    printGrid           ()
    {
        system("clear");
        for (auto &x : grid)
            {
                for (auto &y : x)
                {
                    /**
                     * VERY IMPORTANT PART OF A* ALGO , THESE 2 METHODS COMPUTE VALUES FOR ACCURATE CHOOSING OF NODES
                     * 
                     * YOU CAN CHOOSE ONE, MANHATTAN SEEMS TO BE MORE ACCURATE
                     * **/

                    //y.EuclideanDist(grid.end);
                    y.ManhattanDist(end);

                    if (y.canStep)
                    {
                        if (&y == end)
                        {
                            cout << "\033[1;31mE\033[0m";
                        }
                        else if(&y == start)
                        {
                            cout << "\033[1;31mS\033[0m";
                        }
                        else if (y.isOpened && !y.inRoad)
                        {
                            cout << "\033[1;31mX\033[0m";
                        }
                        else if (y.inRoad)
                        {
                            cout << "\033[1;36mX\033[0m";
                        }
                        else
                        {
                            cout << " ";
                        }
                    }else
                    {
                        cout << "X";
                    }
                }
                cout << endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    void    reassembleRoad      ()
    {
        Node *node = end;
        while(node != nullptr)
        {
            node->inRoad = true;
            node = node->ancestor;
        }
        printGrid();
        cout << "Distance = " << PathLength << endl;
    }
    bool    LookAtSides         (Node * node, Side side)
    {
        switch (side)
        {
        case Left:
                return !findNode(node,Left).isOpened;         // look if node is already opened
                        //&& findNode(node,Left).canStep;
        case Right:
                return !findNode(node,Right).isOpened;  
                        //&& findNode(node,Right).canStep;   
        case Up:
                return !findNode(node,Up).isOpened;  
                        //&& findNode(node,Up).canStep;   
        case Down:
                return !findNode(node,Down).isOpened;  
                        //&& findNode(node,Down).canStep;   
        default:
                return false;
        }
    }
    void    RoadNotFound        ()
    {
        cout << "Road does not exists" << endl;
        return;
    }
    void    DijkstraRelaxation  (Node * node, Side side)
    {
        switch (side)
        {
        case Left:
                if (findNode(node,Left).distance_from_start > node->distance_from_start + 1)
                {
                    findNode(node,Left).AdjustNode(node);
                }
            break;
        case Right:
                if (findNode(node,Right).distance_from_start > node->distance_from_start + 1)
                {
                    findNode(node,Right).AdjustNode(node);
                }
            break;
        case Down:
                if (findNode(node,Down).distance_from_start > node->distance_from_start + 1)
                {
                    findNode(node,Down).AdjustNode(node);
                }
            break;
        case Up:
                if (findNode(node,Up).distance_from_start > node->distance_from_start + 1)
                {
                    findNode(node,Up).AdjustNode(node);
                }
            break;

        default:
            break;
        }
    }
    void    getRandomNode       (vector<Node*> &v_RandomS,Node* &node)
    {
        int i = 0;
        srand( (unsigned)time(NULL) );
        i = rand() % v_RandomS.size();
        node = v_RandomS.at(i);
        v_RandomS.erase(v_RandomS.begin() + i);
        
    }
    //--------------------------------------------------------------------------------------------------------------------
    void DFS                    ();
    void BFS                    ();
    void Dijkstra               ();
    void RandomSearch           ();
    void GreedySearch           ();
    void AStar                  ();



};
//--------------------------------------------------------------------------------------------------------------------
struct LessThanByDist
{
  bool operator()(Node* lhs,Node* rhs) 
  {
    return lhs->distance_from_start > rhs->distance_from_start;
  }
};
struct LessThanByHeur
{
  bool operator()(Node* lhs,Node* rhs) 
  {
    return lhs->heuristic > rhs->heuristic;
  }
};

struct LessThanForAStar
{
  bool operator()(Node* lhs,Node* rhs) 
  {
    return lhs->heuristic + lhs->distance_from_start > rhs->heuristic + rhs->distance_from_start;
  }
};
//--------------------------------------------------------------------------------------------------------------------
void Grid::DFS()
{
    Node*         node = nullptr;
    stack<Node*>  s_DFS;
    
    s_DFS.push(start);
    
    while(!s_DFS.empty())
    {
        node           = s_DFS.top();
        node->isOpened = true;        
        s_DFS.pop();
        
        if(!node->canStep)
            continue;
        
        if (node == end)
        {
            PathLength = node->distance_from_start;
            reassembleRoad();
            return;
        }
        
        try
                    {
                            if (LookAtSides(node,Left))
                            {
                                s_DFS.push(&findNode(node,Left));
                                findNode(node,Left).AdjustNode(node);
                            }
                    }
        catch(const std::exception& e)
                    {
                    }
        try
                    {
                            if (LookAtSides(node,Right))
                            {
                                s_DFS.push(&findNode(node,Right));
                                findNode(node,Right).AdjustNode(node);
                            }    
                    }
        catch(const std::exception& e)
                    {
                    }
        try
                    {
                            if (LookAtSides(node,Up))
                            {
                                s_DFS.push(&findNode(node,Up));
                                findNode(node,Up).AdjustNode(node);
                            }
                    }
        catch(const std::exception& e)
                    {
                    }
        try
                    {
                            if (LookAtSides(node,Down))
                            {
                                s_DFS.push(&findNode(node,Down));
                                findNode(node,Down).AdjustNode(node);
                            }
                    }
        catch(const std::exception& e)
                    {
                    }
    }
    RoadNotFound();
    return;
}
void Grid::BFS()
{
    Node*         node = nullptr;
    queue <Node*> q_BFS;
    q_BFS.push(start);
    
    while(!q_BFS.empty())
    {
        node = q_BFS.front();
        node->isOpened = true;
        q_BFS.pop();
        
        if(!node->canStep)
            continue;
        
        if (node == end)
        {
            PathLength = node->distance_from_start;
            reassembleRoad();
            return;
        }

        try
                    {
                            if (LookAtSides(node,Left))
                            {
                                q_BFS.push(&findNode(node,Left));
                                findNode(node,Left).AdjustNode(node);
                            }
                    }
        catch(const std::exception& e)
                    {
                    }
        try
                    {
                            if (LookAtSides(node,Right))
                            {
                                q_BFS.push(&findNode(node,Right));
                                findNode(node,Right).AdjustNode(node);
                            }    
                    }
        catch(const std::exception& e)
                    {
                    }

        try
                    {
                            if (LookAtSides(node,Up))
                            {
                                q_BFS.push(&findNode(node,Up));
                                findNode(node,Up).AdjustNode(node);
                            }
                    }
        catch(const std::exception& e)
                    {
                    }

        try
                    {
                            if (LookAtSides(node,Down))
                            {
                                q_BFS.push(&findNode(node,Down));
                                findNode(node,Down).AdjustNode(node);
                            }
                    }
        catch(const std::exception& e)
                    {
                    }
    }
    RoadNotFound();
    return;

}
void Grid::Dijkstra()
{
    Node * node = nullptr;
    priority_queue<Node*, vector <Node*> , LessThanByDist > pq_Dijkstra;
    pq_Dijkstra.push(start);
    
    while(!pq_Dijkstra.empty())
    {
        node = pq_Dijkstra.top();
        pq_Dijkstra.pop();
        node->isOpened = true;

        if(!node->canStep)
            continue;
        
        if (node == end)
        {
            PathLength = node->distance_from_start;
            reassembleRoad();
            return;
        }

        try
                    {
                            if (!LookAtSides(node,Left))
                            {
                                DijkstraRelaxation(node,Left);
                            }
                            else 
                            {
                                pq_Dijkstra.push(&findNode(node,Left));
                                findNode(node,Left).AdjustNode(node);  
                            }
                    }
        catch(const std::exception& e)
                    {
                    }
        try
                    {
                            if (!LookAtSides(node,Right))
                            {
                                DijkstraRelaxation(node,Right);
                            }else 
                            {
                                pq_Dijkstra.push(&findNode(node,Right));
                                findNode(node,Right).AdjustNode(node);
                                
                            } 
                    }
        catch(const std::exception& e)
                    {
                    }

        try
                    {
                            if (!LookAtSides(node,Up))
                            {
                                DijkstraRelaxation(node,Up);
                            }else 
                            {
                                pq_Dijkstra.push(&findNode(node,Up));
                                findNode(node,Up).AdjustNode(node);
                                
                            }      
                    }
        catch(const std::exception& e)
                    {
                    }

        try
                    {
                            if (!LookAtSides(node,Down))
                            {
                                DijkstraRelaxation(node,Down);
                            }else 
                            {
                                pq_Dijkstra.push(&findNode(node,Down));
                                findNode(node,Down).AdjustNode(node);
                                
                            }
                    }
        catch(const std::exception& e)
                    {
                    }
    }
    RoadNotFound();
    return;
}
void Grid::RandomSearch()
{
    int i = 0;
    vector<Node*> v_RandomS;
    v_RandomS.push_back(start);
    Node* node = nullptr;
    
    while(!v_RandomS.empty())
    {
        getRandomNode(v_RandomS,node);   
        node->isOpened = true;
        
        if(!node->canStep)
            continue;
        
        if (node == end)
        {
            PathLength = node->distance_from_start;
            reassembleRoad();
            return;
        }

        try
                    {
                            if (LookAtSides(node,Left))
                            {
                                v_RandomS.push_back(&findNode(node,Left));
                                findNode(node,Left).AdjustNode(node);
                            }
                    }
        catch(const std::exception& e)
                    {
                    }
        try
                    {
                            if (LookAtSides(node,Right))
                            {
                                v_RandomS.push_back(&findNode(node,Right));
                                findNode(node,Right).AdjustNode(node);
                            }    
                    }
        catch(const std::exception& e)
                    {
                    }

        try
                    {
                            if (LookAtSides(node,Up))
                            {
                                v_RandomS.push_back(&findNode(node,Up));
                                findNode(node,Up).AdjustNode(node);
                            }
                    }
        catch(const std::exception& e)
                    {
                    }

        try
                    {
                            if (LookAtSides(node,Down))
                            {
                                v_RandomS.push_back(&findNode(node,Down));
                                findNode(node,Down).AdjustNode(node);
                            }
                    }
        catch(const std::exception& e)
                    {
                    }

    }
        RoadNotFound();
        return;
}
void Grid::GreedySearch()
{
    priority_queue<Node*, vector <Node*> , LessThanByHeur > pq_GreedyS;
    pq_GreedyS.push(start);
    Node * node = nullptr;
    while(!pq_GreedyS.empty())
    {
        node = pq_GreedyS.top();
        pq_GreedyS.pop();
        node->isOpened = true;
        
        if(!node->canStep)
            continue;
        
        if (node == end)
        {
            PathLength = node->distance_from_start;
            reassembleRoad();
            return;
        }
        try
                    {
                            if (LookAtSides(node,Left))
                            {
                                pq_GreedyS.push(&findNode(node,Left));
                                findNode(node,Left).AdjustNode(node);
                            }
                    }
        catch(const std::exception& e)
                    {
                    }
        try
                    {
                            if (LookAtSides(node,Right) )
                            {
                                pq_GreedyS.push(&findNode(node,Right));
                                findNode(node,Right).AdjustNode(node);
                            }    
                    }
        catch(const std::exception& e)
                    {
                    }

        try
                    {
                            if (LookAtSides(node,Up))
                            {
                                pq_GreedyS.push(&findNode(node,Up));
                                findNode(node,Up).AdjustNode(node);
                            }
                    }
        catch(const std::exception& e)
                    {
                    }

        try
                    {
                            if (LookAtSides(node,Down))
                            {
                                pq_GreedyS.push(&findNode(node,Down));
                                findNode(node,Down).AdjustNode(node);
                            }
                    }
        catch(const std::exception& e)
                    {
                    }
    }
    RoadNotFound();
    return;
}
void Grid::AStar()
{
    priority_queue<Node*, vector <Node*> , LessThanForAStar > pq_Astar;
    pq_Astar.push(start);
    Node * node = nullptr;
    while(!pq_Astar.empty())
    {
        node = pq_Astar.top();
        pq_Astar.pop();
        node->isOpened = true;
        
        if(!node->canStep)
            continue;
        
        if (node == end)
        {
            PathLength = node->distance_from_start;
            reassembleRoad();
            return;
        }

        try
                    {
                            if (LookAtSides(node,Left))
                            {
                                pq_Astar.push(&findNode(node,Left));
                                findNode(node,Left).AdjustNode(node);
                            }
                    }
        catch(const std::exception& e)
                    {
                    }
        try
                    {
                            if (LookAtSides(node,Right))
                            {
                                pq_Astar.push(&findNode(node,Right));
                                findNode(node,Right).AdjustNode(node);
                            }    
                    }
        catch(const std::exception& e)
                    {
                    }

        try
                    {
                            if (LookAtSides(node,Up))
                            {   
                                pq_Astar.push(&findNode(node,Up));
                                findNode(node,Up).AdjustNode(node);
                            }
                    }
        catch(const std::exception& e)
                    {
                    }

        try
                    {
                            if (LookAtSides(node,Down))
                            {
                                pq_Astar.push(&findNode(node,Down));
                                findNode(node,Down).AdjustNode(node);
                            }
                    }
        catch(const std::exception& e)
                    {
                    }
    }
    RoadNotFound();
    return;
}

int main(int argc, char *argv[])
{
    Grid grid;
    ifstream is(argv[1]);
    grid.ReadGrid(is);
    grid.setStartEnd(is);
    grid.printGrid();

    int cislo_algu;
    cout << "Choose aglorithm: \n1.BFS\n2.DFS\n3.Dijkstra\n4.Random search\n5.Greedy Search\n6.A*" << endl;
    cin >> cislo_algu;
    switch (cislo_algu)
    {
    case 1:
        grid.BFS();
        break;
    case 2:
        grid.DFS();
        break;
    case 3:
        grid.Dijkstra();
        break;
    case 4:
        grid.RandomSearch();
        break;
    case 5:
        grid.GreedySearch();
        break;
    case 6:
        grid.AStar();
        break;
    default:
        cout << "Not an option" << endl;
        break;
    }


    
    
}