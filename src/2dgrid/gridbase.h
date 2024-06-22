#ifndef gridbase_h
#define gridbase_h

// This class is to operate just one grid
class GridBase
{
public:
	class Grid
	{
	public:
		// 0 for empty, 1 for obstacles, 2 for start pos, 3 for terminal
		int state = 0; //true: the grid is occupied with an obstacle
		bool obstacle = false;
		Grid(int incoming){
			this->state = incoming;
			if(1 == incoming){
				obstacle = true;
			}
		}
		Grid(){
		}
		~Grid(){}
	};
	// inline Grid MakeGrid(bool obstacle)
	// {
	// 	Grid g;
	// 	g.obstacle = obstacle;
	// 	retu
	// rn g;
	// }
};

#endif /* gridbase_h */