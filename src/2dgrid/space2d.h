#ifndef SPACE2D_H_IS_INCLUDED
#define SPACE2D_H_IS_INCLUDED

#include <iostream>
#include <sstream>
#include "lattice2d.h"
#include "gridbase.h"
#include <string>
#include <vector>
#include <fstream>
#include "parameter.h"
class Space2D : public GridBase, public Lattice2D<GridBase::Grid>
{
public:
	int startindex = -1;
	int endindex = -1;
	Parameter *param;
	std::string filepath;
	std::vector<std::string> Parse(std::string incoming)
	{
		int state = 0;
		std::vector<std::string> words;
		std::string currentWord;
		for (auto c : incoming)
		{
			if (0 == state)
			{
				if (' ' != c && '\n' != c && '\t' != c && '\r' != c)
				{
					state = 1;
					currentWord.push_back(c);
				}
			}
			else
			{
				if (' ' != c && '\n' != c && '\t' != c && '\r' != c)
				{
					currentWord.push_back(c);
				}
				else
				{
					state = 0;
					words.push_back(currentWord);
					currentWord = "";
				}
			}
		}
		if (0 != state)
		{
			words.push_back(currentWord);
		}
		return words;
	}
	Space2D()
	{
	}
	Space2D(const Space2D &incoming)
	{
		CopyFrom(incoming);
	}
	Space2D &operator=(const Space2D &incoming)
	{
		CopyFrom(incoming);
		return *this;
	}
	void CopyFrom(const Space2D &incoming)
	{
		if (this != &incoming)
		{
			CleanUp();

			this->width = incoming.width;
			this->height = incoming.height;
			this->elem = new GridBase::Grid[width * height];
			for (int y = 0; y < GetHeight(); ++y)
			{
				for (int x = 0; x < GetWidth(); ++x)
				{
					Set(x, y, incoming.elem[y * width + x]);
				}
			}
		}
	}
	~Space2D()
	{
		CleanUp();
	}
	void Initialize(int wid, int hei);
	void LoadMap(const std::string filename)
	{
		filepath = filename;
		if (this->width == 0 || this->height == 0)
		{
			std::cerr << "please load map after initialization" << std::endl;
		}
		std::vector<int> map;
		int nblock = this->width * this->height;
		std::ifstream ifp(filename);
		if (true == ifp.is_open())
		{
			std::string str;
			int nBlockRead = 0;
			while (true != ifp.eof())
			{
				std::getline(ifp, str);
				auto words = Parse(str);
				if (2 <= words.size() && words.size() + nBlockRead <= nblock)
				{
					for (std::string word : words)
					{
						// std::cout<<word<<std::endl;
						Grid g(atoi(word.c_str()));
						this->elem[nBlockRead] = g;
						if (2 == g.state)
						{
							startindex = nBlockRead;
						}
						else if (3 == g.state)
						{
							endindex = nBlockRead;
						}
						nBlockRead++;
					}
				}
			}
		}
		else
		{
			std::cerr << "unable to open map file!" << std::endl;
		}
		std::cout << map.size() << std::endl;
	}
	void SetStates(int index, int state)
	{
		if (0 == state || 1 == state)
		{
			this->elem[index].state = state;
		}
		else if (2 == state)
		{
			if (startindex == -1)
			{
				startindex = index;
				this->elem[index].state = state;
			}
			else
			{
				this->elem[startindex].state = 0;
				startindex = index;
				this->elem[index].state = state;
			}
		}
		else if (3 == state)
		{
			if (endindex == -1)
			{
				endindex = index;
				this->elem[index].state = state;
			}
			else
			{
				this->elem[endindex].state = 0;
				endindex = index;
				this->elem[index].state = state;
			}
		}
	}

	std::vector<int> GetStartPoint() const
	{
		std::vector<int> startpoint;
		if (-1 != startindex)
		{
			startpoint.push_back(startindex / param->NXGRID);
			startpoint.push_back(startindex % param->NXGRID);
		}
		else{
			std::cout<<"Start point not initialized!"<<std::endl;
		}
		return startpoint;
	}

	std::vector<int> GetEndPoint() const
	{
		std::vector<int> endpoint;
		if (-1 != endindex)
		{
			endpoint.push_back(endindex / param->NXGRID);
			endpoint.push_back(endindex % param->NXGRID);
		}
		else
		{
			std::cout << "End point not initialized!" << std::endl;
		}
		return endpoint;
	}

	int GetGridState(int x, int y) const // this function does not check buffer overflow
	{
		return this->elem[x*param->NXGRID+y].state;
	}

	void saveMap()
	{
		std::ofstream mapOut;
		mapOut.open(filepath);
		if (true == mapOut.is_open())
		{
			for (int i = 0; i < param->NXGRID * param->NYGRID; ++i)
			{
				if (param->NXGRID - 1 != i % param->NXGRID || i == 0)
				{
					mapOut << this->elem[i].state << " ";
				}
				else
				{
					mapOut << this->elem[i].state << std::endl;
				}
			}
			mapOut.close();
		}
		else
		{
			std::cerr << "Cannot open output file" << std::endl;
		}
	}
};

#endif /* space2d_h */