#ifndef LATTICE2D_H_IS_INCLUDED
#define LATTICE2D_H_IS_INCLUDED

#include <stdio.h>

template <class T>
class Lattice2D
{
protected:
	int width = 0, height = 0;
	T* elem = nullptr;
public:
	Lattice2D() {}
	~Lattice2D()
	{
		CleanUp();
	}
	void CleanUp(void)
	{
		if (nullptr != elem)
		{
			delete[] elem;
		}
		elem = nullptr;
		width = 0;
		height = 0;
	}

	void Create(unsigned int width, unsigned int height)
	{
		CleanUp();
		if (0 < width * height)
		{
			this->width = width;
			this->height = height;
			elem = new T[width * height];
		}
	}
	const int GetWidth(void) const
	{
		return width;
	}
	const int GetHeight(void) const
	{
		return height;
	}

	T& Get(int x, int y)
	{
		return elem[y * width + x];
	}
	const T& Get(int x, int y) const
	{
		return elem[y * width + x];
	}
	void Set(int x, int y, const T& incoming)
	{
		if (false == IsInRange(x, y))
		{
			printf("%s%d\n", __FUNCTION__, __LINE__);
			printf("(x=%d,y=%d)is out of the range.\n", x, y);
		}
		else
		{
			elem[y * width + x] = incoming;
		}
	}
	bool IsInRange(int x, int y) const
	{
		return (0 <= x && x < width && 0 <= y && y < height);
	}
};

#endif /* lattice2d_h */