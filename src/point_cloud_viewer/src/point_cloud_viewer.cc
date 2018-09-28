/*
	Views each input point cloud and merge them with ICP matching
		1. select 2 points from each point cloud (with shift + left-click) that has overlap
		2. preview all point clouds before performing ICP matching
		3. perform ICP matching
		4. Done
*/

#include <iostream>
#include <vector>

int main(int argc, char** argv)
{
	for (int i = 0; i < argc; ++i)
	{
		printf("argv[%i]: %s\n", i, argv[i]);
	}

	return 0;
}
