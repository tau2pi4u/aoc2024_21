#include <unordered_map>
#include <map>
#include <array>
#include "utils.hpp"


#define TESTING 0
#if TESTING
#define INFILE "testInput.txt"
#else
#define INFILE "input.txt"
#endif

using Path = std::vector<Direction>;

// https://stackoverflow.com/questions/2590677/how-do-i-combine-hash-values-in-c0x
template <class T>
inline void hash_combine(std::size_t& seed, const T& v)
{
	std::hash<T> hasher;
	seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

struct PathHasher
{
	size_t operator()(Path const& path) const
	{
		size_t seed = 0;
		for (auto const& dir : path)
		{
			hash_combine(seed, dir);
		}
		return seed;
	}
};


struct Coord
{
	int x;
	int y;
};

Coord operator-(Coord const& lhs, Coord const& rhs)
{
	return Coord{ .x = lhs.x - rhs.x, .y = lhs.y - rhs.y };
}

Coord operator+(Coord const& lhs, Coord const& rhs)
{
	return Coord{ .x = lhs.x + rhs.x, .y = lhs.y + rhs.y };
}

bool operator==(Coord const& lhs, Coord const& rhs)
{
	return lhs.x == rhs.x && lhs.y == rhs.y;
}

/*
*     0   1   2
*   +---+---+---+
* 0 | 7 | 8 | 9 |
*   +---+---+---+
* 1 | 4 | 5 | 6 |
*   +---+---+---+
* 2 | 1 | 2 | 3 |
*   +---+---+---+
* 3 	| 0 | A |
*   	+---+---+
*/
Coord CharToCoord(char c)
{
	switch (c)
	{
	case '0': return { 1, 3 };
	case '1': return { 0, 2 };
	case '2': return { 1, 2 };
	case '3': return { 2, 2 };
	case '4': return { 0, 1 };
	case '5': return { 1, 1 };
	case '6': return { 2, 1 };
	case '7': return { 0, 0 };
	case '8': return { 1, 0 };
	case '9': return { 2, 0 };
	case 'A': return { 2, 3 };
	default: Unreachable();
	}
}

/*
*     0   1   2
*   	+---+---+
* 0 	| ^ | A |
*   +---+---+---+
* 1 | < | v | > |
*   +---+---+---+
*/
Coord DirToCoord(Direction dir)
{
	switch (dir)
	{
	case Direction::Up: return { 1, 0 };
	case Direction::Right: return { 2, 1 };
	case Direction::Down: return { 1, 1 };
	case Direction::Left: return { 0, 1 };
	case Direction::Count: return { 2, 0 };
	default: Unreachable();
	}
}


// Consider a straight path from start to end
// moving in X first.
bool OffsetToPathXFirst(Coord const& start, Coord const& end, Coord const& banned, std::vector<Path>& outPaths)
{
	Coord d = end - start;

	// If we don't move in X and this would be handled by the ypath, skip
	if (d.x == 0 && d.y != 0) return false;

	Coord corner = start + Coord{ .x = d.x, .y = 0 };

	// If the path is illegal, skip
	if (corner == banned) return false;


	// move in x then y
	outPaths.push_back(Path());
	for (int i = 0; i < abs(d.x); ++i)
	{
		outPaths.back().push_back(XToDirection(d.x));
	}
	for (int i = 0; i < abs(d.y); ++i)
	{
		outPaths.back().push_back(YToDirection(d.y));
	}

	// Press Button
	outPaths.back().push_back(Direction::Count);
	return true;
}

// Consider a straight path from start to end
// moving in Y first.
bool OffsetToPathYFirst(Coord const& start, Coord const& end, Coord const& banned, std::vector<Path>& outPaths)
{
	Coord d = end - start;

	// If we don't move in y, skip (Static handled by X)
	if (d.y == 0) return false;

	Coord corner = start + Coord{ .x = 0, .y = d.y };

	// If the path is illegal, skip
	if (corner == banned) return false;

	// y then x
	outPaths.push_back(Path());
	for (int i = 0; i < abs(d.y); ++i)
	{
		outPaths.back().push_back(YToDirection(d.y));
	}
	for (int i = 0; i < abs(d.x); ++i)
	{
		outPaths.back().push_back(XToDirection(d.x));
	}

	// Press Button
	outPaths.back().push_back(Direction::Count);
	return true;
}

// Find candidate paths (only consider straightest paths)
void OffsetToPaths(Coord const& start, Coord const& end, Coord const& banned, std::vector<Path> & outPaths)
{
	OffsetToPathXFirst(start, end, banned, outPaths);
	OffsetToPathYFirst(start, end, banned, outPaths);
	if (outPaths.empty()) Unreachable();
}

void PrintPath(Path const& path)
{
	for (auto const& dir : path)
	{
		printf("%c", DirectionToArrow(dir));
	}
}

// For robot n, generate the path for robot n+1
size_t PathToPath(Path const& in, int count)
{
	// We need a cache of { robot_id : { Path : length } }
	// Otherwise this takes forever
	static std::unordered_map<int, std::unordered_map<Path, size_t, PathHasher>> cache;

	// Our empty coordinate is at 0, 0
	static const Coord banned = { 0, 0 };

	// We are done, return the path length
	if (count == 0)
	{
		cache[count][in] = in.size();
	}

	// We have seen this before, we know where it ends
	if (cache.count(count) && cache.at(count).count(in))
	{
		return cache.at(count).at(in);
	}

	// Direction::Count is used for A
	// We always start at A (either this is the first input, or the 
	// previous one just pressed a button)
	size_t cost = 0;
	Direction current = Direction::Count;

	// Consider pairs of direcitons and generate the path for the robot
	// above to make this robot move from current to next and press it
	for (auto itr = in.begin(); itr != in.end(); ++itr)
	{
		std::vector<Path> stepPaths;
		Direction next = *itr;

		// Generate the paths
		OffsetToPaths(DirToCoord(current), DirToCoord(next), banned, stepPaths);

		// Consider the cost to the robots above
		size_t minPathCost = UINT64_MAX;
		Path minPath;

		for (auto const& path : stepPaths)
		{
			// Run this path through the robot above
			size_t cost = PathToPath(path, count - 1);
			if (cost < minPathCost)
			{
				minPathCost = cost;
			}
		}

		// Add our cost
		cost += minPathCost;
	
		current = next;
	}

	// cache our result (otherwise it takes forever)
	cache[count][in] = cost;
	return cost;
}

// Base layer to translate from a passcode
// to directions for the base robot
size_t StringToPathLength(std::string const& str, int robotCount)
{
	// We always start at A
	char current = 'A';

	size_t cost = 0;

	// Consider pairs of buttons
	for (auto itr = str.begin(); itr != str.end(); ++itr)
	{
		char next = *itr;

		// Get path that robot above needs to direct the base robot on
		std::vector<Path> stepPaths;
		OffsetToPaths(CharToCoord(current), CharToCoord(next), Coord{ 0, 3 }, stepPaths);


		size_t minPathCost = UINT64_MAX;
		Path minPath;

		for (auto const& path : stepPaths)
		{
			// Consider costs to robots above
			size_t cost = PathToPath(path, robotCount);
			if (cost < minPathCost)
			{
				minPath = path;
				minPathCost = cost;
			}
		}

		// Add this cost
		cost += minPathCost;

		current = next;
	}

	return cost;
}

// Per line scoring function
size_t Decode(std::string const& line, int robotCount)
{
	size_t length = StringToPathLength(line, robotCount);
	std::string extracted;
	for (auto const& c : line)
	{
		if (c >= '0' && c <= '9') extracted.push_back(c);
	}
	size_t numeric = std::stoi(extracted);
	//printf("%zu * %zu\n", length, numeric);
	return length * numeric;
}

// Overall scoring function
size_t Decode(std::vector<std::string> const& lines, int robotCount)
{
	size_t total = 0;
	for (auto const& line : lines)
	{
		total += Decode(line, robotCount);
	}
	return total;
}

#include <chrono>
int main()
{
	auto inputLines = GetInputAsString(INFILE);
	auto start = std::chrono::high_resolution_clock::now();
	auto p1 = Decode(inputLines, 2);
	auto p2 = Decode(inputLines, 25);
	auto end = std::chrono::high_resolution_clock::now();
	
	printf("%zu\n", p1);
	printf("%zu\n", p2);
	printf("%lld us\n", std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());
}