#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <optional>
#include <cstdint>
#include <utility>
#include <algorithm>



#if defined(_MSC_VER)
#define Unreachable() {__debugbreak(); __assume(0);}
#define SCANF sscanf_s
#define SCANF_S_ARG(arg) ,arg
#else
#define SCANF sscanf
#define SCANF_S_ARG(arg)
#define Unreachable() __builtin_unreachable();
#endif

template<typename T>
void EasyErase(std::vector<T>& vec, T const& toErase)
{
    auto itr = std::find(vec.begin(), vec.end(), toErase);
    if (itr == vec.end()) return;
    vec.erase(itr);
}

enum class Direction : uint8_t
{
    Up,
    Right,
    Down,
    Left,
    Count
};

enum class Reflection : uint8_t
{
    Leading,
    Trailing,
    Count
};

template <typename T>
int constexpr AsInt(T e)
{
    return static_cast<int>(e);
}

template <typename T>
size_t constexpr AsSizeT(T e)
{
    return static_cast<size_t>(e);
}

template <typename T>
Direction constexpr AsDir(T e)
{
    return static_cast<Direction>(e);
}

Direction Rotate(Direction startDir, uint8_t count = 1)
{
    return static_cast<Direction>((AsInt(startDir) + count) % AsInt(Direction::Count));
}

Direction Reflect(Direction startDir, Reflection reflector)
{
    // Leading \
    // Up    (0) -> Left  (3)
    // Right (1) -> Down  (2)
    // Down  (2) -> Right (1)
    // Left  (3) -> Up    (0)

    // Trailing /
    // Up    (0) -> Right (2)
    // Right (1) -> Up    (0)
    // Down  (2) -> Left  (3)
    // Left  (3) -> Down  (2)
    static const Direction lookup[AsInt(Reflection::Count)][AsInt(Direction::Count)] =
    {
        {
            Direction::Left, Direction::Down, Direction::Right, Direction::Up
        },
        {
            Direction::Right, Direction::Up, Direction::Left, Direction::Down
        }
    };

    return lookup[AsInt(reflector)][AsInt(startDir)];
}

int DirectionToX(Direction dir)
{
    static int lookup[] = { 0, 1, 0, -1 };
    return lookup[AsInt(dir)];
}

int DirectionToY(Direction dir)
{
    static int lookup[] = { -1, 0, 1, 0 };
    return lookup[AsInt(dir)];
}

Direction XToDirection(int dX)
{
    if (dX < 0) return Direction::Left;
    if (dX > 0) return Direction::Right;
    return Direction::Count;
}

Direction YToDirection(int dY)
{
    if (dY < 0) return Direction::Up;
    if (dY > 0) return Direction::Down;
    return Direction::Count;
}

uint8_t DirectionToMask(Direction dir)
{
    return 1 << AsInt(dir);
}

struct DirectionIterator
{
    struct Iterator
    {
        int asInt;
        Direction asDir;

        std::pair<int, Direction> operator*() { return { asInt, asDir }; }
        Iterator operator++(int) { return Iterator{ asInt + 1, AsDir(asInt + 1) }; }
        Iterator& operator++() { ++asInt; asDir = AsDir(asInt); return *this; }
        bool operator!=(Iterator const& rhs) { return asInt != rhs.asInt; }
    };

    Iterator begin() { return Iterator{ AsInt(Direction::Up), Direction::Up }; }
    Iterator end() { return Iterator{ AsInt(Direction::Count), Direction::Count }; }
};

const char* DirectionToString(Direction dir)
{
    switch (dir)
    {
    case Direction::Up: return "Up";
    case Direction::Right: return "Right";
    case Direction::Down: return "Down";
    case Direction::Left: return "Left";
    default: return "Invalid";
    }
}

char DirectionToArrow(Direction dir)
{
    switch (dir)
    {
    case Direction::Up: return '^';
    case Direction::Right: return '>';
    case Direction::Down: return 'v';
    case Direction::Left: return '<';
    default: return 'A';
    }
}

template <typename T>
struct TwoDVector
{
    TwoDVector(size_t innerDim, size_t reserve = 0) : innerDim(innerDim) { _vec.reserve(reserve ? reserve : innerDim); }

    template <typename T2>
    TwoDVector(TwoDVector<T2> const& input) :
        innerDim(input.innerDim)
    {
        _vec.reserve(input._vec.size());
        for (auto& element : input) _vec.emplace_back(element);
    }

    T* operator[](size_t idx)
        requires (!std::is_same<T, bool>::value)
    {
        return _vec.data() + idx * innerDim;
    }

    T const* operator[](size_t idx) const
        requires (!std::is_same<T, bool>::value)
    {
        return _vec.data() + idx * innerDim;
    }

    T& get(size_t y, size_t x)
        requires(!std::is_same<T, bool>::value)
    {
        return _vec[y * innerDim + x];
    }

    T const& get(size_t y, size_t x) const
        requires(!std::is_same<T, bool>::value)
    {
        return _vec[y * innerDim + x];
    }

    T get(size_t y, size_t x) const
        requires(std::is_same<T, bool>::value)
    {
        return _vec[y * innerDim + x];
    }

    T& get(size_t y, size_t x, Direction offset, int count)
        requires(!std::is_same<T, bool>::value)
    {
        y += DirectionToY(offset) * count;
        x += DirectionToX(offset) * count;
        return get(y, x);
    }

    T const& get(size_t y, size_t x, Direction offset, int count) const
        requires(!std::is_same<T, bool>::value)
    {
        y += DirectionToY(offset) * count;
        x += DirectionToX(offset) * count;
        return get(y, x);
    }

    T get(size_t y, size_t x, Direction offset, int count) const
        requires(std::is_same<T, bool>::value)
    {
        y += DirectionToY(offset) * count;
        x += DirectionToX(offset) * count;
        return get(y, x);
    }

    void set(size_t y, size_t x, T const& value)
    {
        _vec[y * innerDim + x] = value;
    }

    void set(size_t y, size_t x, Direction offset, int count, T const& value)
    {
        y += DirectionToY(offset) * count;
        x += DirectionToX(offset) * count;
        set(y, x, value);
    }

    T* GetPointerIfInBounds(size_t y, size_t x)
    {
        if (!IsInBounds(y, x)) return nullptr;
        return &get(y, x);
    }

    T const* GetPointerIfInBounds(size_t y, size_t x) const
    {
        if (!IsInBounds(y, x)) return nullptr;
        return &get(y, x);
    }

    std::optional<T> GetIfInBounds(size_t y, size_t x) const
    {
        if (!IsInBounds(y, x)) return std::optional<T>();
        return get(y, x);
    }

    T* GetPointerIfInBounds(size_t y, size_t x, Direction offset, int count)
    {
        y += DirectionToY(offset) * count;
        x += DirectionToX(offset) * count;
        return GetPointerIfInBounds(y, x);
    }

    T const* GetPointerIfInBounds(size_t y, size_t x, Direction offset, int count) const
    {
        y += DirectionToY(offset) * count;
        x += DirectionToX(offset) * count;
        return GetPointerIfInBounds(y, x);
    }

    std::optional<T> GetIfInBounds(size_t y, size_t x, Direction offset, int count) const
    {
        y += DirectionToY(offset) * count;
        x += DirectionToX(offset) * count;
        return GetIfInBounds(y, x);
    }

    void push_back(T const& val)
    {
        _vec.push_back(val);
    }

    template<typename iterator>
    void append(iterator begin, iterator end)
    {
        _vec.insert(_vec.end(), begin, end);
    }

    void resize(size_t newsize)
    {
        _vec.resize(newsize);
    }

    typename std::vector<T>::iterator begin()
    {
        return _vec.begin();
    }

    typename std::vector<T>::iterator end()
    {
        return _vec.end();
    }

    typename std::vector<T>::const_iterator begin() const
    {
        return _vec.begin();
    }

    typename std::vector<T>::const_iterator end() const
    {
        return _vec.end();
    }

    size_t XDim() const
    {
        return innerDim;
    }

    size_t YDim() const
    {
        return _vec.size() / innerDim;
    }

    size_t IsInBounds(int64_t x, int64_t y) const
    {
        return y >= 0ll && x >= 0ll && y < static_cast<int64_t>(YDim()) && x < static_cast<int64_t>(XDim());
    }

    size_t innerDim;
    std::vector<T> _vec;
};

std::vector<std::string> GetInputAsString(std::ifstream& ifstream)
{
    std::vector<std::string> output;
    std::string line;
    while (std::getline(ifstream, line))
    {
        output.emplace_back(std::move(line));
    }
    return output;
}

std::vector<std::string> GetInputAsString(const char* fpath)
{
    std::ifstream ifstream;
    ifstream.open(fpath);
    std::vector<std::string> output;
    std::string line;
    while (std::getline(ifstream, line))
    {
        output.emplace_back(std::move(line));
    }
    return output;
}

template<typename T>
TwoDVector<T> GetInputGrid(std::vector<std::string> const& inputLines)
{
    TwoDVector<T> output(inputLines.front().size(), inputLines.front().size() * inputLines.size());
    for (auto& line : inputLines)
    {
        output.append(line.begin(), line.end());
    }
    return output;
}

template<typename T>
TwoDVector<T> GetInputGrid(std::ifstream& ifstream)
{
    return GetInputGrid<T>(GetInputAsString(ifstream));
}

template<typename T>
TwoDVector<T> GetInputGrid(const char * fpath)
{
    std::ifstream ifstream;
    ifstream.open(fpath);
    return GetInputGrid<T>(GetInputAsString(ifstream));
}