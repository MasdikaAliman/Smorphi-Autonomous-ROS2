#ifndef WAVEFRONT_HPP
#define WAVEFRONT_HPP

#include <iostream>
#include <queue>
#include <stack>
#include <vector>
#include <climits>
#include "icecream.hpp"

class WaveFront
{

private:
    const int INF = INT_MAX;
    int dx[4] = {-1, 1, 0, 0};
    int dy[4] = {0, 0, -1, 1};

    int rows{0}, cols{0};

public:
    WaveFront();
    int getRows() { return rows; }
    int getCols() { return cols; }

    bool isValid(int x, int y, std::vector<std::vector<int>> &grid);
    std::vector<std::vector<int>> distanceTransform(std::vector<std::vector<int>> &grid, std::pair<int, int> goal);
    std::vector<std::pair<int, int>> areaCoverageWithBackTracking(std::vector<std::vector<int>> &distanceGrid, std::pair<int, int> start, std::pair<int, int> goal);
    std::vector<std::pair<int, int>> calculate(std::vector<std::vector<int>> grid, std::pair<int, int> start, std::pair<int, int> goal);
};

#endif