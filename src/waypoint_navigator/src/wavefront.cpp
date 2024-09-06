#include "waypoint_navigator/wavefront.hpp"

WaveFront::WaveFront()
{
    // IC_CONFIG.disable();
}

bool WaveFront::isValid(int x, int y, std::vector<std::vector<int>> &grid)
{
    return (x >= 0 && x < cols && y >= 0 && y < rows && grid[y][x] != -1);
}


std::vector<std::vector<int>> WaveFront::distanceTransform(std::vector<std::vector<int>> &grid, std::pair<int, int> goal)
{
    std::vector<std::vector<int>> distanceGrid(rows, std::vector<int>(cols, -1));
    std::queue<std::pair<int, int>> q;
    distanceGrid[goal.second][goal.first] = 0;
    q.push(goal);
    while (not q.empty())
    {
        std::pair<int, int> current = q.front();
        q.pop();
        int currDist = distanceGrid[current.second][current.first];
        for (int i = 0; i < 4; i++)
        {
            int newX = current.first + dx[i];
            int newY = current.second + dy[i];

            if (isValid(newX, newY, grid) and distanceGrid[newY][newX] == -1)
            {
                distanceGrid[newY][newX] = currDist + 1;
                q.push({newX, newY});
            }
        }
    }
    IC(distanceGrid);
    return distanceGrid;
}

std::vector<std::pair<int, int>> WaveFront::areaCoverageWithBackTracking(std::vector<std::vector<int>> &distanceGrid, std::pair<int, int> start, std::pair<int, int> goal)
{
    std::vector<std::pair<int, int>> ret_vector;
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    std::stack<std::pair<int, int>> pathStack;

    std::pair<int, int> current = start;
    pathStack.push(current);
    visited[current.second][current.first] = true;

    while (not pathStack.empty())
    {
        bool moved = false;
        int maxDist = -1;
        std::pair<int, int> nextStep = {-1, -1};

        for (int i = 0; i < 4; i++)
        {
            int newX = current.first + dx[i];
            int newY = current.second + dy[i];

            if (isValid(newX, newY, distanceGrid) and not visited[newY][newX])
            {
                if (distanceGrid[newY][newX] > maxDist)
                {
                    maxDist = distanceGrid[newY][newX];
                    nextStep = {newX, newY};
                    moved = true;
                }
            }
        }

        if (moved)
        {
            current = nextStep;
            pathStack.push(current);
            visited[current.second][current.first] = true;
            // std::cout << "(" << current.first << ", " << current.second << ") -> ";
        }
        else
        {
            pathStack.pop();
            if (not pathStack.empty())
            {
                current = pathStack.top();
                // std::cout << "Backtracking to (" << current.first << ", " << current.second << ") -> ";
            }
        }
        ret_vector.push_back(current);
        if (current == goal)
            break;
    }
    return ret_vector;
}

std::vector<std::pair<int, int>> WaveFront::calculate(std::vector<std::vector<int>> grid, std::pair<int, int> start, std::pair<int, int> goal)
{
    rows = grid.size();
    cols = grid.at(0).size();
    auto distanceGrid = distanceTransform(grid, goal);
    return areaCoverageWithBackTracking(distanceGrid, start, goal);
}