#include <iostream>
#include <vector>
#include <algorithm>

struct Point {
    int x, y;

    Point(int x, int y) : x(x), y(y) {}
};

class AStar {
public:
    AStar(const std::vector<std::vector<int>>& grid, Point start, Point goal)
        : grid(grid), start(start), goal(goal) {}

    std::vector<Point> findPath(int maxIterations) {
    std::vector<Point> path;
    std::vector<Node> openSet;
    std::vector<std::vector<bool>> closedSet(grid.size(), std::vector<bool>(grid[0].size(), false));

    openSet.push_back(Node(start, 0, calculateHeuristic(start, goal), nullptr));
    std::make_heap(openSet.begin(), openSet.end(), std::greater<>());  // Make the vector a min-heap

    std::vector<Point> directions = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}};

    int iterations = 0;

    while (!openSet.empty() && iterations < maxIterations) {
        std::pop_heap(openSet.begin(), openSet.end(), std::greater<>());  // Move the minimum element to the end
        Node current = openSet.back();  // Get the minimum element
        openSet.pop_back();  // Remove the minimum element

        if (current.position.x == goal.x && current.position.y == goal.y) {
            // Goal reached, reconstruct the path

            for (int i = 0; i < iterations; i++) {
                path.push_back(current.position);
                current = *current.parent;
            }

            // Add the start position
            path.push_back(start);

            std::reverse(path.begin(), path.end());
            return path;
        }

        closedSet[current.position.x][current.position.y] = true;

        for (const auto& direction : directions) {
            int nextX = current.position.x + direction.x;
            int nextY = current.position.y + direction.y;

            if (isValidPosition(nextX, nextY) && !closedSet[nextX][nextY]) {
                int newG = current.g + 1;
                int newH = calculateHeuristic({nextX, nextY}, goal);
                int newF = newG + newH;

                // Check if the node is already in the open set
                auto it = std::find_if(openSet.begin(), openSet.end(),
                                       [&nextX, &nextY](const Node& node) {
                                           return node.position.x == nextX && node.position.y == nextY;
                                       });

                if (it != openSet.end()) {
                    // Node is already in the open set, update it if necessary
                    if (newF < it->f) {
                        it->g = newG;
                        it->h = newH;
                        it->f = newF;
                        it->parent = &current;
                        std::make_heap(openSet.begin(), openSet.end(), std::greater<>());  // Re-heapify
                    }
                } else {
                    // Node is not in the open set, add it
                    openSet.push_back(Node({nextX, nextY}, newG, newH, &current));
                    std::push_heap(openSet.begin(), openSet.end(), std::greater<>());  // Maintain heap property
                }
            }
        }

        iterations++;
    }

    // If the openSet is empty and the goal is not reached, there is no path
    return {};
}


private:
    struct Node {
        Point position;
        int g;
        int h;
        int f;
        Node* parent;

        Node(Point position, int g, int h, Node* parent)
            : position(position), g(g), h(h), f(g + h), parent(parent) {}

        // Comparison function for sorting in priority queue
        bool operator>(const Node& other) const {
            return f > other.f;
        }
    };

    std::vector<std::vector<int>> grid;
    Point start;
    Point goal;

    int calculateHeuristic(const Point& current, const Point& goal) const {
        return std::abs(current.x - goal.x) + std::abs(current.y - goal.y);
    }

    bool isValidPosition(int x, int y) const {
        return x >= 0 && x < grid.size() && y >= 0 && y < grid[0].size() && grid[x][y] == 0;
    }
};

int main() {
    // Example usage
    std::vector<std::vector<int>> grid = {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 1, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 0, 0}
    };

    Point start = {0, 0};
    Point goal = {4, 4};

    AStar astar(grid, start, goal);

    // Set a maximum iteration limit (e.g., 1000)
    int maxIterations = 1000;

    std::vector<Point> path = astar.findPath(maxIterations);

    // Print the result
    if (path.empty()) {
        std::cout << "No path found within the maximum iterations." << std::endl;
    } else {
        std::cout << "Path found:" << std::endl;
        for (const auto& point : path) {
            std::cout << "(" << point.x << ", " << point.y << ") ";
        }
        std::cout << std::endl;
    }

    return 0;
}
