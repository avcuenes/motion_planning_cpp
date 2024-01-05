#include <iostream>
#include <vector>
#include <queue>
#include <cmath>

using namespace std;

struct Node {
    int x, y;  // Coordinates of the node
    int cost;  // Cost to reach this node from the start
    int heuristic;  // Heuristic value (estimated cost to goal)

    // Overloaded comparison operator for priority_queue
    bool operator>(const Node& other) const {
        return cost + heuristic > other.cost + other.heuristic;
    }
};

class AStar {
public:
    AStar(vector<vector<int>>& grid, pair<int, int> start, pair<int, int> goal)
        : grid(grid), start(start), goal(goal) {}

    int run() {
        priority_queue<Node, vector<Node>, greater<Node>> pq;

        // Add the start node to the priority queue
        pq.push({start.first, start.second, 0, calculateHeuristic(start)});

        while (!pq.empty()) {
            Node current = pq.top();
            pq.pop();

            // Check if the current node is the goal
            if (current.x == goal.first && current.y == goal.second) {
                cout << "Path found! Cost: " << current.cost << endl;
                return current.cost;
            }

            // Explore neighbors
            for (int i = 0; i < 4; ++i) {
                int nx = current.x + dx[i];
                int ny = current.y + dy[i];

                if (isValid(nx, ny) && grid[nx][ny] == 0) {
                    int newCost = current.cost + 1;
                    int heuristic = calculateHeuristic({nx, ny});

                    // Check if the neighbor is not visited or has a lower cost
                    if (!visited[nx][ny] || newCost < costMap[nx][ny]) {
                        visited[nx][ny] = true;
                        costMap[nx][ny] = newCost;
                        pq.push({nx, ny, newCost, heuristic});
                    }
                }
            }
        }

        cout << "No path found!" << endl;
        return -1;
    }

private:
    vector<vector<int>> grid;
    pair<int, int> start, goal;
    vector<vector<bool>> visited;
    vector<vector<int>> costMap;
    const int dx[4] = {1, 0, -1, 0};
    const int dy[4] = {0, 1, 0, -1};

    bool isValid(int x, int y) {
        return x >= 0 && x < grid.size() && y >= 0 && y < grid[0].size();
    }

    int calculateHeuristic(pair<int, int> current) {
        // Euclidean distance heuristic
        return static_cast<int>(hypot(goal.first - current.first, goal.second - current.second));
    }
};

int main() {
    // Example usage
    vector<vector<int>> grid = {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 1, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 0, 0}
    };

    pair<int, int> start = {0, 0};
    pair<int, int> goal = {4, 4};

    AStar astar(grid, start, goal);
    astar.run();

    return 0;
}
