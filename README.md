# Usman_Mujtaba_399619_AI_Assignment_2
# Manhattan: 
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>

using namespace std;

struct PuzzleState {
    vector<vector<int>> board;
    int cost;
    int heuristic;
    int moves;
    const PuzzleState* parent;  // Change PuzzleState* to const PuzzleState*

    PuzzleState(const vector<vector<int>>& b, int g, int h, int m, const PuzzleState* p)
        : board(b), cost(g), heuristic(h), moves(m), parent(p) {}

    bool operator==(const PuzzleState& other) const {
        return board == other.board;
    }
};

struct PuzzleStateHash {
    size_t operator()(const PuzzleState& state) const {
        size_t hash = 0;
        for (const auto& row : state.board) {
            for (int num : row) {
                hash ^= hash << 6 ^ hash >> 2 ^ size_t(num) + 0x9e3779b9 + (hash << 14) + (hash >> 7);
            }
        }
        return hash;
    }
};

struct PuzzleStateComparator {
    bool operator()(const PuzzleState& a, const PuzzleState& b) const {
        return a.cost + a.heuristic > b.cost + b.heuristic;
    }
};

void printBoard(const vector<vector<int>>& board) {
    for (const auto& row : board) {
        for (int num : row) {
            cout << num << " ";
        }
        cout << endl;
    }
    cout << endl;
}

pair<int, int> findNumber(const vector<vector<int>>& board, int number) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (board[i][j] == number) {
                return {i, j};
            }
        }
    }
    return {-1, -1};
}

int calculateManhattan(const PuzzleState& state) {
    int distance = 0;
    for (int num = 1; num <= 8; ++num) {
        pair<int, int> currentPos = findNumber(state.board, num);
        pair<int, int> goalPos = {(num - 1) / 3, (num - 1) % 3};
        distance += abs(currentPos.first - goalPos.first) + abs(currentPos.second - goalPos.second);
    }
    return distance;
}

bool isValidMove(int i, int j) {
    return i >= 0 && i < 3 && j >= 0 && j < 3;
}

vector<PuzzleState> generateSuccessors(const PuzzleState& state);

void aStarSearch(const vector<vector<int>>& initialBoard);

int main() {
    vector<vector<int>> initialBoard = {{1, 2, 3}, {0, 4, 6}, {7, 5, 8}};

    cout << "Initial state:" << endl;
    printBoard(initialBoard);

    aStarSearch(initialBoard);

    return 0;
}

vector<PuzzleState> generateSuccessors(const PuzzleState& state) {
    vector<PuzzleState> successors;
    pair<int, int> emptyPos = findNumber(state.board, 0);

    const int moves[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    for (const auto& move : moves) {
        int newI = emptyPos.first + move[0];
        int newJ = emptyPos.second + move[1];

        if (isValidMove(newI, newJ)) {
            vector<vector<int>> newBoard = state.board;
            swap(newBoard[emptyPos.first][emptyPos.second], newBoard[newI][newJ]);

            int newCost = state.cost + 1;
            int newHeuristic = calculateManhattan({newBoard, 0, 0, 0, nullptr});
            int newMoves = state.moves + 1;

            successors.emplace_back(newBoard, newCost, newHeuristic, newMoves, &state);
        }
    }

    return successors;
}

void aStarSearch(const vector<vector<int>>& initialBoard) {
    PuzzleState initialState{initialBoard, 0, calculateManhattan({initialBoard, 0, 0, 0, nullptr}), 0, nullptr};

    priority_queue<PuzzleState, vector<PuzzleState>, PuzzleStateComparator> openList;

    unordered_set<PuzzleState, PuzzleStateHash> closedList;

    openList.push(initialState);

    while (!openList.empty()) {
        PuzzleState currentState = openList.top();
        openList.pop();

        if (currentState.heuristic == 0) {
            cout << "Goal state reached in " << currentState.moves << " moves." << endl;
            cout << "Solution:" << endl;

            while (currentState.parent != nullptr) {
                printBoard(currentState.board);
                currentState = *currentState.parent;
            }
            printBoard(initialState.board);
            return;
        }

        closedList.insert(currentState);

        vector<PuzzleState> successors = generateSuccessors(currentState);

        for (const PuzzleState& successor : successors) {
            if (closedList.find(successor) == closedList.end()) {
                openList.push(successor);
            }
        }
    }

    cout << "Goal state not reachable." << endl;
}

# Greedy 

#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <algorithm>

using namespace std;

// Structure to represent a state of the puzzle
struct PuzzleState {
    vector<vector<int>> board;
    int cost;

    bool operator<(const PuzzleState& other) const {
        return cost > other.cost; // Use greater for min-heap
    }
};

// Function to print the puzzle board
void printBoard(const vector<vector<int>>& board) {
    for (const auto& row : board) {
        for (int value : row) {
            cout << value << " ";
        }
        cout << endl;
    }
    cout << endl;
}

// Function to check if two puzzle states are equal
bool isEqual(const PuzzleState& state1, const PuzzleState& state2) {
    return state1.board == state2.board;
}

// Function to check if a puzzle state is valid
bool isValid(int i, int j) {
    return i >= 0 && i < 3 && j >= 0 && j < 3;
}

// Function to find the heuristic cost (Manhattan distance)
int heuristic(const vector<vector<int>>& current, const vector<vector<int>>& goal) {
    int h = 0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            int value = current[i][j];
            if (value != 0) {
                auto goalPos = find_if(goal.begin(), goal.end(), [value](const vector<int>& row) {
                    return find(row.begin(), row.end(), value) != row.end();
                });
                h += abs(i - distance(goal.begin(), goalPos)) + abs(j - distance(goalPos->begin(), find(goalPos->begin(), goalPos->end(), value)));
            }
        }
    }
    return h;
}

// Function to perform Greedy Best-First Search on the 8-puzzle problem
void greedySearch(const vector<vector<int>>& start, const vector<vector<int>>& goal) {
    priority_queue<PuzzleState> openSet;
    map<vector<vector<int>>, bool> visited;

    PuzzleState initialState{start, heuristic(start, goal)};
    openSet.push(initialState);

    while (!openSet.empty()) {
        PuzzleState current = openSet.top();
        openSet.pop();

        if (isEqual(current, {goal, 0})) {
            cout << "Goal State Reached!" << endl;
            printBoard(current.board);
            return;
        }

        if (!visited[current.board]) {
            visited[current.board] = true;

            int i, j;
            // Find the position of the empty tile
            for (i = 0; i < 3; ++i) {
                for (j = 0; j < 3; ++j) {
                    if (current.board[i][j] == 0) {
                        break;
                    }
                }
                if (j < 3) {
                    break;
                }
            }

            // Define possible moves (up, down, left, right)
            int moves[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

            for (int k = 0; k < 4; ++k) {
                int ni = i + moves[k][0];
                int nj = j + moves[k][1];

                if (isValid(ni, nj)) {
                    // Create a new state by swapping the empty tile with the adjacent tile
                    vector<vector<int>> nextState = current.board;
                    swap(nextState[i][j], nextState[ni][nj]);

                    PuzzleState nextStateInfo{nextState, heuristic(nextState, goal)};
                    openSet.push(nextStateInfo);
                }
            }
        }
    }

    cout << "Goal State Not Reachable!" << endl;
}

int main() {
    vector<vector<int>> start = {{1, 2, 3}, {4, 0, 5}, {6, 7, 8}};
    vector<vector<int>> goal = {{1, 2, 3}, {4, 5, 6}, {7, 8, 0}};

    cout << "Starting State:" << endl;
    printBoard(start);

    cout << "Goal State:" << endl;
    printBoard(goal);

    cout << "Performing Greedy Best-First Search:" << endl;
    greedySearch(start, goal);

    return 0;
}
