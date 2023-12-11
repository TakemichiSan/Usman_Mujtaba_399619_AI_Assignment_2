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
#include <unordered_set>

using namespace std;

struct PuzzleState {
    vector<vector<int>> board;
    int heuristic;  // Only heuristic is used for greedy search
    int moves;
    PuzzleState* parent;

    PuzzleState(const vector<vector<int>>& b, int h, int m, PuzzleState* p)
        : board(b), heuristic(h), moves(m), parent(p) {}

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
        return a.heuristic > b.heuristic;  // Compare using only heuristic for greedy search
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

int calculateManhattan(const vector<vector<int>>& board, int g, int h, PuzzleState* parent) {
    int distance = 0;
    for (int num = 1; num <= 8; ++num) {
        pair<int, int> currentPos = findNumber(board, num);
        pair<int, int> goalPos = {(num - 1) / 3, (num - 1) % 3};
        distance += abs(currentPos.first - goalPos.first) + abs(currentPos.second - goalPos.second);
    }
    return g + h;
}

bool isValidMove(int i, int j) {
    return i >= 0 && i < 3 && j >= 0 && j < 3;
}

vector<PuzzleState> generateSuccessors(const PuzzleState& state);

void greedySearch(const vector<vector<int>>& initialBoard);

int main() {
    vector<vector<int>> initialBoard = {{1, 2, 3}, {0, 4, 6}, {7, 5, 8}};

    cout << "Initial state:" << endl;
    printBoard(initialBoard);

    greedySearch(initialBoard);

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

            int newHeuristic = calculateManhattan(newBoard, state.moves + 1, 0, nullptr);
            int newMoves = state.moves + 1;

            successors.emplace_back(newBoard, newHeuristic, newMoves, nullptr);
        }
    }

    return successors;
}

void greedySearch(const vector<vector<int>>& initialBoard) {
    PuzzleState initialState{initialBoard, 0, 0, nullptr};

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
