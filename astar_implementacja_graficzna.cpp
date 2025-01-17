#define SDL_MAIN_HANDLED
#include <SDL2/SDL.h>
#include <vector>
#include <cmath>
#include <tuple>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <algorithm>

const int WINDOW_SIZE = 600;
const int GRID_SIZE = 20;
const int CELL_SIZE = WINDOW_SIZE / GRID_SIZE;

const int EMPTY = 0;
const int OBSTACLE = 5;
const int PATH = 2;
const int START = 3;
const int END = 4;
const int VISITED = 6;
int ORDER = 0;

struct Point {
    int x, y, z;
    Point(int x, int y, int z) : x(x), y(y), z(z) {}
};

struct Node {
    Point position;
    double gCost, hCost;
    Node* parent;

    Node(Point position, double gCost, double hCost, Node* parent)
        : position(position), gCost(gCost), hCost(hCost), parent(parent) {
    }

    double fCost() const {
        return gCost + hCost;
    }
};

double calculateHeuristic(const Point& a, const Point& b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

bool isPointOnMap(int x, int y) {
    return x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE;
}

std::vector<std::tuple<Point, double>> getNeighbors(const Point& point) {
    return {
        {Point(point.x, point.y + 1, ORDER), 1.0},
        {Point(point.x, point.y - 1, ORDER + 1), 1.0},
        {Point(point.x - 1, point.y, ORDER + 2), 1.0},
        {Point(point.x + 1, point.y, ORDER + 3), 1.0},
    };
}

Node* findNodeInList(const std::vector<Node*>& list, const Point& point) {
    for (Node* node : list) {
        if (node->position.x == point.x && node->position.y == point.y) {
            return node;
        }
    }
    return nullptr;
}

void drawGrid(SDL_Renderer* renderer, int grid[GRID_SIZE][GRID_SIZE]) {
    for (int y = 0; y < GRID_SIZE; ++y) {
        for (int x = 0; x < GRID_SIZE; ++x) {
            SDL_Rect cell = { x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE };

            switch (grid[y][x]) {
            case OBSTACLE:
                SDL_SetRenderDrawColor(renderer, 255, 105, 180, 255);
                break;
            case PATH:
                SDL_SetRenderDrawColor(renderer, 199, 21, 133, 255);
                break;
            case START:
                SDL_SetRenderDrawColor(renderer, 255, 20, 147, 255);
                break;
            case END:
                SDL_SetRenderDrawColor(renderer, 219, 112, 147, 255);
                break;
            case VISITED:
                SDL_SetRenderDrawColor(renderer, 255, 192, 203, 255);
                break;
            default:
                SDL_SetRenderDrawColor(renderer, 255, 240, 245, 255);
                break;
            }

            SDL_RenderFillRect(renderer, &cell);
            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            SDL_RenderDrawRect(renderer, &cell);
        }
    }
}

bool aStarWithVisualization(SDL_Renderer* renderer, int grid[GRID_SIZE][GRID_SIZE], Point start, Point goal) {
    std::vector<Node*> openList;
    std::vector<Node*> closedList;

    openList.push_back(new Node(start, 0.0, calculateHeuristic(start, goal), nullptr));

    while (!openList.empty()) {
        auto currentIt = min_element(openList.begin(), openList.end(),
            [](Node* a, Node* b) {
                if (a->fCost() != b->fCost()){
                    return a->fCost() < b->fCost();
                }
                if (a->position.z > b->position.z) {
                    return a->position.z > b->position.z;
                }
                return b->position.z > a->position.z;
            });

        Node* currentNode = *currentIt;

        if (currentNode->position.x == goal.x && currentNode->position.y == goal.y) {
            Node* pathNode = currentNode;
            while (pathNode) {
                if (grid[pathNode->position.y][pathNode->position.x] != START && grid[pathNode->position.y][pathNode->position.x] != END) {
                    grid[pathNode->position.y][pathNode->position.x] = PATH;
                }
                pathNode = pathNode->parent;
            }
            return true;
        }

        openList.erase(currentIt);
        closedList.push_back(currentNode);

        for (const auto& [neighbor, moveCost] : getNeighbors(currentNode->position)) {
            if (!isPointOnMap(neighbor.x, neighbor.y) || grid[neighbor.y][neighbor.x] == OBSTACLE) {
                continue;
            }

            ORDER += 4;

            double gCost = currentNode->gCost + moveCost;
            double hCost = calculateHeuristic(neighbor, goal);

            Node* neighborNode = findNodeInList(closedList, neighbor);
            if (neighborNode) continue;

            neighborNode = findNodeInList(openList, neighbor);
            if (!neighborNode) {
                openList.push_back(new Node(neighbor, gCost, hCost, currentNode));
            }
            else if (gCost < neighborNode->gCost) {
                neighborNode->gCost = gCost;
                neighborNode->parent = currentNode;
            }

            grid[neighbor.y][neighbor.x] = VISITED;
        }

        drawGrid(renderer, grid);
        SDL_RenderPresent(renderer);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return false;
}

bool loadGridFromFile(const std::string& filename, int grid[GRID_SIZE][GRID_SIZE]) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Nie mozna otworzyc pliku: " << filename << std::endl;
        return false;
    }

    for (int y = 0; y < GRID_SIZE; ++y) {
        for (int x = 0; x < GRID_SIZE; ++x) {
            file >> grid[y][x];
        }
    }

    file.close();
    return true;
}

int main(int argc, char* argv[]) {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* window = SDL_CreateWindow("A* Visualization", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WINDOW_SIZE, WINDOW_SIZE, SDL_WINDOW_SHOWN);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    int grid[GRID_SIZE][GRID_SIZE];
    if (!loadGridFromFile("grid.txt", grid)) {
        return -1;
    }

    Point start(0, 19, -1);
    Point goal(19, 0, -1);
    grid[start.y][start.x] = START;
    grid[goal.y][goal.x] = END;

    if (!aStarWithVisualization(renderer, grid, start, goal)) {
        std::cerr << "Nie znaleziono sciezki!" << std::endl;
    }

    SDL_Event event;
    bool running = true;
    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
        }

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        drawGrid(renderer, grid);
        SDL_RenderPresent(renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
