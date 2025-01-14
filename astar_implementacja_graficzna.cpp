#define SDL_MAIN_HANDLED
#include <SDL.h>
#include <vector>
#include <queue>
#include <tuple>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <thread> 

const int WINDOW_SIZE = 600;
const int GRID_SIZE = 20;
const int CELL_SIZE = WINDOW_SIZE / GRID_SIZE;

const int EMPTY = 0;
const int OBSTACLE = 5;
const int PATH = 2;
const int START = 3;
const int END = 4;
const int VISITED = 6; 

struct Node {
    int x, y, cost, priority;
    bool operator<(const Node& other) const { return priority > other.priority; }
};

const int DX[4] = { 1, -1, 0, 0 };
const int DY[4] = { 0, 0, 1, -1 };

bool isPointOnMap(int x, int y) {
    return x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE;
}

int heuristic(int x1, int y1, int x2, int y2) {
    return static_cast<int>(sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) * 10);
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

// Funkcja A* z animacją
std::vector<std::pair<int, int>> aStarWithAnimation(SDL_Renderer* renderer, int grid[GRID_SIZE][GRID_SIZE], int startX, int startY, int endX, int endY) {
    std::priority_queue<Node> openList;
    int cost[GRID_SIZE][GRID_SIZE];
    std::pair<int, int> parent[GRID_SIZE][GRID_SIZE];
    bool visited[GRID_SIZE][GRID_SIZE] = { false };

    for (int y = 0; y < GRID_SIZE; ++y) {
        for (int x = 0; x < GRID_SIZE; ++x) {
            cost[y][x] = INT_MAX;
        }
    }

    openList.push({ startX, startY, 0, heuristic(startX, startY, endX, endY) });
    cost[startY][startX] = 0;

    while (!openList.empty()) {
        Node current = openList.top();
        openList.pop();

        if (visited[current.y][current.x]) continue;
        visited[current.y][current.x] = true;

        if (current.x == endX && current.y == endY) {
            std::vector<std::pair<int, int>> path;
            int cx = endX, cy = endY;
            while (!(cx == startX && cy == startY)) {
                path.push_back({ cx, cy });
                std::tie(cx, cy) = parent[cy][cx];
            }
            path.push_back({ startX, startY });
            return path;
        }

        // Oznacz aktualny węzeł jako odwiedzony w siatce
        if (grid[current.y][current.x] != START && grid[current.y][current.x] != END) {
            grid[current.y][current.x] = VISITED;
        }

        // Rysowanie animacji
        drawGrid(renderer, grid);
        SDL_RenderPresent(renderer);
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Opóźnienie (50ms)

        for (int i = 0; i < 4; ++i) {
            int nx = current.x + DX[i];
            int ny = current.y + DY[i];

            if (isPointOnMap(nx, ny) && grid[ny][nx] != OBSTACLE && !visited[ny][nx]) {
                int moveCost = cost[current.y][current.x] + 10;
                if (moveCost < cost[ny][nx]) {
                    cost[ny][nx] = moveCost;
                    parent[ny][nx] = { current.x, current.y };
                    openList.push({ nx, ny, moveCost, moveCost + heuristic(nx, ny, endX, endY) });
                }
            }
        }
    }

    return {};
}

bool loadGridFromFile(const std::string& filename, int grid[GRID_SIZE][GRID_SIZE]) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Nie można otworzyć pliku: " << filename << std::endl;
        return false;
    }

    for (int y = 0; y < GRID_SIZE; ++y) {
        for (int x = 0; x < GRID_SIZE; ++x) {
            file >> grid[y][x];
            if (file.fail()) {
                std::cerr << "Błąd podczas odczytu pliku: " << filename << std::endl;
                return false;
            }
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

    if (!loadGridFromFile("grid1.txt", grid)) {
        return -1;
    }

    grid[19][0] = START;
    grid[0][19] = END;

    auto path = aStarWithAnimation(renderer, grid, 0, 19, 19, 0);

    for (auto [x, y] : path) {
        if (grid[y][x] != START && grid[y][x] != END) {
            grid[y][x] = PATH;
        }
    }

    bool running = true;
    SDL_Event event;

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
