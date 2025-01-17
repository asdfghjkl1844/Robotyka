#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <tuple>
#include <fstream>
using namespace std;
const int MAP_SIZE = 20;
const int OBSTACLE = 5;
const int FREE = 0;
const int PATH = 3;
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
        : position(position), gCost(gCost), hCost(hCost), parent(parent) {}

    double fCost() const {
        return gCost + hCost;
    }
};

// Obliczamy heurystyke jako odleglosc euklidesowa
double calculateHeuristic(const Point& a, const Point& b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

bool isPointOnMap(int x, int y) {
    return x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE;
}

// Generujemy sasiadow (gora, dol, lewo, prawo)
vector<tuple<Point, double>> getNeighbors(const Point& point) {
    return {
        {Point(point.x, point.y + 1, ORDER), 1.0},
        {Point(point.x, point.y - 1, ORDER + 1), 1.0},
        {Point(point.x - 1, point.y, ORDER + 2), 1.0},
        {Point(point.x + 1, point.y, ORDER + 3), 1.0},
    };
}

// Zwraca wskaznik do Node w liscie (jesli istnieje)
Node* findNodeInList(const vector<Node*>& list, const Point& point) {
    for (Node* node : list) {
        if (node->position.x == point.x && node->position.y == point.y) {
            return node;
        }
    }
    return nullptr;
}

bool aStar(int map[MAP_SIZE][MAP_SIZE], Point start, Point goal) {
    vector<Node*> openList;
    vector<Node*> closedList;

    // Dodaj startowy punkt do otwartej listy
    openList.push_back(new Node(start, 0.0, calculateHeuristic(start, goal), nullptr));

    while (!openList.empty()) {
        auto currentIt = min_element(openList.begin(), openList.end(),
            [](Node* a, Node* b) {
                if (a->fCost() != b->fCost()) {
                    return a->fCost() < b->fCost();
                }
                if (a->position.z > b->position.z) {
                    return a->position.z > b->position.z;
                }
                return b->position.z > a->position.z;
            });

        Node* currentNode = *currentIt;

        // Jesli dotarlismy do celu
        if (currentNode->position.x == goal.x && currentNode->position.y == goal.y) {
            // Odtworz sciezke
            Node* pathNode = currentNode;
            while (pathNode) {
                map[pathNode->position.y][pathNode->position.x] = PATH;
                pathNode = pathNode->parent;
            }
            return true;
        }

        // Przenies currentNode z otwartej do zamknietej listy
        openList.erase(currentIt);
        closedList.push_back(currentNode);

        // Przetworz sasiadow
        for (const auto& [neighbor, moveCost] : getNeighbors(currentNode->position)) {
            if (!isPointOnMap(neighbor.x, neighbor.y) || map[neighbor.y][neighbor.x] == OBSTACLE) {
                continue;
            }

            ORDER += 4;

            double gCost = currentNode->gCost + moveCost; // Koszt ruchu
            double hCost = calculateHeuristic(neighbor, goal);

            Node* neighborNode = findNodeInList(closedList, neighbor);
            if (neighborNode) continue; // Sasiad jest na zamknietej liscie

            neighborNode = findNodeInList(openList, neighbor);
            if (!neighborNode) {
                // Nowy Node - dodaj do otwartej listy
                openList.push_back(new Node(neighbor, gCost, hCost, currentNode));
            }
            else if (gCost < neighborNode->gCost) {
                // Aktualizuj gCost i rodzica, jesli znaleziono lepsza sciezke
                neighborNode->gCost = gCost;
                neighborNode->parent = currentNode;
            }
        }
    }

    return false; // Brak sciezki
}

void loadMap(int map[MAP_SIZE][MAP_SIZE], const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        cout << "Nie mozna otworzyc pliku: " << filename << endl;
        exit(1);
    }

    for (int i = 0; i < MAP_SIZE; i++) {
        for (int j = 0; j < MAP_SIZE; j++) {
            file >> map[i][j];
        }
    }

    file.close();
}

void printMap(int map[MAP_SIZE][MAP_SIZE]) {
    for (int i = 0; i < MAP_SIZE; i++) {
        for (int j = 0; j < MAP_SIZE; j++) {
            cout << map[i][j] << " ";
        }
        cout << "\n";
    }
}
int main() {
    int map[MAP_SIZE][MAP_SIZE];
    loadMap(map, "grid.txt");

    Point start(0, 19, -1);
    Point goal(19, 0, -1);

    if (aStar(map, start, goal)) {
        cout << "Sciezka zostala znaleziona:\n";
    }
    else {
        cout << "Nie udalo sie znalezc sciezki.\n";
    }

    printMap(map);
    return 0;
}

