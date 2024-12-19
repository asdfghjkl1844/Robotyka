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
const int PATH = 2;

// Współrzędne punktu na mapie
struct Point {
    int x, y;
    Point(int x, int y) : x(x), y(y) {}
};

// Węzeł w algorytmie A*

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

// Obliczamy heurystykę jako odległość euklidesową
double calculateHeuristic(const Point& a, const Point& b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

// Sprawdza, czy punkt znajduje się w granicach mapy
bool isPointOnMap(int x, int y) {
    return x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE;
}

// Generuje sąsiadów (góra, dół, lewo, prawo, ruchy po skosie)
vector<tuple<Point, double>> getNeighbors(const Point& point) {
    return {
        {Point(point.x, point.y + 1), 1.0},       // Góra
        {Point(point.x, point.y - 1), 1.0},       // Dół
        {Point(point.x - 1, point.y), 1.0},       // Lewo
        {Point(point.x + 1, point.y), 1.0},       // Prawo
        //{Point(point.x - 1, point.y + 1), sqrt(2)}, // Lewo-góra
        //{Point(point.x + 1, point.y + 1), sqrt(2)}, // Prawo-góra
        //{Point(point.x - 1, point.y - 1), sqrt(2)}, // Lewo-dół
        //{Point(point.x + 1, point.y - 1), sqrt(2)}  // Prawo-dół
    };
}

// Zwraca wskaźnik do Node w liście (jeśli istnieje)
Node* findNodeInList(const vector<Node*>& list, const Point& point) {
    for (Node* node : list) {
        if (node->position.x == point.x && node->position.y == point.y) {
            return node;
        }
    }
    return nullptr;
}

// Główna funkcja algorytmu A*
bool aStar(int map[MAP_SIZE][MAP_SIZE], Point start, Point goal) {
    vector<Node*> openList;
    vector<Node*> closedList;

    // Dodaj startowy punkt do otwartej listy
    openList.push_back(new Node(start, 0.0, calculateHeuristic(start, goal), nullptr));

    while (!openList.empty()) {
        // Znajdź Node z najniższym kosztem f
        auto currentIt = min_element(openList.begin(), openList.end(),
                                          [](Node* a, Node* b) { return a->fCost() < b->fCost(); });
        Node* currentNode = *currentIt;

        // Jeśli dotarliśmy do celu
        if (currentNode->position.x == goal.x && currentNode->position.y == goal.y) {
            // Odtwórz ścieżkę
            Node* pathNode = currentNode;
            while (pathNode) {
                map[pathNode->position.y][pathNode->position.x] = PATH;
                pathNode = pathNode->parent;
            }
            return true;
        }

        // Przenieś currentNode z otwartej do zamkniętej listy
        openList.erase(currentIt);
        closedList.push_back(currentNode);

        // Przetwórz sąsiadów
        for (const auto& [neighbor, moveCost] : getNeighbors(currentNode->position)) {
            if (!isPointOnMap(neighbor.x, neighbor.y) || map[neighbor.y][neighbor.x] == OBSTACLE) {
                continue;
            }

            double gCost = currentNode->gCost + moveCost; // Koszt ruchu
            double hCost = calculateHeuristic(neighbor, goal);

            Node* neighborNode = findNodeInList(closedList, neighbor);
            if (neighborNode) continue; // Sąsiad jest na zamkniętej liście

            neighborNode = findNodeInList(openList, neighbor);
            if (!neighborNode) {
                // Nowy Node - dodaj do otwartej listy
                openList.push_back(new Node(neighbor, gCost, hCost, currentNode));
            } else if (gCost < neighborNode->gCost) {
                // Aktualizuj gCost i rodzica, jeśli znaleziono lepszą ścieżkę
                neighborNode->gCost = gCost;
                neighborNode->parent = currentNode;
            }
        }
    }

    return false; // Brak ścieżki
}

// Wczytuje mapę z pliku
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

// Wyświetla mapę
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
    loadMap(map, "grid1.txt"); // Wczytaj mapę z pliku grid.txt

    Point start(0, 0);
    Point goal(19, 19);

    if (aStar(map, start, goal)) {
        cout << "Sciezka zostala znaleziona:\n";
    } else {
        cout << "Nie udalo sie znalezc sciezki.\n";
    }

    printMap(map);
    return 0;
}
