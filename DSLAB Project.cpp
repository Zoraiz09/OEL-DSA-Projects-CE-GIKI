#include <iostream>
#include <vector>
#include <queue>
#include <list>
#include <limits>
#include <unordered_map>
#include <set>
#include <algorithm>
#include <stack>
#include <cstdlib>
#include <chrono>
#include <iomanip>
#include <numeric>
#include <forward_list>

using namespace std;

// Enhanced Traffic Signal Structure with more detailed state representation
struct TrafficSignal {
    string signalId;
    int greenDuration;
    int redDuration;
    int yellowDuration;
    bool isEmergencyMode;
    vector<string> connectedRoads;
    enum State { RED, YELLOW, GREEN } currentState;
    chrono::system_clock::time_point lastStateChange;
    int x, y;
    int timeRemaining; // Added time remaining for current state
    vector<int> historicalDurations; // Track historical durations

    TrafficSignal() : signalId(""), greenDuration(30), redDuration(30),
                      yellowDuration(5), isEmergencyMode(false),
                      currentState(RED), x(0), y(0), timeRemaining(30) {
        lastStateChange = chrono::system_clock::now();
    }

    TrafficSignal(const string& id) : signalId(id), greenDuration(30), redDuration(30),
                                      yellowDuration(5), isEmergencyMode(false),
                                      currentState(RED), x(0), y(0), timeRemaining(30) {
        lastStateChange = chrono::system_clock::now();
    }
};

// Enhanced Road Structure with more detailed traffic metrics
struct Road {
    string roadId;
    int trafficDensity;
    int speed;
    int capacity;
    bool hasEmergencyVehicle;
    int congestionLevel;
    int accidentStatus;
    vector<pair<int, int>> historicalData;
    int startX, startY, endX, endY;
    string routeName;
    int estimatedTravelTime;
    vector<int> hourlyTrafficVolume; // Added hourly traffic volume tracking
    double averageSpeed; // Added average speed tracking
    int peakHourTraffic; // Added peak hour traffic volume

    Road() : roadId(""), trafficDensity(0), speed(60), capacity(100),
             hasEmergencyVehicle(false), congestionLevel(0), accidentStatus(0),
             startX(0), startY(0), endX(0), endY(0), routeName(""),
             estimatedTravelTime(0), averageSpeed(60), peakHourTraffic(0) {
        hourlyTrafficVolume.resize(24, 0);
    }

    Road(const string& id) : roadId(id), trafficDensity(0), speed(60), capacity(100),
                             hasEmergencyVehicle(false), congestionLevel(0), accidentStatus(0),
                             startX(0), startY(0), endX(0), endY(0),
                             routeName(id), estimatedTravelTime(0),
                             averageSpeed(60), peakHourTraffic(0) {
        hourlyTrafficVolume.resize(24, 0);
    }

    string getTrafficStatus() const {
        if (trafficDensity >= 80) return "Heavy";
        if (trafficDensity >= 40) return "Moderate";
        return "Light";
    }

    // Added analytics methods
    double getAverageTrafficDensity() const {
        if (historicalData.empty()) return 0;
        double sum = 0;
        for (const auto& data : historicalData) {
            sum += data.second;
        }
        return sum / historicalData.size();
    }

    int getPeakTrafficDensity() const {
        int peak = 0;
        for (const auto& data : historicalData) {
            peak = max(peak, data.second);
        }
        return peak;
    }
};

// Forward declaration
struct Node;

// Define Edge first
struct Edge {
    Node* source;
    Node* destination;
    int weight;
    Edge* next;

    Edge(Node* src, Node* dest, int w) :
        source(src), destination(dest), weight(w), next(nullptr) {}
};

// Then define Node
struct Node {
    string id;
    Node* next;
    vector<Edge*> edges;

    Node(string id) : id(id), next(nullptr) {}
};

class AdjacencyList {
private:
    Node* head;
    int size;

public:
    AdjacencyList() : head(nullptr), size(0) {}

    void addNode(string id) {
        Node* newNode = new Node(id);
        if (!head) {
            head = newNode;
        } else {
            newNode->next = head;
            head = newNode;
        }
        size++;
    }

    Node* findNode(string id) {
        Node* current = head;
        while (current) {
            if (current->id == id) return current;
            current = current->next;
        }
        return nullptr;
    }
};

class TrafficManagementSystem {
private:
    AdjacencyList graph;
    stack<Node*> pathStack;

    // Replace maps with linked lists
    struct SignalList {
        TrafficSignal data;
        SignalList* next;
    } *signalHead;

    struct RoadList {
        Road data;
        RoadList* next;
    } *roadHead;

    // Add intersection structure
    struct Intersection {
        string id;
        vector<string> connectedRoads;
        TrafficSignal signal;
    };

    unordered_map<string, Intersection> intersections;

    // Modified addRoad to handle intersections
    void addIntersection(const string& roadId1, const string& roadId2) {
        string intersectionId = roadId1 + "_" + roadId2;
        Intersection& intersection = intersections[intersectionId];
        intersection.id = intersectionId;
        intersection.connectedRoads = {roadId1, roadId2};
        intersection.signal = TrafficSignal(intersectionId);
    }

public:
    TrafficManagementSystem(int v) : signalHead(nullptr), roadHead(nullptr) {
        const string streets[] = {"F8", "F6", "E5", "I8", "F7"};
        for (const string& street : streets) {
            graph.addNode(street);
            addSignal(street);
        }
    }

    void addSignal(const string& id) {
        SignalList* newSignal = new SignalList();
        newSignal->data = TrafficSignal(id);
        newSignal->next = signalHead;
        signalHead = newSignal;
    }

    void addRoad(const string& from, const string& to, int weight) {
        Node* fromNode = graph.findNode(from);
        Node* toNode = graph.findNode(to);

        if (!fromNode || !toNode) return;

        Edge* newEdge = new Edge(fromNode, toNode, weight);
        fromNode->edges.push_back(newEdge);
        toNode->edges.push_back(new Edge(toNode, fromNode, weight));

        // Add to road list
        RoadList* newRoad = new RoadList();
        newRoad->data = Road(from + "-" + to);
        newRoad->next = roadHead;
        roadHead = newRoad;

        // Update connected roads for signals
        SignalList* fromSignal = findSignal(from);
        SignalList* toSignal = findSignal(to);

        if (fromSignal) {
            fromSignal->data.connectedRoads.push_back(from + "-" + to);
        }
        if (toSignal) {
            toSignal->data.connectedRoads.push_back(from + "-" + to);
        }

        // Add intersection
        addIntersection(from, to);
    }

    void displaySystemStatus() {
        // Display Current Traffic Situation
        cout << "\nCurrent Traffic Situation:\n";
        cout << setfill('-') << setw(100) << "-" << setfill(' ') << endl;
        cout << setw(20) << left << "Route Name"
             << setw(15) << "Status"
             << setw(15) << "Travel Time"
             << setw(15) << "Density"
             << setw(15) << "Avg Density"
             << setw(20) << "Peak Density" << endl;
        cout << setfill('-') << setw(100) << "-" << setfill(' ') << endl;

        RoadList* currentRoad = roadHead;
        while (currentRoad) {
            cout << setw(20) << left << currentRoad->data.routeName
                 << setw(15) << currentRoad->data.getTrafficStatus()
                 << setw(15) << currentRoad->data.estimatedTravelTime
                 << setw(15) << currentRoad->data.trafficDensity
                 << setw(15) << fixed << setprecision(2) << currentRoad->data.getAverageTrafficDensity()
                 << setw(20) << currentRoad->data.getPeakTrafficDensity() << endl;
            currentRoad = currentRoad->next;
        }

        // Update Signal Analytics section
        cout << "\nTraffic Signal Analytics:\n";
        cout << setfill('-') << setw(100) << "-" << setfill(' ') << endl;
        cout << setw(15) << left << "Signal ID"
             << setw(15) << "State"
             << setw(15) << "Remaining"
             << setw(20) << "Connected Roads"
             << setw(15) << "Efficiency"
             << setw(20) << "Load Balance" << endl;
        cout << setfill('-') << setw(100) << "-" << setfill(' ') << endl;

        SignalList* currentSignal = signalHead;
        while (currentSignal) {
            string state;
            string color;
            switch (currentSignal->data.currentState) {
                case TrafficSignal::RED:
                    state = "RED";
                    color = "\033[1;31m";
                    break;
                case TrafficSignal::YELLOW:
                    state = "YELLOW";
                    color = "\033[1;33m";
                    break;
                case TrafficSignal::GREEN:
                    state = "GREEN";
                    color = "\033[1;32m";
                    break;
            }

            // Calculate signal efficiency (example metric)
            double efficiency = (currentSignal->data.greenDuration * 100.0) /
                                  (currentSignal->data.greenDuration + currentSignal->data.redDuration);

            // Calculate load balance across connected roads
            double avgLoad = 0;
            for (const string& roadId : currentSignal->data.connectedRoads) {
                RoadList* road = findRoad(roadId);
                if (road) {
                    avgLoad += road->data.trafficDensity;
                }
            }
            avgLoad = currentSignal->data.connectedRoads.empty() ? 0 :
                     avgLoad / currentSignal->data.connectedRoads.size();
            cout << setw(15) << left << currentSignal->data.signalId
                 << setw(30) << (color + state + "\033[0m")
                 << setw(15) << currentSignal->data.timeRemaining
                 << setw(20) << currentSignal->data.connectedRoads.size()
                 << setw(15) << fixed << setprecision(2) << efficiency
                 << setw(20) << avgLoad << endl;
            currentSignal = currentSignal->next;
        }
        cout << endl;

        // Display Network Analytics
        cout << "\nNetwork Analytics Summary:\n";
        cout << setfill('-') << setw(50) << "-" << setfill(' ') << endl;

        // Calculate network-wide metrics
        double totalDensity = 0;
        int maxCongestion = 0;
        RoadList* road = roadHead;
        while (road) {
            totalDensity += road->data.trafficDensity;
            maxCongestion = max(maxCongestion, road->data.congestionLevel);
            road = road->next;
        }
        double avgDensity = roadHead ? totalDensity / countRoads() : 0;

        cout << "Average Network Density: " << fixed << setprecision(2)
             << avgDensity << "%\n";
        cout << "Maximum Congestion Level: " << maxCongestion << "\n";
        cout << "Total Active Signals: " << countSignals() << "\n";
        cout << "Total Monitored Roads: " << countRoads() << "\n";
        cout << endl;
    }

    vector<string> findOptimalRoute(const string& start, const string& end) {
        Node* startNode = graph.findNode(start);
        Node* endNode = graph.findNode(end);
        
        if (!startNode || !endNode) return vector<string>();
        
        // Initialize data structures for Dijkstra's algorithm
        unordered_map<string, double> distances;
        unordered_map<string, string> previous;
        set<pair<double, string>> pq;  // Priority queue {distance, nodeId}
        
        // Initialize distances to infinity
        Node* temp = graph.findNode(start);
        while (temp) {
            distances[temp->id] = numeric_limits<double>::infinity();
            temp = temp->next;
        }
        
        // Set start distance to 0
        distances[start] = 0;
        pq.insert({0, start});
        
        while (!pq.empty()) {
            string currentId = pq.begin()->second;
            pq.erase(pq.begin());
            
            if (currentId == end) break;
            
            Node* currentNode = graph.findNode(currentId);
            if (!currentNode) continue;
            
            for (Edge* edge : currentNode->edges) {
                string nextId = edge->destination->id;
                // Calculate cost based on edge weight and traffic density
                RoadList* road = findRoad(currentId + "-" + nextId);
                double edgeCost = edge->weight;
                if (road) {
                    edgeCost *= (1 + road->data.trafficDensity / 100.0);
                }
                
                double newDist = distances[currentId] + edgeCost;
                
                if (newDist < distances[nextId]) {
                    pq.erase({distances[nextId], nextId});
                    distances[nextId] = newDist;
                    previous[nextId] = currentId;
                    pq.insert({newDist, nextId});
                }
            }
        }
        
        // Reconstruct path
        vector<string> path;
        string currentId = end;
        
        // Check if a path exists
        if (distances[end] == numeric_limits<double>::infinity()) {
            return path;  // Return empty path if no route found
        }
        
        // Build the path from end to start
        while (currentId != start) {
            path.push_back(currentId);
            if (previous.find(currentId) == previous.end()) break;
            currentId = previous[currentId];
        }
        path.push_back(start);
        
        // Reverse to get path from start to end
        reverse(path.begin(), path.end());
        
        return path;
    }

    bool hasSignal(const string& streetId) const {
        SignalList* current = signalHead;
        while (current) {
            if (current->data.signalId == streetId) return true;
            current = current->next;
        }
        return false;
    }

    void updateSignalState(const string& streetId, TrafficSignal::State newState) {
        SignalList* signal = findSignal(streetId);
        if (signal) {
            signal->data.currentState = newState;
            signal->data.lastStateChange = chrono::system_clock::now();
        }
    }

    void simulateRealTimeTraffic() {
        const int SIMULATION_ROUNDS = 2;
        for (int i = 0; i < SIMULATION_ROUNDS; i++) {
            cout << "\nSimulation Round " << (i + 1) << endl;

            // Initialize road data for each round
            if (i == 0) {
                // Round 1: Morning peak hour scenario
                initializeRoadData("F8-F6", 75, 50, "Heavy morning traffic from F8 to F6");
                initializeRoadData("F6-E5", 60, 45, "Moderate traffic to E5");
                initializeRoadData("E5-I8", 40, 55, "Normal flow to I8");
                initializeRoadData("I8-F7", 85, 35, "Heavy traffic towards F7");
                initializeRoadData("F8-F7", 65, 40, "Moderate direct route");
                initializeRoadData("F8-E5", 45, 50, "Alternative route traffic");
                initializeRoadData("F6-F7", 70, 45, "Busy connector road");
                initializeRoadData("F6-I8", 55, 60, "Medium traffic flow");
            } else {
                // Round 2: Evening peak hour scenario
                initializeRoadData("F8-F6", 45, 60, "Evening return traffic");
                initializeRoadData("F6-E5", 80, 35, "Heavy evening traffic");
                initializeRoadData("E5-I8", 65, 45, "Increased evening flow");
                initializeRoadData("I8-F7", 50, 55, "Moderate evening traffic");
                initializeRoadData("F8-F7", 90, 30, "Peak return route");
                initializeRoadData("F8-E5", 70, 40, "Busy alternative path");
                initializeRoadData("F6-F7", 55, 50, "Evening connector flow");
                initializeRoadData("F6-I8", 40, 65, "Reduced evening traffic");
            }

            // Update each signal based on its specific connected roads
            SignalList* currentSignal = signalHead;
            while (currentSignal) {
                double connectedRoadsDensity = 0;
                int numConnectedRoads = 0;

                // Calculate average density of connected roads
                for (const string& roadId : currentSignal->data.connectedRoads) {
                    RoadList* road = findRoad(roadId);
                    if (road) {
                        connectedRoadsDensity += road->data.trafficDensity;
                        numConnectedRoads++;
                    }
                }

                double avgDensity = numConnectedRoads > 0 ?
                                   connectedRoadsDensity / numConnectedRoads : 0;

                // Set signal state based on average density
                if (avgDensity >= 70) {
                    currentSignal->data.currentState = TrafficSignal::GREEN;
                    currentSignal->data.greenDuration = 45;
                    currentSignal->data.redDuration = 15;
                    currentSignal->data.timeRemaining = 45;
                } else if (avgDensity >= 40) {
                    currentSignal->data.currentState = TrafficSignal::YELLOW;
                    currentSignal->data.greenDuration = 35;
                    currentSignal->data.redDuration = 25;
                    currentSignal->data.timeRemaining = 35;
                } else {
                    currentSignal->data.currentState = TrafficSignal::RED;
                    currentSignal->data.greenDuration = 30;
                    currentSignal->data.redDuration = 30;
                    currentSignal->data.timeRemaining = 30;
                }

                // Calculate unique efficiency for each signal
                double baseEfficiency;
                if (avgDensity >= 70) {
                    baseEfficiency = 95.0;  // Very efficient for high traffic
                } else if (avgDensity >= 50) {
                    baseEfficiency = 85.0;  // More efficient for medium-high traffic
                } else if (avgDensity >= 30) {
                    baseEfficiency = 75.0;  // Medium efficiency for medium traffic
                } else {
                    baseEfficiency = 65.0;  // Lower efficiency for low traffic
                }

                // Adjust efficiency based on signal timing and connected roads
                double timingFactor = static_cast<double>(currentSignal->data.greenDuration) /
                                    (currentSignal->data.greenDuration + currentSignal->data.redDuration);

                double congestionFactor = 1.0;
                for (const string& roadId : currentSignal->data.connectedRoads) {
                    RoadList* road = findRoad(roadId);
                    if (road) {
                        congestionFactor *= (1.0 - (road->data.congestionLevel * 0.05));
                    }
                }

                // Final efficiency calculation
                double finalEfficiency = baseEfficiency * timingFactor * congestionFactor;

                // Store the calculated efficiency
                currentSignal->data.historicalDurations.push_back(static_cast<int>(finalEfficiency));

                currentSignal = currentSignal->next;
            }

            monitorTrafficFlow();
            displaySystemStatus();
            cout << endl;
        }
    }

private:
    bool dfs(Node* current, Node* target, set<Node*>& visited) {
        visited.insert(current);

        if (current == target) {
            pathStack.push(current);
            return true;
        }

        for (Edge* edge : current->edges) {
            Node* next = edge->destination;
            if (visited.find(next) == visited.end()) {
                if (dfs(next, target, visited)) {
                    pathStack.push(current);
                    return true;
                }
            }
        }
        return false;
    }

    int countSignals() {
        int count = 0;
        SignalList* current = signalHead;
        while (current) {
            count++;
            current = current->next;
        }
        return count;
    }

    int countRoads() {
        int count = 0;
        RoadList* current = roadHead;
        while (current) {
            count++;
            current = current->next;
        }
        return count;
    }

    RoadList* findRoad(const string& roadId) {
        RoadList* current = roadHead;
        while (current) {
            if (current->data.roadId == roadId) return current;
            current = current->next;
        }
        return nullptr;
    }

    void optimizeSignalTimings(const string& signalId) {
        SignalList* signal = findSignal(signalId);
        if (!signal) return;

        int totalDensity = 0;
        int maxCongestion = 0;

        for (const string& roadId : signal->data.connectedRoads) {
            RoadList* road = findRoad(roadId);
            if (road) {
                totalDensity += road->data.trafficDensity;
                maxCongestion = max(maxCongestion, road->data.congestionLevel);
            }
        }

        if (maxCongestion >= 2 || totalDensity > 150) {
            signal->data.greenDuration = 45;
            signal->data.redDuration = 15;
            signal->data.yellowDuration = 3;
        } else if (totalDensity > 100) {
            signal->data.greenDuration = 35;
            signal->data.redDuration = 25;
            signal->data.yellowDuration = 4;
        } else {
            signal->data.greenDuration = 30;
            signal->data.redDuration = 30;
            signal->data.yellowDuration = 5;
        }

        // Store historical duration for analytics
        signal->data.historicalDurations.push_back(signal->data.greenDuration);
    }

    SignalList* findSignal(const string& signalId) {
        SignalList* current = signalHead;
        while (current) {
            if (current->data.signalId == signalId) return current;
            current = current->next;
        }
        return nullptr;
    }

    void handleEmergencyVehicle(const string& roadId) {
        RoadList* road = findRoad(roadId);
        if (!road) return;

        road->data.hasEmergencyVehicle = true;

        SignalList* currentSignal = signalHead;
        while (currentSignal) {
            auto it = find(currentSignal->data.connectedRoads.begin(),
                         currentSignal->data.connectedRoads.end(),
                         roadId);
            if (it != currentSignal->data.connectedRoads.end()) {
                currentSignal->data.isEmergencyMode = true;
                currentSignal->data.greenDuration = 45;
                currentSignal->data.currentState = TrafficSignal::GREEN;
            }
            currentSignal = currentSignal->next;
        }
    }

    void updateTrafficDensity(const string& roadId, int newDensity) {
        RoadList* road = findRoad(roadId);
        if (!road || newDensity < 0 || newDensity > 100) return;

        road->data.trafficDensity = newDensity;
        road->data.historicalData.push_back({
            static_cast<int>(chrono::system_clock::now().time_since_epoch().count()),
            newDensity
        });

        // Update congestion level
        if (newDensity < 40) road->data.congestionLevel = 0;
        else if (newDensity < 60) road->data.congestionLevel = 1;
        else if (newDensity < 80) road->data.congestionLevel = 2;
        else road->data.congestionLevel = 3;

        // Update estimated travel time based on congestion
        road->data.estimatedTravelTime = road->data.estimatedTravelTime * (1 + (road->data.congestionLevel * 0.25));

        // Update hourly traffic volume
        int currentHour = chrono::system_clock::now().time_since_epoch().count() / 3600 % 24;
        road->data.hourlyTrafficVolume[currentHour] = newDensity;
        road->data.peakHourTraffic = *max_element(road->data.hourlyTrafficVolume.begin(),
                                          road->data.hourlyTrafficVolume.end());

        SignalList* currentSignal = signalHead;
        while (currentSignal) {
            auto it = find(currentSignal->data.connectedRoads.begin(),
                         currentSignal->data.connectedRoads.end(),
                         roadId);
            if (it != currentSignal->data.connectedRoads.end()) {
                optimizeSignalTimings(currentSignal->data.signalId);
            }
            currentSignal = currentSignal->next;
        }
    }

    void monitorTrafficFlow() {
        cout << "\nTraffic Flow Report\n";
        RoadList* currentRoad = roadHead;
        while (currentRoad) {
            cout << "Road " << currentRoad->data.roadId << " - Density: "
                 << currentRoad->data.trafficDensity << "%\n";
            currentRoad = currentRoad->next;
        }
    }

    // Add this new helper function to initialize road data
    void initializeRoadData(const string& roadId, int density, int speed, const string& status) {
        RoadList* road = findRoad(roadId);
        if (road) {
            road->data.trafficDensity = density;
            road->data.speed = speed;
            road->data.estimatedTravelTime = calculateTravelTime(density, speed);
            road->data.congestionLevel = calculateCongestionLevel(density);

            // Update historical data
            road->data.historicalData.push_back({
                static_cast<int>(chrono::system_clock::now().time_since_epoch().count()),
                density
            });

            // Update hourly volume
            int currentHour = chrono::system_clock::now().time_since_epoch().count() / 3600 % 24;
            road->data.hourlyTrafficVolume[currentHour] = density;

            // Update other metrics
            road->data.averageSpeed = speed;
            road->data.peakHourTraffic = max(road->data.peakHourTraffic, density);
        }
    }

    // Helper function to calculate travel time based on density and speed
    int calculateTravelTime(int density, int speed) {
        // Base travel time increases with density and decreases with speed
        return (100 + density) / (speed / 30);
    }

    // Helper function to calculate congestion level
    int calculateCongestionLevel(int density) {
        if (density >= 80) return 3;      // High congestion
        if (density >= 60) return 2;      // Moderate congestion
        if (density >= 40) return 1;      // Light congestion
        return 0;                         // No congestion
    }
};

int main() {
    try {
        TrafficManagementSystem tms(5);

        // Initialize road network with more interconnections
        tms.addRoad("F8", "F6", 5);
        tms.addRoad("F6", "E5", 8);
        tms.addRoad("E5", "I8", 4);
        tms.addRoad("I8", "F7", 2);
        tms.addRoad("F8", "F7", 8);
        // Add these new interconnections
        tms.addRoad("F8", "E5", 6);  // F8 connects to E5
        tms.addRoad("F6", "F7", 2);   // F6 connects to F7
        tms.addRoad("F6", "I8", 1);   // F6 connects to I8

        cout << "\033[1;36m" << "=== Traffic Management System  ===" << "\033[0m" << endl;
        cout << "\033[1;32m" << "Analytics On Preset Data" << "\033[0m" << endl;

        // First run the simulation
        tms.simulateRealTimeTraffic();

        // Then show the optimal route based on updated traffic conditions
        vector<string> route = tms.findOptimalRoute("F8", "F7");
        cout << "\nOptimal route after traffic analysis: ";
        for (const string& node : route) {
            cout << node << " ";
        }
        cout << endl;

        // Manual emergency signal control loop
        char choice;
        do {
            cout << "\n\033[1;33mDo you want to change signal state for a specific street? (y/n): \033[0m";
            cin >> choice;

            if (tolower(choice) == 'y') {
                string streetId;
                cout << "Enter street ID: ";
                cin >> streetId;

                // Validate street exists
                if (!tms.hasSignal(streetId)) {
                    cout << "\033[1;31mError: Street ID not found!\033[0m\n";
                    continue;
                }

                cout << "\nSelect new signal state:\n";
                cout << "1. RED\n";
                cout << "2. YELLOW\n";
                cout << "3. GREEN\n";
                cout << "Choice: ";

                int stateChoice;
                cin >> stateChoice;

                TrafficSignal::State newState;
                switch(stateChoice) {
                    case 1:
                        newState = TrafficSignal::RED;
                        break;
                    case 2:
                        newState = TrafficSignal::YELLOW;
                        break;
                    case 3:
                        newState = TrafficSignal::GREEN;
                        break;
                    default:
                        cout << "\033[1;31mInvalid choice!\033[0m\n";
                        continue;
                }

                tms.updateSignalState(streetId, newState);

                // Display updated status
                cout << "\n\033[1;32mSignal state updated. Current system status:\033[0m\n";
                tms.displaySystemStatus();
            }

        } while (tolower(choice) == 'y');

        return 0;
    }
    catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }
}