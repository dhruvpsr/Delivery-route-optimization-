#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <cmath>
#include <limits>
#include <queue>
#include <string>

using namespace std;

struct Node {
    string id;
    double latitude;
    double longitude;
    double height;
    map<string, double> edges;
    double heuristic;  // Heuristic for A*
    bool isRed;  // Flag to mark if the node is above the drone's flying height

    Node(string _id, double lat, double lon, double h) : id(_id), latitude(lat), longitude(lon), height(h), heuristic(0), isRed(false) {}
};

class Graph {
private:
    map<string, Node*> nodes;

public:
    void addNode(string node_id, double latitude, double longitude, double height,double drone_height) {
        if (height <= drone_height) { // Check if the node's height is within the drone's flying height
            Node* node = new Node(node_id, latitude, longitude, height);
            nodes[node_id] = node;
        }
    }


    const map<string, Node*>& getNodes() const {
        return nodes;
    }

    void addEdge(string node1_id, string node2_id, double distance) {
        Node* node1 = nodes[node1_id];
        Node* node2 = nodes[node2_id];
        node1->edges[node2_id] = distance;
        node2->edges[node1_id] = distance;
    }

    double calculateDistance(Node* node1, Node* node2) {
        double R = 6371; // Radius of the Earth in kilometers
        double lat1_rad = M_PI * node1->latitude / 180;
        double lon1_rad = M_PI * node1->longitude / 180;
        double lat2_rad = M_PI * node2->latitude / 180;
        double lon2_rad = M_PI * node2->longitude / 180;
        double delta_lat = lat2_rad - lat1_rad;
        double delta_lon = lon2_rad - lon1_rad;
        double a = pow(sin(delta_lat/2), 2) + cos(lat1_rad) * cos(lat2_rad) * pow(sin(delta_lon/2), 2);
        double c = 2 * atan2(sqrt(a), sqrt(1-a));
        double distance = R * c* 1000;

        // Adding vertical distance
        // double dh = abs(node2->height - node1->height);
        // distance = sqrt(pow(distance, 2) + pow(dh, 2));

        return distance;
    }

    void markRedNodes(double drone_height) {
        for (auto& node_pair : nodes) {
            if (node_pair.second->height > drone_height) {
                node_pair.second->isRed = true;
            } else {
                node_pair.second->isRed = false;
            }
        }
    }

    void calculateInitialHeuristic(string end_node_id) {
        Node* end_node = nodes[end_node_id];
        for (auto& node_pair : nodes) {
            Node* node = node_pair.second;
            node->heuristic = calculateDistance(node, end_node);
        }
    }

    vector<string> shortestPath(string start_node_id, string end_node_id, double drone_height, double *min_dist_now) {
        markRedNodes(drone_height);
        calculateInitialHeuristic(end_node_id);

        map<string, double> dist;
        map<string, string> prev;
        priority_queue<pair<double, string>, vector<pair<double, string>>, greater<pair<double, string>>> pq;

        for (auto it = nodes.begin(); it != nodes.end(); ++it) {
            if (!it->second->isRed) {
                dist[it->first] = numeric_limits<double>::infinity();
                prev[it->first] = "";
            }
        }

        dist[start_node_id] = 0;
        pq.push(make_pair(0, start_node_id));

        while (!pq.empty()) {
            string u = pq.top().second;
            pq.pop();
            for (auto it = nodes[u]->edges.begin(); it != nodes[u]->edges.end(); ++it) {
                string v = it->first;
                double weight = it->second;
                if (!nodes[v]->isRed) {
                    double new_dist = dist[u] + weight;
                    if (new_dist < dist[v]) {
                        dist[v] = new_dist;
                        prev[v] = u;
                        pq.push(make_pair(dist[v] + nodes[v]->heuristic, v));
                    }
                }
            }
        }
        if((dist[end_node_id] + 2*drone_height) < *min_dist_now){
            *min_dist_now = (dist[end_node_id] + 2*drone_height);
        }
        vector<string> path;
        string current = end_node_id;
        while (current != "") {
            path.insert(path.begin(), current);
            current = prev[current];
        }
        return path;
    }

    void connectNodesInRange(double range) {
        for (auto it1 = nodes.begin(); it1 != nodes.end(); ++it1) {
            for (auto it2 = next(it1); it2 != nodes.end(); ++it2) {
                double distance = calculateDistance(it1->second, it2->second);
                if (distance <= range) {
                    addEdge(it1->first, it2->first, distance);
                }
            }
        }
    }
};

void visualizeGraph(const Graph& graph) {
    std::ofstream dotFile("graph.dot");
    if (!dotFile.is_open()) {
        std::cerr << "Error: Unable to create DOT file\n";
        return;
    }

    // Write DOT file header
    dotFile << "digraph DroneGraph {\n";

    // Traverse the graph and write nodes and edges to the DOT file
    for (const auto& node_pair : graph.getNodes()) {
        const Node& node = *(node_pair.second); // Derefencing the pointer
        dotFile << "  " << node.id << " [label=\"" << node.id << "\\n" << node.latitude << ", " << node.longitude << "\\n" << node.height << "\"];\n";

        for (const auto& edge_pair : node.edges) {
            const std::string& neighbor_id = edge_pair.first;
            double distance = edge_pair.second;
            dotFile << "  " << node.id << " -> " << neighbor_id << " [label=\"" << distance << "\"];\n";
        }
    }

    // Write DOT file footer
    dotFile << "}\n";
    dotFile.close();

    //std::cout << "DOT file generated successfully: graph.dot\n";
}

int main() {
    Graph g;
    double min_height = 20;
    double drone_height = 50;
    double min_distance = 999999999.0;
    double drone_height_for_shortest_path = 20; 
    vector<string> shortest_path;

    // Read data from CSV file
    ifstream file("dsa_project_dataset.csv");
    if (!file.is_open()) {
        cerr << "Error: Could not open file." << endl;
        return 1;
    }

    std::cout << "Opened file." << endl; // Just to indicate that the file was opened successfully

    // Prompt user for starting and goal points
    string start_node_id, goal_node_id;
    std::cout << "Enter starting node ID: ";
    cin >> start_node_id;
    std::cout << "Enter goal node ID: ";
    cin >> goal_node_id;

    for(drone_height = 50; drone_height<=200; drone_height+=10){
        string line;
        while (getline(file, line)) {
            stringstream ss(line); // Create a stringstream from the line
            string node_id, latitude_str, longitude_str, height_str;

            // Extract data from the stringstream using getline
            getline(ss, node_id, ',');
            getline(ss, latitude_str, ',');
            getline(ss, longitude_str, ',');
            getline(ss, height_str);

            // Convert string data to appropriate types
            double latitude = stod(latitude_str);
            double longitude = stod(longitude_str);
            double height = stod(height_str);
            if(node_id == start_node_id){
                g.addNode("1000", latitude, longitude, min_height, drone_height);
            }
            if(node_id == goal_node_id){
                g.addNode("1001", latitude, longitude, min_height, drone_height);
            }
            if (height <= drone_height) { // Check height condition
                //cout<< node_id <<" "; // Print node ID
                // Add the node to your data structure
                g.addNode(node_id, latitude, longitude, height, drone_height);
            }

        }

        //std::cout << "\nNodes Added\n";

        double connection_range = 75; // Adjust the range as needed
        g.connectNodesInRange(connection_range);

        double min_distance_now = 999999999.0;
        // Find shortest path
        vector<string> shortest_path_now = g.shortestPath("1000", "1001", drone_height, &min_distance_now);

        //std::cout << "Nodes Connected\n";
        if(min_distance_now <= min_distance){
            min_distance = min_distance_now;
            shortest_path = shortest_path_now;
            //std::cout << min_distance << " ";
            drone_height_for_shortest_path = drone_height;
            visualizeGraph(g);
        }
    }

    file.close();
    if(shortest_path.empty()){
        std::cout << "No path found! Try increasing the connection range.";
        return 0;
    }
    std::cout << "\nShortest path Found!"<<endl<< "For Drone flying height = "<< drone_height_for_shortest_path<< endl;
    std::cout << "Length of Shortest path = "<< min_distance<< endl;
    std::cout << "Shortest path: ";
    for (const string& node : shortest_path) {
        std::cout << node << " ";
    }
    std::cout << endl;
    
    return 0;
}