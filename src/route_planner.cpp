#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.

    // auto start_node = m_Model.FindClosestNode(start_x, start_y); // --> This was wrong
    // auto end_node = m_Model.FindClosestNode(end_x, end_y); // --> This was wrong

    start_node = &model.FindClosestNode(start_x, start_y);
    end_node = &model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // return end_node->distance(*node); // This works too
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (RouteModel::Node *a_neighbour: current_node->neighbors){
        a_neighbour->parent = current_node;
        a_neighbour->h_value = CalculateHValue(a_neighbour);
        a_neighbour->g_value = current_node->g_value + current_node->distance(*a_neighbour);
        a_neighbour->visited = true;
        open_list.push_back(a_neighbour);
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {

    RouteModel::Node* next_node;
    // High simulated first least_score = 10 heuristic trips from start_node to end_node
    float least_score = CalculateHValue(start_node) * 10;
    int next_node_pos {0};
    int counter {0};
    for (RouteModel::Node* a_node: open_list){
        float score = a_node->h_value + a_node->g_value; 
        if (score<least_score){
            least_score = score;
            next_node = a_node;
            next_node_pos = counter;
        }
        counter ++;
    }
    open_list.erase(open_list.begin()+next_node_pos);
    return next_node;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    std::vector<RouteModel::Node*> reverse_path_found;

    // TODO: Implement your solution here.
    RouteModel::Node* next_parent = current_node;
    while (true){
        reverse_path_found.push_back(next_parent);
        if (next_parent->x == start_node->x && next_parent->y == start_node->y) break;
        // if (next_parent == start_node) break; // Simpler implementation of the above since both are pointers
        distance += next_parent->distance(*next_parent->parent);
        next_parent = next_parent->parent;
    }

    for (int i = reverse_path_found.size() -1; i >= 0; i --)
        path_found.push_back(*reverse_path_found[i]);
    
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;

    // TODO: Implement your solution here.
    current_node->visited = true;

    while (current_node != end_node){
        AddNeighbors(current_node);
        current_node = NextNode();
    }
    m_Model.path = ConstructFinalPath(current_node);

}