// #include <iostream>
// #include <vector>
// #include <cmath>
// #include <limits>
// #include <algorithm>
// #include <chrono>

// using namespace std;

// class AndreyTree {
// private:
//     struct Node {
//         vector<float> point;
//         Node* left;
//         Node* right;

//         Node(const vector<float>& arr) : point(arr), left(nullptr), right(nullptr) {}
//     };

//     Node* root;
//     size_t dimensions;

//     Node* insertRecursive(Node* node, const vector<float>& point, size_t depth) {
//         if (node == nullptr) {
//             return new Node(point);
//         }

//         size_t cd = depth % dimensions;

//         if (point[cd] < node->point[cd]) {
//             node->left = insertRecursive(node->left, point, depth + 1);
//         } else {
//             node->right = insertRecursive(node->right, point, depth + 1);
//         }

//         return node;
//     }

//     Node* buildTree(vector<vector<float>>& points, size_t depth) {
//         if (points.empty()) {
//             return nullptr;
//         }

//         size_t cd = depth % dimensions;
//         sort(points.begin(), points.end(), [cd](const vector<float>& a, const vector<float>& b) {
//             return a[cd] < b[cd];
//         });

//         size_t medianIndex = points.size() / 2;
//         Node* node = new Node(points[medianIndex]);

//         vector<vector<float>> leftPoints(points.begin(), points.begin() + medianIndex);
//         vector<vector<float>> rightPoints(points.begin() + medianIndex + 1, points.end());

//         node->left = buildTree(leftPoints, depth + 1);
//         node->right = buildTree(rightPoints, depth + 1);

//         return node;
//     }

//     bool searchRecursive(Node* node, const vector<float>& point, size_t depth) const {
//         if (node == nullptr) {
//             return false;
//         }

//         if (node->point == point) {
//             return true;
//         }

//         size_t cd = depth % dimensions;

//         if (point[cd] < node->point[cd]) {
//             return searchRecursive(node->left, point, depth + 1);
//         } else {
//             return searchRecursive(node->right, point, depth + 1);
//         }
//     }

//     void printRecursive(Node* node, size_t depth) const {
//         if (node == nullptr) {
//             return;
//         }

//         for (size_t i = 0; i < depth; ++i) {
//             cout << "  ";
//         }
//         for (size_t i = 0; i < dimensions; ++i) {
//             cout << node->point[i] << " ";
//         }
//         cout << endl;

//         printRecursive(node->left, depth + 1);
//         printRecursive(node->right, depth + 1);
//     }

//     double distanceSquared(const vector<float>& a, const vector<float>& b) const {
//         double dist = 0;
//         for (size_t i = 0; i < dimensions; ++i) {
//             dist += (a[i] - b[i]) * (a[i] - b[i]);
//         }
//         return dist;
//     }

//     void nearestRecursive(Node* node, const vector<float>& target, size_t depth, Node*& best, double& bestDist) const {
//         if (node == nullptr) {
//             return;
//         }

//         double d = distanceSquared(node->point, target);
//         if (d < bestDist) {
//             bestDist = d;
//             best = node;
//         }

//         size_t cd = depth % dimensions;
//         Node* next = (target[cd] < node->point[cd]) ? node->left : node->right;
//         Node* other = (target[cd] < node->point[cd]) ? node->right : node->left;

//         nearestRecursive(next, target, depth + 1, best, bestDist);

//         if ((target[cd] - node->point[cd]) * (target[cd] - node->point[cd]) < bestDist) {
//             nearestRecursive(other, target, depth + 1, best, bestDist);
//         }
//     }

// public:
//     AndreyTree(size_t dims) : root(nullptr), dimensions(dims) {}

//     void insert(const vector<vector<float>>& points) {
//         vector<vector<float>> pointsCopy = points;
//         root = buildTree(pointsCopy, 0);
//     }

//     bool search(const vector<float>& point) const {
//         return searchRecursive(root, point, 0);
//     }

//     void print() const {
//         printRecursive(root, 0);
//     }

//     vector<float> nearest(const vector<float>& target) const {
//         Node* best = nullptr;
//         double bestDist = numeric_limits<double>::max();
//         nearestRecursive(root, target, 0, best, bestDist);
//         return best->point;
//     }
// };

// // int main() {
// //     // Create a KDTree with 3 dimensions
// //     AndreyTree tree(3);

// //     // Insert points into the KDTree
// //     vector<vector<float>> points = {
// //         {5.0, 2.0, 3.0},
// //         {8.0, 5.0, 6.0},
// //         {7.0, 8.0, 9.0},
// //         {4.0, 7.0, 2.0},
// //         {2.0, 9.0, 4.0},
// //         {9.0, 6.0, 7.0},
// //         {5.0, 4.0, 5.0},
// //         {6.0, 3.0, 8.0},
// //         {3.0, 1.0, 1.0}
// //     };
// //     tree.insert(points);

// //     // Print the KDTree
// //     cout << "KDTree structure:" << endl;
// //     tree.print();

// //     // Find the nearest point to the target
// //     vector<float> target = {5.0, 5.0, 5.0};

// //     std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
// //     vector<float> nearestPoint = tree.nearest(target);
// //     std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

// //     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

// //     cout << "Nearest point to (" << target[0] << ", " << target[1] << ", " << target[2] << ") is (";
// //     for (size_t i = 0; i < 3; ++i) {
// //         cout << nearestPoint[i];
// //         if (i < 2) {
// //             cout << ", ";
// //         }
// //     }
// //     cout << ")" << endl;

// //     cout << "Search took " << duration << " microseconds." << endl;

// //     return 0;
// // }