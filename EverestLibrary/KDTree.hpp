#pragma once

/// @file KDTree.hpp
/// @author J. Frederico Carvalho
///
/// This is an adaptation of the KD-tree implementation in rosetta code
///  https://rosettacode.org/wiki/K-d_tree
/// It is a reimplementation of the C code using C++.
/// It also includes a few more queries than the original

#include <algorithm>
#include <functional>
#include <list>
#include <memory>
#include <vector>
#include <cstdio>
#include <cstdlib>

/// The point type (vector of float precision floats)
// using std::vector<float> = std::vector<float>;

/// Array of indices
using indexArr = std::vector<size_t>;

/// Pair of point and Index
using pointIndex = typename std::pair<std::vector<float>, size_t>;

class KDNode {
  public:
    using KDNodePtr = std::shared_ptr<KDNode>;
    size_t index;
    std::vector<float> x;
    KDNodePtr left;
    KDNodePtr right;

    // initializer
    KDNode();
    KDNode(std::vector<float> const&, size_t const&, KDNodePtr const&, KDNodePtr const&);
    KDNode(pointIndex const&, KDNodePtr const&, KDNodePtr const&);
    ~KDNode();

    // getter
    float coord(size_t const&);

    // conversions
    explicit operator bool();
    explicit operator std::vector<float>();
    explicit operator size_t();
    explicit operator pointIndex();
};

using KDNodePtr = std::shared_ptr<KDNode>;

KDNodePtr NewKDNodePtr();

// square euclidean distance
inline float dist2(std::vector<float> const&, std::vector<float> const&);
inline float dist2(KDNodePtr const&, KDNodePtr const&);

// Need for sorting
class comparer {
  public:
    size_t idx;
    explicit comparer(size_t idx_);
    inline bool compare_idx(std::pair<std::vector<float>, size_t> const&, //
                            std::pair<std::vector<float>, size_t> const&  //
    );
};

using pointIndexArr = typename std::vector<pointIndex>;

inline void sort_on_idx(pointIndexArr::iterator const&, //
                        pointIndexArr::iterator const&, //
                        size_t idx);

// using std::vector<std::vector<float>> = std::vector<std::vector<float>>;

class KDTree {

  public:
    KDTree() = default;

    /// Build a KDtree
    explicit KDTree(std::vector<std::vector<float>> point_array);

    /// Get the point which lies closest to the input point.
    /// @param pt input point.
    std::vector<float> nearest_point(std::vector<float> const& pt);

    /// Get the index of the point which lies closest to the input point.
    ///
    /// @param pt input point.
    size_t nearest_index(std::vector<float> const& pt);

    /// Get the point and its index which lies closest to the input point.
    ///
    /// @param pt input point.
    pointIndex nearest_pointIndex(std::vector<float> const& pt);

    /// Get both the point and the index of the points closest to the input
    /// point.
    ///
    /// @param pt input point.
    /// @param num_nearest Number of nearest points to return.
    ///
    /// @returns a vector containing the points and their respective indices
    /// which are at a distance smaller than rad to the input point.
    pointIndexArr nearest_pointIndices(std::vector<float> const& pt,
                                      size_t const& num_nearest);

    /// Get the nearest set of points to the given input point.
    ///
    /// @param pt input point.
    /// @param num_nearest Number of nearest points to return.
    ///
    /// @returns a vector containing the points which are at a distance smaller
    /// than rad to the input point.
    std::vector<std::vector<float>> nearest_points(std::vector<float> const& pt, size_t const& num_nearest);

    /// Get the indices of points closest to the input point.
    ///
    /// @param pt input point.
    /// @param num_nearest Number of nearest points to return.
    ///
    /// @returns a vector containing the indices of the points which are at a
    /// distance smaller than rad to the input point.
    indexArr nearest_indices(std::vector<float> const& pt, size_t const& num_nearest);

    /// Get both the point and the index of the points which are at a distance
    /// smaller than the input radius to the input point.
    ///
    /// @param pt input point.
    /// @param rad input radius.
    ///
    /// @returns a vector containing the points and their respective indices
    /// which are at a distance smaller than rad to the input point.
    pointIndexArr neighborhood(std::vector<float> const& pt, float const& rad);

    /// Get the points that are at a distance to the input point which is
    /// smaller than the input radius.
    ///
    /// @param pt input point.
    /// @param rad input radius.
    ///
    /// @returns a vector containing the points which are at a distance smaller
    /// than rad to the input point.
    std::vector<std::vector<float>> neighborhood_points(std::vector<float> const& pt, float const& rad);

    // side = 0 means left, side = 1 means right, side = 2 means root
        void printRecursive(KDNodePtr node, size_t depth, int side, FILE* pFile) const {
        if (node == nullptr) {
            return;
        }

        for (size_t i = 0; i < depth; ++i) {
            fprintf(pFile, "  ");
        }

        if (side == 0) {
            fprintf(pFile, "L- ");
        } else if (side == 1) {
            fprintf(pFile, "R- ");
        } else {
            fprintf(pFile, "Root- ");
        }

        fprintf(pFile, "Index: %zu Point: ", node->index);
        for (const auto& val : node->x) {
            fprintf(pFile, "%f ", val);
        }
        fprintf(pFile, "\n");

        printRecursive(node->left, depth + 1, 0, pFile);
        printRecursive(node->right, depth + 1, 1, pFile);
    }

    void printBT(const std::string& prefix, const KDNodePtr node, bool isLeft, FILE* pFile) const
    {
        if( node != nullptr )
        {
            // std::cout << prefix;
            fprintf(pFile, "%s", prefix.c_str());

            // std::cout << (isLeft ? "├──" : "└──" );
            if(node->left != nullptr || node->right != nullptr) {
              fprintf(pFile, "%s", (isLeft ? "├──" : "└──"));
            }

            // print the value of the node
            for (const auto& val : node->x) {
              fprintf(pFile, "%f ", val);
            }
            fprintf(pFile, "\n");

            // enter the next tree level - left and right branch
            if(node->left != nullptr || node->right != nullptr) {
              printBT( prefix + (isLeft ? "│   " : "    "), node->left, true, pFile);
              printBT( prefix + (isLeft ? "│   " : "    "), node->right, false, pFile);
            }

            

        }
    }

    void printBT(const KDNodePtr node, FILE* pFile) const
    {
        printBT("", node, false, pFile);    
    }

    /// Get the indices of points that are at a distance to the input point
    /// which is smaller than the input radius.
    ///
    /// @param pt input point.
    /// @param rad input radius.
    ///
    /// @returns a vector containing the indices of the points which are at a
    /// distance smaller than rad to the input point.
    indexArr neighborhood_indices(std::vector<float> const& pt, float const& rad);

  private:
    KDNodePtr make_tree(pointIndexArr::iterator const& begin,
                        pointIndexArr::iterator const& end,
                        size_t const& level);

    void knearest_(KDNodePtr const& branch, std::vector<float> const& pt,
                   size_t const& level, size_t const& num_nearest,
                   std::list<std::pair<KDNodePtr, float>>& k_nearest_buffer);

    void node_query_(KDNodePtr const& branch, std::vector<float> const& pt,
                     size_t const& level, size_t const& num_nearest,
                     std::list<std::pair<KDNodePtr, float>>& k_nearest_buffer);

    // default caller
    KDNodePtr nearest_(std::vector<float> const& pt);

    void neighborhood_(KDNodePtr const& branch, std::vector<float> const& pt,
                       float const& rad2, size_t const& level,
                       pointIndexArr& nbh);

    KDNodePtr root_;
    KDNodePtr leaf_;

    void printTheTree() const {
        // Clear tree log file
        if (remove("tree.txt") == 0) {
            printf("File deleted successfully tree.txt\n");
        } else {
            printf("Error deleting file tree.txt\n");
        }

        // Open tree log file
        FILE* pFile = fopen("tree.txt", "a+");
        if (pFile == nullptr) {
            printf("Error opening file tree.txt\n");
            return;
        }

        // printRecursive(root_, 0, 2, pFile);
        printBT(root_, pFile);

        fclose(pFile);
    }
};
