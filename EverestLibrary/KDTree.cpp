/// @file KDTree.cpp
/// @author J. Frederico Carvalho
///
/// This is an adaptation of the KD-tree implementation in rosetta code
/// https://rosettacode.org/wiki/K-d_tree
///
/// It is a reimplementation of the C code using C++.  It also includes a few
/// more queries than the original, namely finding all points at a distance
/// smaller than some given distance to a point.

#include <algorithm>
#include <cassert>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <vector>

#include "KDTree.hpp"

KDNode::KDNode() = default;

KDNode::KDNode(std::vector<float> const& pt, size_t const& idx_, KDNodePtr const& left_,
               KDNodePtr const& right_) {
    x = pt;
    index = idx_;
    left = left_;
    right = right_;
}

KDNode::KDNode(pointIndex const& pi, KDNodePtr const& left_,
               KDNodePtr const& right_) {
    x = pi.first;
    index = pi.second;
    left = left_;
    right = right_;
}

KDNode::~KDNode() = default;

float KDNode::coord(size_t const& idx) { return x.at(idx); }
KDNode::operator bool() { return (!x.empty()); }
KDNode::operator std::vector<float>() { return x; }
KDNode::operator size_t() { return index; }
KDNode::operator pointIndex() { return std::make_pair(x, index); }

KDNodePtr NewKDNodePtr() {
    KDNodePtr mynode = std::make_shared<KDNode>();
    return mynode;
}

inline float dist2(std::vector<float> const& a, std::vector<float> const& b) {
    assert(a.size() == b.size());
    float distc = 0;
    for (size_t i = 0; i < a.size(); i++) {
        float di = a.at(i) - b.at(i);
        distc += di * di;
    }
    return distc;
}

inline float dist2(KDNodePtr const& a, KDNodePtr const& b) {
    return dist2(a->x, b->x);
}

comparer::comparer(size_t idx_) : idx{idx_} {}

inline bool comparer::compare_idx(pointIndex const& a, pointIndex const& b) {
    return (a.first.at(idx) < b.first.at(idx));
}

inline void sort_on_idx(pointIndexArr::iterator const& begin,
                        pointIndexArr::iterator const& end, size_t idx) {
    comparer comp(idx);
    comp.idx = idx;

    using std::placeholders::_1;
    using std::placeholders::_2;

    std::nth_element(begin, begin + std::distance(begin, end) / 2, end,
                     std::bind(&comparer::compare_idx, comp, _1, _2));
}

namespace detail {
inline bool compare_node_distance(std::pair<KDNodePtr, float> a,
                                  std::pair<KDNodePtr, float> b) {
    return a.second < b.second;
}
} // namespace detail

// using std::vector<std::vector<float>> = std::vector<std::vector<float>>;

KDNodePtr KDTree::make_tree(pointIndexArr::iterator const& begin,
                            pointIndexArr::iterator const& end,
                            size_t const& level) {
    if (begin == end) {
        return leaf_; // empty tree
    }

    assert(std::distance(begin, end) > 0);

    size_t const dim = begin->first.size();
    sort_on_idx(begin, end, level);

    auto const num_points = std::distance(begin, end);
    auto const middle{std::next(begin, num_points / 2)};

    size_t const next_level{(level + 1) % dim};
    KDNodePtr const left{make_tree(begin, middle, next_level)};
    KDNodePtr const right{make_tree(std::next(middle), end, next_level)};
    return std::make_shared<KDNode>(*middle, left, right);
}

KDTree::KDTree(std::vector<std::vector<float>> point_array) : leaf_{std::make_shared<KDNode>()} {
    pointIndexArr arr;
    for (size_t i = 0; i < point_array.size(); i++) {
        arr.emplace_back(point_array.at(i), i);
    }
    root_ = KDTree::make_tree(arr.begin(), arr.end(), 0 /* level */);
}

void KDTree::node_query_(
    KDNodePtr const& branch, std::vector<float> const& pt, size_t const& level,
    size_t const& num_nearest,
    std::list<std::pair<KDNodePtr, float>>& k_nearest_buffer) {
    if (!static_cast<bool>(*branch)) {
        return;
    }
    knearest_(branch, pt, level, num_nearest, k_nearest_buffer);
    float const dl = dist2(branch->x, pt);
    // assert(*branch);
    auto const node_distance = std::make_pair(branch, dl);
    auto const insert_it =
        std::upper_bound(k_nearest_buffer.begin(), k_nearest_buffer.end(),
                         node_distance, detail::compare_node_distance);
    if (insert_it != k_nearest_buffer.end() ||
        k_nearest_buffer.size() < num_nearest) {
        k_nearest_buffer.insert(insert_it, node_distance);
    }
    while (k_nearest_buffer.size() > num_nearest) {
        k_nearest_buffer.pop_back();
    }
}

void KDTree::knearest_(
    KDNodePtr const& branch, std::vector<float> const& pt, size_t const& level,
    size_t const& num_nearest,
    std::list<std::pair<KDNodePtr, float>>& k_nearest_buffer) {
    if (branch == nullptr || !static_cast<bool>(*branch)) {
        return;
    }

    std::vector<float> branch_pt{*branch};
    size_t dim = branch_pt.size();
    assert(dim != 0);
    assert(dim == pt.size());

    float const dx = branch_pt.at(level) - pt.at(level);
    float const dx2 = dx * dx;

    // select which branch makes sense to check
    KDNodePtr const close_branch = (dx > 0) ? branch->left : branch->right;
    KDNodePtr const far_branch = (dx > 0) ? branch->right : branch->left;

    size_t const next_level = (level + 1) % dim;
    node_query_(close_branch, pt, next_level, num_nearest, k_nearest_buffer);

    // only check the other branch if it makes sense to do so
    if (dx2 < k_nearest_buffer.back().second ||
        k_nearest_buffer.size() < num_nearest) {
        node_query_(far_branch, pt, next_level, num_nearest, k_nearest_buffer);
    }
};

// default caller
KDNodePtr KDTree::nearest_(std::vector<float> const& pt) {
    size_t level = 0;
    std::list<std::pair<KDNodePtr, float>> k_buffer{};
    k_buffer.emplace_back(root_, dist2(static_cast<std::vector<float>>(*root_), pt));
    knearest_(root_,   // beginning of tree
              pt,      // point we are querying
              level,   // start from level 0
              1,       // number of nearest neighbours to return in k_buffer
              k_buffer // list of k nearest neigbours (to be filled)
    );
    if (k_buffer.size() > 0) {
        return k_buffer.front().first;
    }
    return nullptr;
};

std::vector<float> KDTree::nearest_point(std::vector<float> const& pt) {
    return static_cast<std::vector<float>>(*nearest_(pt));
}

size_t KDTree::nearest_index(std::vector<float> const& pt) {
    return static_cast<size_t>(*nearest_(pt));
}

pointIndex KDTree::nearest_pointIndex(std::vector<float> const& pt) {
    KDNodePtr Nearest = nearest_(pt);
    return static_cast<pointIndex>(*Nearest);
}

pointIndexArr KDTree::nearest_pointIndices(std::vector<float> const& pt,
                                           size_t const& num_nearest) {
    size_t level = 0;
    std::list<std::pair<KDNodePtr, float>> k_buffer{};
    k_buffer.emplace_back(root_, dist2(static_cast<std::vector<float>>(*root_), pt));
    knearest_(root_,       // beginning of tree
              pt,          // point we are querying
              level,       // start from level 0
              num_nearest, // number of nearest neighbours to return in k_buffer
              k_buffer);   // list of k nearest neigbours (to be filled)
    pointIndexArr output{num_nearest};
    std::transform(k_buffer.begin(), k_buffer.end(), output.begin(),
                   [](auto const& nodeptr_dist) {
                       return static_cast<pointIndex>(*(nodeptr_dist.first));
                   });
    return output;
}

std::vector<std::vector<float>> KDTree::nearest_points(std::vector<float> const& pt, size_t const& num_nearest) {
    auto const k_nearest{nearest_pointIndices(pt, num_nearest)};
    std::vector<std::vector<float>> k_nearest_points{k_nearest.size()};
    std::transform(k_nearest.begin(), k_nearest.end(), k_nearest_points.begin(),
                   [](pointIndex const& x) { return x.first; });
    return k_nearest_points;
}

indexArr KDTree::nearest_indices(std::vector<float> const& pt, size_t const& num_nearest) {
    auto const k_nearest{nearest_pointIndices(pt, num_nearest)};
    indexArr k_nearest_indices{k_nearest.size()};
    std::transform(k_nearest.begin(), k_nearest.end(),
                   k_nearest_indices.begin(),
                   [](pointIndex const& x) { return x.second; });
    return k_nearest_indices;
}

void KDTree::neighborhood_(KDNodePtr const& branch, std::vector<float> const& pt,
                           float const& rad2, size_t const& level,
                           pointIndexArr& nbh) {
    if (!bool(*branch)) {
        // branch has no point, means it is a leaf,
        // no points to add
        return;
    }

    size_t const dim = pt.size();

    float const d = dist2(static_cast<std::vector<float>>(*branch), pt);
    float const dx = static_cast<std::vector<float>>(*branch).at(level) - pt.at(level);
    float const dx2 = dx * dx;

    if (d <= rad2) {
        nbh.push_back(static_cast<pointIndex>(*branch));
    }

    KDNodePtr const close_branch = (dx > 0) ? branch->left : branch->right;
    KDNodePtr const far_branch = (dx > 0) ? branch->right : branch->left;

    size_t const next_level{(level + 1) % dim};
    neighborhood_(close_branch, pt, rad2, next_level, nbh);
    if (dx2 < rad2) {
        neighborhood_(far_branch, pt, rad2, next_level, nbh);
    }
}

pointIndexArr KDTree::neighborhood(std::vector<float> const& pt, float const& rad) {
    pointIndexArr nbh;
    neighborhood_(root_, pt, rad * rad, /*level*/ 0, nbh);
    return nbh;
}

std::vector<std::vector<float>> KDTree::neighborhood_points(std::vector<float> const& pt, float const& rad) {
    auto nbh = std::make_shared<pointIndexArr>();
    neighborhood_(root_, pt, rad * rad, /*level*/ 0, *nbh);
    std::vector<std::vector<float>> nbhp(nbh->size());
    auto const first = [](pointIndex const& x) { return x.first; };
    std::transform(nbh->begin(), nbh->end(), nbhp.begin(), first);
    return nbhp;
}

indexArr KDTree::neighborhood_indices(std::vector<float> const& pt, float const& rad) {
    auto nbh = std::make_shared<pointIndexArr>();
    neighborhood_(root_, pt, rad * rad, /*level*/ 0, *nbh);
    indexArr nbhi(nbh->size());
    auto const second = [](pointIndex const& x) { return x.second; };
    std::transform(nbh->begin(), nbh->end(), nbhi.begin(), second);
    return nbhi;
}
