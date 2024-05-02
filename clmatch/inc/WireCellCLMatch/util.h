#ifndef WIRECELLDEV_CLMATCH_UTIL
#define WIRECELLDEV_CLMATCH_UTIL

#include "WireCellUtil/PointTree.h"
#include "WireCellIface/ITensorSet.h"

namespace WireCell::CLMatch {
    void dump_bee_3d(const PointCloud::Tree::Points::node_t& root, const std::string& fn);
    void dump_bee_flash(const ITensorSet::pointer& ts, const std::string& fn);
}

#endif