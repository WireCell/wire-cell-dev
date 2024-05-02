#include "WireCellAux/SimpleTensor.h"

#include "WireCellUtil/doctest.h"
#include "WireCellUtil/Logging.h"
#include "WireCellUtil/Testing.h"

#include <iostream>

using namespace WireCell;
using spdlog::debug;

TEST_CASE("tensor and array")
{
    typedef boost::multi_array<double, 2> MultiArray;
    MultiArray array(boost::extents[2][3]);
    for (size_t i = 0; i < 2; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            array[i][j] = i * 3 + j;
        }
    }
    std::vector<size_t> shape = {array.shape()[0], array.shape()[1]};
    Json::Value md = Json::objectValue;
    auto tens = std::make_shared<Aux::SimpleTensor>(shape, array.data(), md);

    boost::array<MultiArray::index, 2> shape_arr = {shape[0], shape[1]};
    boost::multi_array_ref<double, 2> mar((double*)tens->data(), shape_arr);
    for (size_t i = 0; i < shape[0]; ++i) {
        for (size_t j = 0; j < shape[1]; ++j) {
            std::cout << mar[i][j] << " ";
        }
        std::cout << std::endl;
    }
}