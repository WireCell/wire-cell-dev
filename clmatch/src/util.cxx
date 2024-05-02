#include "WireCellCLMatch/util.h"

#include "WireCellUtil/Exceptions.h"
#include "WireCellImg/PointCloudFacade.h"
#include "WireCellAux/SimpleTensor.h"
#include "WireCellAux/SimpleTensorSet.h"


#include <fstream>

using namespace WireCell;
using namespace WireCell::PointCloud::Facade;
using namespace WireCell::PointCloud::Tree;

void WireCell::CLMatch::dump_bee_3d(const Points::node_t& root, const std::string& fn)
{
    using spdlog::debug;
    using WireCell::PointCloud::Facade::float_t;
    using WireCell::PointCloud::Facade::int_t;

    Json::Value bee;
    bee["runNo"] = 0;
    bee["subRunNo"] = 0;
    bee["eventNo"] = 0;
    bee["geom"] = "uboone";
    bee["type"] = "cluster";

    std::vector<float_t> x;
    std::vector<float_t> y;
    std::vector<float_t> z;
    std::vector<float_t> q;
    std::vector<int_t> cluster_id;
    int_t cid = 0;
    for (const auto cnode : root.children()) {  // this is a loop through all clusters ...
        Scope scope = {"3d", {"x", "y", "z"}};
        const auto& sv = cnode->value.scoped_view(scope);

        const auto& spcs = sv.pcs();  // spcs 'contains' all blobs in this cluster ...
        // int npoints = 0;

        for (const auto& spc : spcs) {  // each little 3D pc --> (blobs)   spc represents x,y,z in a blob
            if (spc.get().get("x") == nullptr) {
                debug("No x in point cloud, skip");
                continue;
            }

            // assume others exist
            const auto& x_ = spc.get().get("x")->elements<float_t>();
            const auto& y_ = spc.get().get("y")->elements<float_t>();
            const auto& z_ = spc.get().get("z")->elements<float_t>();
            const size_t n = x_.size();
            x.insert(x.end(), x_.begin(), x_.end());  // Append x_ to x
            y.insert(y.end(), y_.begin(), y_.end());
            z.insert(z.end(), z_.begin(), z_.end());
            q.insert(q.end(), n, 1.0);
            cluster_id.insert(cluster_id.end(), n, cid);
            // npoints += n;
        }

        // spc.kd() // kdtree ...
        // const auto& skd = sv.kd();
        // std::cout << "xin6: " << sv.npoints() << " " << npoints << " " << spcs.size() << " " <<
        // skd.points().size() << std::endl;

        ++cid;
    }

    Json::Value json_x(Json::arrayValue);
    for (const auto& val : x) {
        json_x.append(val / units::cm);
    }
    bee["x"] = json_x;

    Json::Value json_y(Json::arrayValue);
    for (const auto& val : y) {
        json_y.append(val / units::cm);
    }
    bee["y"] = json_y;

    Json::Value json_z(Json::arrayValue);
    for (const auto& val : z) {
        json_z.append(val / units::cm);
    }
    bee["z"] = json_z;

    Json::Value json_q(Json::arrayValue);
    for (const auto& val : q) {
        json_q.append(val);
    }
    bee["q"] = json_q;

    Json::Value json_cluster_id(Json::arrayValue);
    for (const auto& val : cluster_id) {
        json_cluster_id.append(val);
    }
    bee["cluster_id"] = json_cluster_id;

    // Write cfg to file
    std::ofstream file(fn);
    if (file.is_open()) {
        Json::StreamWriterBuilder writer;
        writer["indentation"] = "    ";
        writer["precision"] = 6; // significant digits
        std::unique_ptr<Json::StreamWriter> jsonWriter(writer.newStreamWriter());
        jsonWriter->write(bee, &file);
        file.close();
    }
    else {
        raise<ValueError>("Failed to open file: " + fn);
    }
}

#include <json/json.h>
namespace {

std::string create_json_object(const std::vector<double>& opt_t, const std::vector<double>& op_peTotal, const std::vector<std::vector<double>>& op_pes, int cluster_id, int id)
{
    Json::Value data;
    data["op_t"] = Json::Value(Json::arrayValue);
    for (const auto& val : opt_t) {
        data["op_t"].append(val);
    }

    data["op_peTotal"] = Json::Value(Json::arrayValue);
    for (const auto& val : op_peTotal) {
        data["op_peTotal"].append(val);
    }

    data["op_pes"] = Json::Value(Json::arrayValue);
    for (const auto& pes_one_flash : op_pes) {
        auto val = Json::Value(Json::arrayValue);
        for (const auto& pe : pes_one_flash) {
            val.append(pe);
        }
        data["op_pes"].append(val);
    }

    data["op_l1_t"] = Json::Value(Json::arrayValue);
    for (size_t i = 0; i < opt_t.size(); ++i) {
        data["op_l1_t"].append(Json::Value(Json::arrayValue));
    }

    data["op_l1_pe"] = Json::Value(Json::arrayValue);
    for (size_t i = 0; i < opt_t.size(); ++i) {
        data["op_l1_pe"].append(Json::Value(Json::arrayValue));
    }

    data["op_pes_pred"] = data["op_pes"];

    data["cluster_id"] = cluster_id;
    data["op_nomatching_cluster_ids"] = Json::Value(Json::arrayValue);
    data["runNo"] = 0;
    data["subRunNo"] = 0;
    data["eventNo"] = id;
    data["geom"] = "sbnd";

    Json::StreamWriterBuilder writer;
    writer["indentation"] = "    ";
    std::string jsonString = Json::writeString(writer, data);

    return jsonString;
}

} // namespace

void WireCell::CLMatch::dump_bee_flash(const ITensorSet::pointer& ts, const std::string& fn)
{
    using spdlog::debug;

    const auto& tens = ts->tensors();
    if (tens->size() != 1) {
        raise<ValueError>("Expected 1 tensor, got %d", tens->size());
    }

    const auto& ten = tens->at(0);

    typedef boost::multi_array<double, 2> MultiArray;
    boost::array<MultiArray::index, 2> shape = {ten->shape()[0], ten->shape()[1]};
    boost::multi_array_ref<double, 2> mar((double*)ten->data(), shape);
    for (size_t i = 0; i < shape[0]; ++i) {
        for (size_t j = 0; j < shape[1]; ++j) {
            std::cout << mar[i][j] << " ";
        }
        std::cout << std::endl;
    }
}