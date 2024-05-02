#ifndef WIRECELLDEV_CLMATCH_CLMATCHING
#define WIRECELLDEV_CLMATCH_CLMATCHING

#include "WireCellIface/ITensorSetFanin.h"
#include "WireCellIface/IConfigurable.h"
#include "WireCellAux/Logger.h"

namespace WireCell::CLMatch {
    class CLMatching : public Aux::Logger, public ITensorSetFanin, public IConfigurable {
    public:
        CLMatching();
        virtual ~CLMatching();

        // INode, override because we get multiplicity at run time.
        virtual std::vector<std::string> input_types();

        // ITensorSetFanin
        // input: 0: charge, 1: light
        virtual bool operator()(const input_vector& invec, output_pointer& out);

        // IConfigurable
        virtual void configure(const WireCell::Configuration& config);
        virtual WireCell::Configuration default_configuration() const;

      private:
        // Count how many times we are called
        size_t m_count{0};
        // Currently can only be 2, TODO: remove this?
        size_t m_multiplicity {2};

        // refer to MultiAlgBlobClustering for the following
        std::string m_inpath{"pointtrees/%d"};
        std::string m_outpath{"pointtrees/%d"};
        std::string m_bee_dir {"data"};
    };
}

#endif