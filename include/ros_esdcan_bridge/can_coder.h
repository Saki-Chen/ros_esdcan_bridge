#pragma once
#include <string>
#include <map>
#include <ros_esdcan_bridge/utils.h>

namespace esdcan
{
    struct Signal
    {
        double factor;
        double offset;
        uint8_t start_bit;
        uint8_t length;
        bool is_signed;
    }; // struct Signal

    struct CanFrame
    {
        int32_t id;
        bool is_bigendian;
        std::map<std::string, Signal> signal_list;
        static bool loadCanFrame(const std::string &json_params_file_path, const std::map<std::string, int32_t> &filterd_frame_id, std::map<std::string, CanFrame> &filterd_frame);
        static bool loadAllCanFrame(const std::string &json_params_file_path, std::map<int32_t, CanFrame> &frame_list);
    };

    class FrameBase
    {
    public:
        FrameBase(bool is_bigendian) : _is_bigendian(is_bigendian) {}

        virtual ~FrameBase() {}

    protected:
        bool _is_bigendian;
    }; // class FrameBase

    class FrameDecoder : virtual public FrameBase
    {
    public:
        FrameDecoder(bool is_bigendian) : FrameBase(is_bigendian) {}
        void decode(ConstCanBytes bytes, const Signal &sig, double &result) const;

    private:
    }; // class FrameDecoder

    class FrameEncoder : virtual public FrameBase
    {
    public:
        FrameEncoder(bool is_bigendian) : FrameBase(is_bigendian) {}
        void encode(const double val, const Signal &sig, CanBytes bytes) const;
    }; // class FrameEncoder

    class FrameProcesser : public FrameEncoder, public FrameDecoder
    {
    public:
        FrameProcesser(bool is_bigendian)
            : FrameBase(is_bigendian), FrameEncoder(is_bigendian), FrameDecoder(is_bigendian)
        {
        }
    }; // class FrameProcesser
} // namespace esdcan
