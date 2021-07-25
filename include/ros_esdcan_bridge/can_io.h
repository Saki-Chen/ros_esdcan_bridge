#ifndef ESDCAN_DRIVER_CAN_IO_H_
#define ESDCAN_DRIVER_CAN_IO_H_

#include <vector>
#include <string>
#include <map>
#include <ntcan.h>

namespace esdcan
{
    constexpr uint CAN_BYTES_COUNT = sizeof(CMSG::data);
    using CanBytes = uint8_t[CAN_BYTES_COUNT];
    using ConstCanBytes = const uint8_t[CAN_BYTES_COUNT];

    class CanBase
    {
    public:
        CanBase(int net, int32_t rx_queuesize, int32_t rx_timeout_ms, int32_t tx_queuesize, int32_t tx_timeout_ms, uint32_t ntcan_baud, bool object_mode);
        virtual ~CanBase();
        bool isOpen() const { return _is_open; };

    protected:
        bool check(NTCAN_RESULT error_code) const;
        NTCAN_HANDLE _nh;
        int32_t _rx_queuesize;
        bool _is_open;
        bool _is_object_mode;
    }; // class CanBase

    struct CanData
    {
        int32_t id;
        CanBytes data;

        CanData() {}
        CanData(int32_t id_, ConstCanBytes data_);
    }; //struct CanData

    class CanReader : public virtual CanBase
    {
    public:
        enum{ALL_ID = -1};
        CanReader(int net, int32_t rx_queuesize, int32_t rx_timeout_ms, uint32_t ntcan_baud = NTCAN_BAUD_500, bool object_mode = true);
        bool take(std::vector<CanData> &can_data);
        bool read(std::vector<CanData> &can_data);
        bool addID(int32_t id);

    private:
        bool _enable_all_id();
        std::vector<CMSG> _msg_buff;
    }; // class CanReader

    class CanWriter : public virtual CanBase
    {
    public:
        CanWriter(int net, int32_t tx_queuesize, int32_t tx_timeout_ms, uint32_t ntcan_baud = NTCAN_BAUD_500);
        bool send(const CanData &can_data) const;
        bool write(const CanData & can_data) const;
        int send(const std::vector<CanData> &can_data) const;
        int write(const std::vector<CanData> &can_data) const;

    private:
        void _copy_to_buf(const CanData &data, CMSG &buf) const;
        void _copy_to_buf(const std::vector<CanData> &data, std::vector<CMSG> &buf) const;

    }; // class CanWriter

    class CanIO : public CanReader, public CanWriter
    {
    public:
        CanIO(int net, int32_t rx_queuesize, int32_t rx_timeout_ms, int32_t tx_queuesize, int32_t tx_timeout_ms, uint32_t ntcan_baud = NTCAN_BAUD_500, bool object_mode = true);

    }; // class CanIO

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
        static bool loadCanFrame(const std::string& json_params_file_path, const std::map<std::string, int32_t>& filterd_frame_id, std::map<std::string, CanFrame>& filterd_frame);
        static bool loadAllCanFrame(const std::string& json_params_file_path, std::map<int32_t, CanFrame>& frame_list);
    };

    class FrameBase
    {
    public:
        FrameBase(bool is_bigendian) : _is_bigendian(is_bigendian) {}

        virtual ~FrameBase(){}

    protected:
        bool _is_bigendian;
    }; // class FrameBase

    class FrameDecoder : virtual public FrameBase
    {
    public:
        FrameDecoder(bool is_bigendian) : FrameBase(is_bigendian) {}
        void decode(ConstCanBytes bytes, const Signal& sig, double& result) const;
    private:
    }; // class FrameDecoder

    class FrameEncoder : virtual public FrameBase
    {
        public:
        FrameEncoder(bool is_bigendian) : FrameBase(is_bigendian) {}
        void encode(const double val, const Signal& sig, CanBytes bytes) const;
    }; // class FrameEncoder

    class FrameProcesser : public FrameEncoder, public FrameDecoder
    {
    public:
        FrameProcesser(bool is_bigendian) 
        : FrameBase(is_bigendian), FrameEncoder(is_bigendian), FrameDecoder(is_bigendian)
        {}
    }; // class FrameProcesser

} // namespace esdcan
#endif //ESDCAN_DRIVER_CAN_IO_H_