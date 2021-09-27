#pragma once
#include <vector>
#include <string>
#include <map>
#include <ntcan.h>
#include <ros_esdcan_bridge/utils.h>

namespace esdcan
{
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

} // namespace esdcan