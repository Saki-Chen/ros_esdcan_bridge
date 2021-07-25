#include "can_io.h"

#include <iostream>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <jsoncpp/json/json.h>
#include <can_encode_decode_inl.h>

namespace esdcan
{
    CanBase::CanBase(int net, int32_t rx_queuesize, int32_t rx_timeout_ms, int32_t tx_queuesize, int32_t tx_timeout_ms, uint32_t ntcan_baud, bool object_mode)
        : _rx_queuesize(rx_queuesize), _is_open(false), _is_object_mode(object_mode)
    {
        uint32_t mode = NTCAN_MODE_NO_RTR | NTCAN_MODE_NO_INTERACTION; /* mode bits for canOpen */
        if (_is_object_mode)
            mode |= NTCAN_MODE_OBJECT;
        if (!check(canOpen(net, mode, tx_queuesize, rx_queuesize, tx_timeout_ms, rx_timeout_ms, &_nh)))
            return;

        if (_is_object_mode)
        {
            CAN_IF_STATUS status;
            canStatus(_nh, &status);
            if ((status.features & NTCAN_FEATURE_RX_OBJECT_MODE) == 0)
            {
                std::cerr << "error:object mode not supported" << std::endl;
                return;
            }
        }

        if (!check(canSetBaudrate(_nh, ntcan_baud)))
            return;

        _is_open = true;
    }

    CanBase::~CanBase()
    {
        if (!isOpen())
            return;
        check(canClose(_nh));
    }

    bool CanBase::check(NTCAN_RESULT error_code) const
    {
        if (error_code == NTCAN_SUCCESS)
            return true;
        char buf[128];
        canFormatError(error_code, NTCAN_ERROR_FORMAT_LONG, buf, 128);
        std::cerr << buf << std::endl;
        return false;
    }

    CanData::CanData(int32_t id_, ConstCanBytes data_)
    {
        id = id_;
        std::copy(data_, data_ + CAN_BYTES_COUNT, data);
    }

    CanReader::CanReader(int net, int32_t rx_queuesize, int32_t rx_timeout_ms, uint32_t ntcan_baud, bool object_mode)
        : CanBase(net, rx_queuesize, rx_timeout_ms, NTCAN_NO_QUEUE, 0, ntcan_baud, object_mode)
    {
        if (!_is_object_mode)
            _msg_buff.resize(rx_queuesize);
    }

    bool CanReader::take(std::vector<CanData> &can_data)
    {
        int32_t msg_len = _msg_buff.size();
        if (!check(canTake(_nh, &_msg_buff[0], &msg_len)))
            return false;
        if (_is_object_mode)
        {
            for (size_t i = 0; i < _msg_buff.size(); ++i)
            {
                if (_msg_buff[i].len & NTCAN_NO_DATA)
                    continue;
                can_data.emplace_back(_msg_buff[i].id, _msg_buff[i].data);
            }
        }
        else
        {
            can_data.reserve(msg_len);
            for (int32_t i = 0; i < msg_len; ++i)
            {
                can_data.emplace_back(_msg_buff[i].id, _msg_buff[i].data);
            }
        }
        return true;
    }

    bool CanReader::read(std::vector<CanData> &can_data)
    {
        if (_is_object_mode)
        {
            std::cerr << "error:call read in object mode" << std::endl;
            return false;
        }

        int32_t msg_len = _msg_buff.size();
        if (!check(canRead(_nh, &_msg_buff[0], &msg_len, NULL)))
            return false;
        can_data.reserve(msg_len);
        for (int32_t i = 0; i < msg_len; ++i)
        {
            can_data.emplace_back(_msg_buff[i].id, _msg_buff[i].data);
        }
        return true;
    }

    bool CanReader::addID(int32_t id)
    {
        if(id == ALL_ID)
        {
            if(_is_object_mode)
            {
                std::cerr << "error:it is not allowed to add all id in object mode" << std::endl;
                return false;
            }
            return _enable_all_id();
        }
        if(_is_object_mode && _msg_buff.size() >= _rx_queuesize)
        {
            std::cerr << "error:add too many id which is more than rx_queuesize" << std::endl;
            return false;
        }
        if (!check(canIdAdd(_nh, id)))
            return false;
        if (_is_object_mode)
        {
            _msg_buff.emplace_back();
            _msg_buff.back().id = id;
        }
        return true;
    }

    bool CanReader::_enable_all_id()
    {
        int32_t count = 1 << 11;
        return check(canIdRegionAdd(_nh, 0, &count)) && count == 1 << 11;
    }

    CanWriter::CanWriter(int net, int32_t tx_queuesize, int32_t tx_timeout_ms, uint32_t ntcan_baud)
        : CanBase(net, NTCAN_NO_QUEUE, 0, tx_queuesize, tx_timeout_ms, ntcan_baud, false)
    {
    }

    void CanWriter::_copy_to_buf(const CanData& data, CMSG& buf) const
    {
        buf.id = data.id;
        buf.len = sizeof(CanData::data);
        std::copy(&data.data[0], &data.data[0] + sizeof(CanData::data), &buf.data[0]);
    }

    void CanWriter::_copy_to_buf(const std::vector<CanData> &data, std::vector<CMSG> &buf) const
    {
        const auto size = data.size();
        buf.resize(size);
        for (size_t i = 0; i < size; ++i)
        {
            _copy_to_buf(data[i], buf[i]);
        }
    }

    bool CanWriter::send(const CanData &can_data) const
    {
        CMSG buf;
        _copy_to_buf(can_data, buf);

        int32_t msg_len = 1;
        if (!check(canSend(_nh, &buf, &msg_len)))
            return false;
        return msg_len == 1;
    }

    int CanWriter::send(const std::vector<CanData> &can_data) const
    {
        std::vector<CMSG> buf;
        _copy_to_buf(can_data, buf);

        int32_t msg_len = buf.size();
        if (!check(canSend(_nh, &buf[0], &msg_len)))
            return 0;
        return msg_len;
    }

    bool CanWriter::write(const CanData &can_data) const
    {
        CMSG buf;
        _copy_to_buf(can_data, buf);

        int32_t msg_len = 1;
        if (!check(canWrite(_nh, &buf, &msg_len, NULL)))
            return false;
        return msg_len == 1;
    }

    int CanWriter::write(const std::vector<CanData> &can_data) const
    {
        std::vector<CMSG> buf;
        _copy_to_buf(can_data, buf);

        int32_t msg_len = buf.size();
        if (!check(canWrite(_nh, &buf[0], &msg_len, NULL)))
            return 0;
        return msg_len;
    }

    CanIO::CanIO(int net, int32_t rx_queuesize, int32_t rx_timeout_ms, int32_t tx_queuesize, int32_t tx_timeout_ms, uint32_t ntcan_baud, bool object_mode)
        : CanBase(net, rx_queuesize, rx_timeout_ms, tx_queuesize, tx_timeout_ms, ntcan_baud, object_mode),
          CanReader(net, rx_queuesize, rx_timeout_ms, ntcan_baud, object_mode),
          CanWriter(net, tx_queuesize, tx_timeout_ms, ntcan_baud)
    {
    }

    void FrameDecoder::decode(ConstCanBytes bytes, const Signal &sig, double &result) const
    {
        result = ::decode(bytes, sig.start_bit, sig.length, _is_bigendian, sig.is_signed, sig.factor, sig.offset);
    }

    void FrameEncoder::encode(const double val, const Signal &sig, CanBytes bytes) const
    {
        ::encode(bytes, val, sig.start_bit, sig.length, _is_bigendian, sig.is_signed, sig.factor, sig.offset);
    }

    bool CanFrame::loadCanFrame(const std::string& json_params_file_path, const std::map<std::string, int32_t>& filtered_frame_id,  std::map<std::string, CanFrame>& filtered_frame)
    {
        Json::Value params;
        Json::Reader param_reader;

        std::ifstream fs(json_params_file_path);
        if (!param_reader.parse(fs, params, false))
        {
            std::cerr << "error:params file opening error in CanFrame" << std::endl;
            fs.close();
            return false;
        }
        fs.close();

        params = params.get("Frame", Json::Value::null);
        
        std::map<int32_t, CanFrame> frame_list;
        std::stringstream ss;
        ss >> std::hex;
        for(const auto &frame : params)
        {   
            ss.clear();
            ss.str(frame.get("id", "-1").asString());

            int32_t id;
            ss >> id;
            auto& new_frame = frame_list[id];
            new_frame.id = id;
            
            new_frame.is_bigendian = frame.get("is_bigendian", false).asBool();
            const auto& signal_list = frame.get("signal_list", Json::Value::null);
            for(const auto &sig : signal_list)
            {
                auto& new_sig = new_frame.signal_list[sig.get("key","").asString()];
                new_sig.factor = sig.get("factor", 1).asDouble();
                new_sig.offset = sig.get("offset", 0).asDouble(); 
                new_sig.start_bit = sig.get("start_bit", 0).asUInt();
                new_sig.length = sig.get("length", 0).asUInt();
                new_sig.is_signed = sig.get("is_signed", false).asBool();
            }
            if(new_frame.signal_list.size() != signal_list.size())
            {
                std::cerr << "error:params error in loading CanFrame\nthere may existing same id in config file " << std::endl;
                return false;
            }
        }

        if(params.size() != frame_list.size())
        {
            std::cerr << "error:params error in loading CanFrame\n there may existing same key in one frame in config file" << std::endl;
            return false;
        }

        for(const auto&item : filtered_frame_id)
        {
            auto p = frame_list.find(item.second);
            if(frame_list.end() == p)
            {
                std::cerr << "error:can frame: " << item.first << " with id: 0x" << std::hex << item.second << " not exists in config file" << std::endl;
                return false;
            }
            filtered_frame[item.first] = p->second;
        }   

        return true;
    }

    bool CanFrame::loadAllCanFrame(const std::string& json_params_file_path, std::map<int32_t, CanFrame>& frame_list)
    {
        Json::Value params;
        Json::Reader param_reader;

        std::ifstream fs(json_params_file_path);
        if (!param_reader.parse(fs, params, false))
        {
            std::cerr << "error:params file opening error in CanFrame" << std::endl;
            fs.close();
            return false;
        }
        fs.close();

        params = params.get("Frame", Json::Value::null);

        if(params == Json::Value::null)
        {
            std::cerr << "field Frame not found in " << json_params_file_path << "\n";
            return false;
        }
        
        std::stringstream ss;
        ss >> std::hex;
        ss << std::hex;
        for(const auto &frame : params)
        {   
            frame.getMemberNames();

            ss.clear();
            ss.str(frame.get("id", "-1").asString());

            int32_t id;
            ss >> id;
            if(id == -1) 
            {
                std::cerr << "missing field: id\n";
                return false;
            }
            if(frame_list.count(id) > 0)
            {
                std::cerr << "id: " << id << " is not unique in " << json_params_file_path;
                return false;
            }
            
            auto& new_frame = frame_list[id];
            new_frame.id = id;
            
            new_frame.is_bigendian = frame.get("is_bigendian", false).asBool();
            const auto& signal_list = frame.get("signal_list", Json::Value::null);
            for(const auto &sig : signal_list)
            {
                auto& new_sig = new_frame.signal_list[sig.get("key","").asString()];
                new_sig.factor = sig.get("factor", 1).asDouble();
                new_sig.offset = sig.get("offset", 0).asDouble(); 
                new_sig.start_bit = sig.get("start_bit", 0).asUInt();
                new_sig.length = sig.get("length", 0).asUInt();
                new_sig.is_signed = sig.get("is_signed", false).asBool();
            }
        }

        return true;
    }

} // namespace esdcan