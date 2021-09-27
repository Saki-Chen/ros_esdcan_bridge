#include <ros_esdcan_bridge/can_coder.h>
#include "can_encode_decode_inl.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <jsoncpp/json/json.h>

namespace esdcan
{
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
}