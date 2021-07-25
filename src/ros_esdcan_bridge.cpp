#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <can_io.h> 

using namespace esdcan;

class RosEsdcanBridge
{
public:
    RosEsdcanBridge(int net, 
    int32_t rx_queuesize = 100,
    int32_t tx_queuesize = 100,
    uint32_t ntcan_baud = NTCAN_BAUD_500)
    {
        _is_ready = false;
        reader.reset(new CanReader(net, rx_queuesize, 0, ntcan_baud, false));
        writer.reset(new CanWriter(net, tx_queuesize, 0, ntcan_baud));
        if(!reader->isOpen() || !writer->isOpen() || !reader->addID(CanReader::ALL_ID)) return; 
        _is_ready = true;
        
        ros::NodeHandle nh;
        _pub_can_msg = nh.advertise<can_msgs::Frame>("can_tx", rx_queuesize);
        _sub_can_msg = nh.subscribe<can_msgs::Frame>("can_rx", tx_queuesize, &RosEsdcanBridge::_can_rx_handler, this);
        
        _can_tx_thread = std::thread(&RosEsdcanBridge::_can_tx_handler, this);
    }

    bool isReady() const
    {
        return _is_ready;
    }

    ~RosEsdcanBridge(){_can_tx_thread.join();}
private:

    void _can_rx_handler(const can_msgs::FrameConstPtr& msg)
    {
        CanData data(msg->id, msg->data.begin());
        writer->send(data);
    }

    void _can_tx_handler()
    {
        ros::NodeHandle nh;
        
        std::vector<CanData> buf;
        can_msgs::Frame frame;
        frame.header.frame_id = "can_tx";
        frame.dlc = 8;
        frame.is_error = false;
        frame.is_extended = false;
        frame.is_rtr = false;
        const auto _1ms = std::chrono::milliseconds(1);
        while(nh.ok())
        {   
            buf.clear();
            frame.header.stamp = ros::Time::now();
            reader->take(buf);
            if(buf.empty())
            {
                std::this_thread::sleep_for(_1ms);
                continue;
            }
            for(const auto& can_data : buf)
            {
                frame.id = can_data.id;
                std::copy(can_data.data, can_data.data + sizeof(can_data.data), frame.data.begin());
                _pub_can_msg.publish(frame);
            }
        } 
    }

    bool _is_ready;
    std::shared_ptr<CanReader> reader;
    std::shared_ptr<CanWriter> writer;

    ros::Publisher _pub_can_msg;
    ros::Subscriber _sub_can_msg;

    std::thread _can_tx_thread;
}; // RosEsdcanBridge

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_esdcan_bridge");
    ros::NodeHandle private_nh("~");
   
    int net; 
    int32_t rx_queuesize;
    int32_t tx_queuesize;
    int32_t ntcan_baud;

    private_nh.param<int>("can_circuit_id", net, 0);
    private_nh.param<int32_t>("rx_queuesize", rx_queuesize, 100);
    private_nh.param<int32_t>("tx_queuesize", tx_queuesize, 100);
    private_nh.param<int32_t>("can_bit_rate", ntcan_baud, 500000);

    switch (ntcan_baud)
    {
    case 500000:
        ntcan_baud = NTCAN_BAUD_500;
        break;
    case 1000000:
        ntcan_baud = NTCAN_BAUD_1000;
        break;
    default:
        ntcan_baud = NTCAN_BAUD_500;
        break;
    }

    RosEsdcanBridge bridge(net, rx_queuesize, tx_queuesize, ntcan_baud);

    if(!bridge.isReady()) return EXIT_FAILURE;

    ros::spin();
}
