// Frame logger (brief Section 8): for every message on the given topics, log
// the header stamp and the wall-clock receipt time, without deserialising the
// message. Messages are received as topic_tools::ShapeShifter; the first 12
// serialised bytes of any message that starts with std_msgs/Header are
// seq (uint32), stamp.sec (uint32) and stamp.nsec (uint32).
//
// Usage: rosrun slambench_tools frame_logger <out.csv> <topic> [<topic> ...]
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

class FrameLogger {
 public:
  FrameLogger(ros::NodeHandle& nh, const std::string& path, const std::vector<std::string>& topics)
      : out_(path) {
    out_ << "wall_time,ros_time,topic,datatype,seq,stamp,size_bytes\n";
    for (const auto& topic : topics) {
      // Large queue so the logger itself never drops messages.
      subs_.push_back(nh.subscribe<topic_tools::ShapeShifter>(
          topic, 2000, boost::bind(&FrameLogger::callback, this, _1, topic), ros::VoidConstPtr(),
          ros::TransportHints().tcpNoDelay()));
    }
  }

  ~FrameLogger() { out_.flush(); }

 private:
  void callback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string& topic) {
    const ros::WallTime wall = ros::WallTime::now();
    const ros::Time now = ros::Time::now();
    buffer_.resize(msg->size());
    ros::serialization::OStream stream(buffer_.data(), buffer_.size());
    msg->write(stream);
    uint32_t seq = 0, sec = 0, nsec = 0;
    if (buffer_.size() >= 12) {
      std::memcpy(&seq, &buffer_[0], 4);
      std::memcpy(&sec, &buffer_[4], 4);
      std::memcpy(&nsec, &buffer_[8], 4);
    }
    out_ << wall.sec << '.' << std::setw(9) << std::setfill('0') << wall.nsec << ','
         << now.sec << '.' << std::setw(9) << std::setfill('0') << now.nsec << ','
         << topic << ',' << msg->getDataType() << ',' << seq << ','
         << sec << '.' << std::setw(9) << std::setfill('0') << nsec << ','
         << buffer_.size() << '\n';
    // Flush every line: the run script polls this file to detect the end of
    // the method's output (a few tens of lines per second, negligible cost).
    out_.flush();
  }

  std::ofstream out_;
  std::vector<ros::Subscriber> subs_;
  std::vector<uint8_t> buffer_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "frame_logger", ros::init_options::AnonymousName);
  if (argc < 3) {
    ROS_ERROR("usage: frame_logger <out.csv> <topic> [<topic> ...]");
    return 1;
  }
  ros::NodeHandle nh;
  std::vector<std::string> topics(argv + 2, argv + argc);
  FrameLogger logger(nh, argv[1], topics);
  ros::spin();
  return 0;
}
