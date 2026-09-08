#ifndef SERIAL_NODE_DATA_BUFFER_HPP
#define SERIAL_NODE_DATA_BUFFER_HPP

#include "protocol.hpp"

#include <mutex>

namespace serial_node {

class DataBuffer {
public:
  void set(const serial_protocol::RxData& rx) {
    std::lock_guard<std::mutex> lk(mutex_);
    rx_ = rx;
    has_data_ = true;
  }

  serial_protocol::RxData get() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return rx_;
  }

  bool hasData() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return has_data_;
  }

private:
  mutable std::mutex mutex_;
  serial_protocol::RxData rx_{};
  bool has_data_{false};
};

}  // namespace serial_node

#endif
