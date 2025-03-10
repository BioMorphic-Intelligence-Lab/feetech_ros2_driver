#include <boost/asio.hpp>
#include <iostream>
#include <thread>

/* Creates a timer that calls the callback function at a given frequency in a separate thread*/

class BoostTimer {
public:
    BoostTimer(double frequency, std::function<void()> callback)
        : io_service_(), timer_(io_service_), callback_(callback),
          period_(std::chrono::milliseconds(static_cast<int>(1000.0 / frequency))) {
        thread_ = std::thread([this]() { io_service_.run(); });
        start();
    }

    ~BoostTimer() {
        io_service_.stop();
        if (thread_.joinable()) {
            thread_.join();
        }
    }

private:
    void start() {
        timer_.expires_after(period_);
        timer_.async_wait([this](const boost::system::error_code& ec) {
            if (!ec) {
                callback_();
                start();
            }
        });
    }

    boost::asio::io_service io_service_;
    boost::asio::steady_timer timer_;
    std::function<void()> callback_;
    std::chrono::milliseconds period_;
    std::thread thread_;
};