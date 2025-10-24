/**
 * @file kromek_hidapi_node.cpp
 * @brief ROS2 node for Kromek Sigma50 gamma-ray detector using HIDAPI
 *
 * Based on the LBL-ANP sigma50 package but ported to ROS2 Humble
 *
 * Configurable parameters:
 * - integration_seconds: Integration time (default: 1)
 * - num_bins: Number of histogram bins for rebinning (default: 100)
 * - gain: Detector gain 20-250 (default: read from device)
 * - lld: Lower Level Discriminator threshold (default: read from device)
 * - publish_rate_hz: Publishing rate in Hz (default: same as integration)
 */

#include <stdio.h>
#include <wchar.h>
#include <string.h>
#include <unistd.h>
#include <hidapi/hidapi.h>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>

#define SIGMA_VID 0x04d8
#define SIGMA_PID 0x0023

// HID Feature Report IDs
#define GAIN_GET_REPORT 0x82
#define GAIN_SET_REPORT 0x02
#define LLD_GET_REPORT  0x89
#define LLD_SET_REPORT  0x09
#define BIAS_GET_REPORT 0x87

const int REPORT_SIZE = 63;
const int DET_CHANNELS = 4096;

class KromekHidapiNode : public rclcpp::Node
{
public:
    KromekHidapiNode() : Node("kromek_hidapi_node")
    {
        // Declare parameters
        this->declare_parameter("integration_seconds", 1);
        this->declare_parameter("num_bins", 100);
        this->declare_parameter("gain", -1);  // -1 means read from device
        this->declare_parameter("lld", -1);   // -1 means read from device
        this->declare_parameter("publish_rate_hz", -1.0);  // -1 means use integration time
        this->declare_parameter("publish_histogram", true);
        this->declare_parameter("publish_sum", true);
        this->declare_parameter("publish_device_status", false);

        integration_seconds_ = this->get_parameter("integration_seconds").as_int();
        num_bins_ = this->get_parameter("num_bins").as_int();
        gain_setting_ = this->get_parameter("gain").as_int();
        lld_setting_ = this->get_parameter("lld").as_int();
        publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
        publish_histogram_ = this->get_parameter("publish_histogram").as_bool();
        publish_sum_ = this->get_parameter("publish_sum").as_bool();
        publish_device_status_ = this->get_parameter("publish_device_status").as_bool();

        // Validate parameters
        if (num_bins_ < 1 || num_bins_ > DET_CHANNELS) {
            RCLCPP_ERROR(this->get_logger(), "num_bins must be between 1 and %d", DET_CHANNELS);
            num_bins_ = 100;
        }
        if (gain_setting_ != -1 && (gain_setting_ < 20 || gain_setting_ > 250)) {
            RCLCPP_WARN(this->get_logger(), "gain must be between 20 and 250, ignoring");
            gain_setting_ = -1;
        }

        // Create publishers
        sum_pub_ = this->create_publisher<std_msgs::msg::Int64>("kromek/sum", 10);
        info_pub_ = this->create_publisher<std_msgs::msg::UInt32MultiArray>("kromek/info", 10);
        raw_pub_ = this->create_publisher<std_msgs::msg::UInt32MultiArray>("kromek/raw", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::UInt32MultiArray>("kromek/status", 10);

        RCLCPP_INFO(this->get_logger(), "Kromek HIDAPI Node initialized");
        RCLCPP_INFO(this->get_logger(), "  Integration time: %d seconds", integration_seconds_);
        RCLCPP_INFO(this->get_logger(), "  Histogram bins: %d", num_bins_);
        if (gain_setting_ > 0) {
            RCLCPP_INFO(this->get_logger(), "  Gain setting: %d", gain_setting_);
        }
        if (lld_setting_ > 0) {
            RCLCPP_INFO(this->get_logger(), "  LLD setting: %d", lld_setting_);
        }
    }

    ~KromekHidapiNode()
    {
        // Close all device handles
        for (auto handle : device_handles_) {
            if (handle) {
                hid_close(handle);
            }
        }
        hid_exit();
    }

    bool initialize()
    {
        if (hid_init()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize HIDAPI");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Enumerating Kromek devices...");

        struct hid_device_info *devs, *cur_dev;
        devs = hid_enumerate(SIGMA_VID, SIGMA_PID);
        cur_dev = devs;

        num_devices_ = 0;
        while (cur_dev) {
            RCLCPP_INFO(this->get_logger(), "Found Kromek Sigma50:");
            RCLCPP_INFO(this->get_logger(), "  Serial: %ls", cur_dev->serial_number);
            RCLCPP_INFO(this->get_logger(), "  Path: %s", cur_dev->path);

            // Open device
            hid_device *handle = hid_open_path(cur_dev->path);
            if (handle) {
                device_handles_.push_back(handle);
                device_serials_.push_back(cur_dev->serial_number ?
                                         std::wstring(cur_dev->serial_number) :
                                         std::wstring(L"unknown"));

                // Read current device settings
                int gain = readGain(handle);
                int bias = readBias(handle);
                int lld = readLLD(handle);

                RCLCPP_INFO(this->get_logger(), "  Current - Gain: %d, Bias: %d, LLD: %d",
                           gain, bias, lld);

                // Apply gain setting if specified
                if (gain_setting_ > 0 && gain_setting_ != gain) {
                    if (setGain(handle, gain_setting_)) {
                        RCLCPP_INFO(this->get_logger(), "  Set gain to: %d", gain_setting_);
                    }
                }

                // Apply LLD setting if specified
                if (lld_setting_ > 0 && lld_setting_ != lld) {
                    if (setLLD(handle, lld_setting_)) {
                        RCLCPP_INFO(this->get_logger(), "  Set LLD to: %d", lld_setting_);
                    }
                }

                // Store current settings
                device_gains_.push_back(gain);
                device_biases_.push_back(bias);
                device_llds_.push_back(lld);

                num_devices_++;
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to open device");
            }

            cur_dev = cur_dev->next;
        }

        hid_free_enumeration(devs);

        if (num_devices_ == 0) {
            RCLCPP_WARN(this->get_logger(), "No Kromek Sigma50 devices found!");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully opened %d Kromek devices", num_devices_);

        // Initialize data structures
        histograms_.resize(num_devices_ * DET_CHANNELS, 0);

        // Publish device info
        publishDeviceInfo();

        return true;
    }

    void publishDeviceInfo()
    {
        auto msg = std_msgs::msg::UInt32MultiArray();
        msg.data.resize(num_devices_);

        for (size_t i = 0; i < device_serials_.size(); i++) {
            std::wstring serial_wstr = device_serials_[i];
            if (!serial_wstr.empty()) {
                try {
                    msg.data[i] = std::stoi(serial_wstr);
                } catch (...) {
                    msg.data[i] = i;
                }
            }
        }

        info_pub_->publish(msg);
    }

    void publishDeviceStatus()
    {
        // Publish current gain, bias, LLD for each detector
        // Format: [dev0_gain, dev0_bias, dev0_lld, dev1_gain, dev1_bias, dev1_lld, ...]
        auto msg = std_msgs::msg::UInt32MultiArray();
        msg.data.resize(num_devices_ * 3);

        for (size_t i = 0; i < device_handles_.size(); i++) {
            // Re-read current values
            device_gains_[i] = readGain(device_handles_[i]);
            device_biases_[i] = readBias(device_handles_[i]);
            device_llds_[i] = readLLD(device_handles_[i]);

            msg.data[i * 3 + 0] = device_gains_[i];
            msg.data[i * 3 + 1] = device_biases_[i];
            msg.data[i * 3 + 2] = device_llds_[i];
        }

        status_pub_->publish(msg);
    }

    void run()
    {
        if (!initialize()) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Starting acquisition loop...");

        // Determine publishing rate
        double rate_hz = publish_rate_hz_;
        if (rate_hz <= 0) {
            rate_hz = 1.0 / integration_seconds_;
        }

        auto period_ms = static_cast<int>(1000.0 / rate_hz);

        auto loop_rate = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            [this]() { this->acquisitionLoop(); }
        );

        rclcpp::spin(this->shared_from_this());
    }

private:
    int readGain(hid_device *handle)
    {
        unsigned char buf[2];
        buf[0] = GAIN_GET_REPORT;
        int res = hid_get_feature_report(handle, buf, 2);
        return (res > 0) ? buf[1] : -1;
    }

    int readBias(hid_device *handle)
    {
        unsigned char buf[3];
        buf[0] = BIAS_GET_REPORT;
        int res = hid_get_feature_report(handle, buf, 3);
        if (res > 0) {
            return ((buf[1] << 4) & 0xFF0) + ((buf[2] >> 4) & 0xF);
        }
        return -1;
    }

    int readLLD(hid_device *handle)
    {
        unsigned char buf[3];
        buf[0] = LLD_GET_REPORT;
        int res = hid_get_feature_report(handle, buf, 3);
        if (res > 0) {
            return ((buf[1] << 4) & 0xFF0) + ((buf[2] >> 4) & 0xF);
        }
        return -1;
    }

    bool setGain(hid_device *handle, int gain)
    {
        if (gain < 20 || gain > 250) return false;

        unsigned char buf[2];
        buf[0] = GAIN_SET_REPORT;
        buf[1] = static_cast<unsigned char>(gain);
        int res = hid_send_feature_report(handle, buf, 2);
        return res > 0;
    }

    bool setLLD(hid_device *handle, int lld)
    {
        unsigned char buf[3];
        buf[0] = LLD_SET_REPORT;
        buf[1] = (lld >> 4) & 0xFF;
        buf[2] = (lld << 4) & 0xF0;
        int res = hid_send_feature_report(handle, buf, 3);
        return res > 0;
    }

    void acquisitionLoop()
    {
        uint64_t total_counts = 0;

        // Read data from all devices
        for (size_t dev_idx = 0; dev_idx < device_handles_.size(); dev_idx++) {
            unsigned char buf[REPORT_SIZE + 1];

            // Read events from device
            while (true) {
                int res = hid_read_timeout(device_handles_[dev_idx], buf, REPORT_SIZE, 10);

                if (res == 0) {
                    break;  // No more data
                } else if (res < 0) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                        "Error reading from device %zu", dev_idx);
                    break;
                }

                // Process the data packet - format: 12-bit channel values
                int offset = 1;
                while (offset + 1 < res && (buf[offset + 1] & 0x1) == 1) {
                    uint16_t channel = ((buf[offset] << 4) & 0xFF0) + ((buf[offset + 1] >> 4) & 0xF);
                    if (channel < DET_CHANNELS) {
                        histograms_[dev_idx * DET_CHANNELS + channel]++;
                        total_counts++;
                    }
                    offset += 2;
                }
            }
        }

        // Publish total counts
        if (publish_sum_) {
            auto sum_msg = std_msgs::msg::Int64();
            sum_msg.data = total_counts;
            sum_pub_->publish(sum_msg);
        }

        // Publish histogram (rebinned to num_bins_)
        if (publish_histogram_) {
            auto raw_msg = std_msgs::msg::UInt32MultiArray();
            raw_msg.data.resize(num_devices_ * num_bins_, 0);

            // Rebin from DET_CHANNELS to num_bins_
            int rebin_factor = DET_CHANNELS / num_bins_;
            for (int dev = 0; dev < num_devices_; dev++) {
                for (int i = 0; i < num_bins_; i++) {
                    uint32_t sum = 0;
                    for (int j = 0; j < rebin_factor; j++) {
                        int channel_idx = i * rebin_factor + j;
                        if (channel_idx < DET_CHANNELS) {
                            sum += histograms_[dev * DET_CHANNELS + channel_idx];
                        }
                    }
                    raw_msg.data[dev * num_bins_ + i] = sum;
                }
            }

            raw_pub_->publish(raw_msg);
        }

        // Publish device status if enabled
        if (publish_device_status_) {
            publishDeviceStatus();
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                            "Total counts this period: %lu", total_counts);
    }

    // Parameters
    int integration_seconds_;
    int num_bins_;
    int gain_setting_;
    int lld_setting_;
    double publish_rate_hz_;
    bool publish_histogram_;
    bool publish_sum_;
    bool publish_device_status_;

    // Device state
    int num_devices_;
    std::vector<hid_device*> device_handles_;
    std::vector<std::wstring> device_serials_;
    std::vector<int> device_gains_;
    std::vector<int> device_biases_;
    std::vector<int> device_llds_;
    std::vector<uint32_t> histograms_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr sum_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr info_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr raw_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr status_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<KromekHidapiNode>();
    node->run();

    rclcpp::shutdown();
    return 0;
}
