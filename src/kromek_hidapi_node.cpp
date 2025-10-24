/**
 * @file kromek_hidapi_node.cpp  
 * @brief ROS2 node for Kromek Sigma50 gamma-ray detector using HIDAPI
 *
 * Multi-device support:
 * - Publishes individual topics for each detector: /kromek_SERIAL/...
 * - Publishes combined topics summing all detectors: /kromek/combined/...
 */

#include <stdio.h>
#include <wchar.h>
#include <string.h>
#include <unistd.h>
#include <hidapi/hidapi.h>
#include <map>

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

// Per-device publishers
struct DevicePublishers {
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr sum_counts;
    rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr raw;
    rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr sum_spectrum;
};

class KromekHidapiNode : public rclcpp::Node
{
public:
    KromekHidapiNode() : Node("kromek_hidapi_node")
    {
        // Declare parameters
        this->declare_parameter("integration_seconds", 1);
        this->declare_parameter("num_bins", 100);
        this->declare_parameter("gain", -1);
        this->declare_parameter("lld", -1);
        this->declare_parameter("publish_rate_hz", -1.0);
        this->declare_parameter("publish_histogram", true);
        this->declare_parameter("publish_sum", true);
        this->declare_parameter("publish_device_status", false);
        this->declare_parameter("publish_individual_devices", true);
        this->declare_parameter("publish_combined", true);

        integration_seconds_ = this->get_parameter("integration_seconds").as_int();
        num_bins_ = this->get_parameter("num_bins").as_int();
        gain_setting_ = this->get_parameter("gain").as_int();
        lld_setting_ = this->get_parameter("lld").as_int();
        publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
        publish_histogram_ = this->get_parameter("publish_histogram").as_bool();
        publish_sum_ = this->get_parameter("publish_sum").as_bool();
        publish_device_status_ = this->get_parameter("publish_device_status").as_bool();
        publish_individual_ = this->get_parameter("publish_individual_devices").as_bool();
        publish_combined_ = this->get_parameter("publish_combined").as_bool();

        // Validate parameters
        if (num_bins_ < 1 || num_bins_ > DET_CHANNELS) {
            RCLCPP_ERROR(this->get_logger(), "num_bins must be between 1 and %d", DET_CHANNELS);
            num_bins_ = 100;
        }
        if (gain_setting_ != -1 && (gain_setting_ < 20 || gain_setting_ > 250)) {
            RCLCPP_WARN(this->get_logger(), "gain must be between 20 and 250, ignoring");
            gain_setting_ = -1;
        }

        // Create combined publishers
        if (publish_combined_) {
            combined_sum_counts_pub_ = this->create_publisher<std_msgs::msg::Int64>("kromek/combined/sum_counts", 10);
            combined_sum_spectrum_pub_ = this->create_publisher<std_msgs::msg::UInt32MultiArray>("kromek/combined/sum_spectrum", 10);
            combined_raw_pub_ = this->create_publisher<std_msgs::msg::UInt32MultiArray>("kromek/combined/raw", 10);
        }

        info_pub_ = this->create_publisher<std_msgs::msg::UInt32MultiArray>("kromek/info", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::UInt32MultiArray>("kromek/status", 10);

        RCLCPP_INFO(this->get_logger(), "Kromek HIDAPI Node initialized");
        RCLCPP_INFO(this->get_logger(), "  Integration time: %d seconds", integration_seconds_);
        RCLCPP_INFO(this->get_logger(), "  Histogram bins: %d", num_bins_);
        RCLCPP_INFO(this->get_logger(), "  Individual device topics: %s", publish_individual_ ? "enabled" : "disabled");
        RCLCPP_INFO(this->get_logger(), "  Combined topics: %s", publish_combined_ ? "enabled" : "disabled");
    }

    ~KromekHidapiNode()
    {
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

        while (cur_dev) {
            RCLCPP_INFO(this->get_logger(), "Found Kromek Sigma50:");
            RCLCPP_INFO(this->get_logger(), "  Serial: %ls", cur_dev->serial_number);
            
            hid_device *handle = hid_open_path(cur_dev->path);
            if (handle) {
                std::wstring serial = cur_dev->serial_number ? 
                                     std::wstring(cur_dev->serial_number) : 
                                     std::wstring(L"unknown");
                
                std::string serial_str;
                for (wchar_t c : serial) {
                    serial_str += static_cast<char>(c);
                }

                device_handles_.push_back(handle);
                device_serials_.push_back(serial);

                // Read settings
                int gain = readGain(handle);
                int bias = readBias(handle);
                int lld = readLLD(handle);

                RCLCPP_INFO(this->get_logger(), "  Current - Gain: %d, Bias: %d, LLD: %d", gain, bias, lld);

                // Apply settings if specified
                if (gain_setting_ > 0 && gain_setting_ != gain) {
                    if (setGain(handle, gain_setting_)) {
                        RCLCPP_INFO(this->get_logger(), "  Set gain to: %d", gain_setting_);
                    }
                }
                if (lld_setting_ > 0 && lld_setting_ != lld) {
                    if (setLLD(handle, lld_setting_)) {
                        RCLCPP_INFO(this->get_logger(), "  Set LLD to: %d", lld_setting_);
                    }
                }

                device_gains_.push_back(gain);
                device_biases_.push_back(bias);
                device_llds_.push_back(lld);

                // Create per-device publishers
                if (publish_individual_) {
                    DevicePublishers pubs;
                    std::string prefix = "kromek_" + serial_str + "/";
                    pubs.sum_counts = this->create_publisher<std_msgs::msg::Int64>(prefix + "sum_counts", 10);
                    pubs.raw = this->create_publisher<std_msgs::msg::UInt32MultiArray>(prefix + "raw", 10);
                    pubs.sum_spectrum = this->create_publisher<std_msgs::msg::UInt32MultiArray>(prefix + "sum_spectrum", 10);
                    device_publishers_[serial_str] = pubs;
                }

                // Initialize histograms for this device
                device_histograms_[serial_str].resize(DET_CHANNELS, 0);
                device_accumulated_histograms_[serial_str].resize(DET_CHANNELS, 0);

            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to open device");
            }

            cur_dev = cur_dev->next;
        }

        hid_free_enumeration(devs);

        if (device_handles_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No Kromek Sigma50 devices found!");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully opened %zu Kromek devices", device_handles_.size());

        // Publish device info
        publishDeviceInfo();

        return true;
    }

    void publishDeviceInfo()
    {
        auto msg = std_msgs::msg::UInt32MultiArray();
        msg.data.resize(device_serials_.size());

        for (size_t i = 0; i < device_serials_.size(); i++) {
            try {
                msg.data[i] = std::stoi(device_serials_[i]);
            } catch (...) {
                msg.data[i] = i;
            }
        }

        info_pub_->publish(msg);
    }

    void publishDeviceStatus()
    {
        auto msg = std_msgs::msg::UInt32MultiArray();
        msg.data.resize(device_handles_.size() * 3);

        for (size_t i = 0; i < device_handles_.size(); i++) {
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
    int readGain(hid_device *handle) {
        unsigned char buf[2];
        buf[0] = GAIN_GET_REPORT;
        int res = hid_get_feature_report(handle, buf, 2);
        return (res > 0) ? buf[1] : -1;
    }

    int readBias(hid_device *handle) {
        unsigned char buf[3];
        buf[0] = BIAS_GET_REPORT;
        int res = hid_get_feature_report(handle, buf, 3);
        if (res > 0) {
            return ((buf[1] << 4) & 0xFF0) + ((buf[2] >> 4) & 0xF);
        }
        return -1;
    }

    int readLLD(hid_device *handle) {
        unsigned char buf[3];
        buf[0] = LLD_GET_REPORT;
        int res = hid_get_feature_report(handle, buf, 3);
        if (res > 0) {
            return ((buf[1] << 4) & 0xFF0) + ((buf[2] >> 4) & 0xF);
        }
        return -1;
    }

    bool setGain(hid_device *handle, int gain) {
        if (gain < 20 || gain > 250) return false;
        unsigned char buf[2];
        buf[0] = GAIN_SET_REPORT;
        buf[1] = static_cast<unsigned char>(gain);
        int res = hid_send_feature_report(handle, buf, 2);
        return res > 0;
    }

    bool setLLD(hid_device *handle, int lld) {
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
            std::string serial_str;
            for (wchar_t c : device_serials_[dev_idx]) {
                serial_str += static_cast<char>(c);
            }

            unsigned char buf[REPORT_SIZE + 1];
            
            while (true) {
                int res = hid_read_timeout(device_handles_[dev_idx], buf, REPORT_SIZE, 10);

                if (res == 0) {
                    break;
                } else if (res < 0) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                        "Error reading from device %zu", dev_idx);
                    break;
                }

                int offset = 1;
                while (offset + 1 < res && (buf[offset + 1] & 0x1) == 1) {
                    uint16_t channel = ((buf[offset] << 4) & 0xFF0) + ((buf[offset + 1] >> 4) & 0xF);
                    if (channel < DET_CHANNELS) {
                        device_histograms_[serial_str][channel]++;
                        device_accumulated_histograms_[serial_str][channel]++;
                        total_counts++;
                    }
                    offset += 2;
                }
            }
        }

        // Publish combined data FIRST (before individual histograms are cleared)
        if (publish_combined_) {
            publishCombinedData();
        }

        // Publish individual device data (this clears histograms)
        if (publish_individual_) {
            for (const auto& [serial, pubs] : device_publishers_) {
                publishDeviceData(serial, pubs);
            }
        }

        // Publish device status if enabled
        if (publish_device_status_) {
            publishDeviceStatus();
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                            "Total counts this period: %lu", total_counts);
    }

    void publishDeviceData(const std::string& serial, const DevicePublishers& pubs)
    {
        auto& hist = device_histograms_[serial];
        auto& acc_hist = device_accumulated_histograms_[serial];

        // Sum counts for this device
        uint64_t counts = 0;
        for (auto c : hist) counts += c;

        // Publish counts
        if (publish_sum_) {
            auto msg = std_msgs::msg::Int64();
            msg.data = counts;
            pubs.sum_counts->publish(msg);
        }

        // Publish histograms
        if (publish_histogram_) {
            // Raw (current period)
            auto raw_msg = createHistogramMessage(hist);
            pubs.raw->publish(raw_msg);

            // Accumulated
            auto sum_msg = createHistogramMessage(acc_hist);
            pubs.sum_spectrum->publish(sum_msg);
        }

        // Clear period histogram
        std::fill(hist.begin(), hist.end(), 0);
    }

    void publishCombinedData()
    {
        // Combine all device histograms
        std::vector<uint32_t> combined_hist(DET_CHANNELS, 0);
        std::vector<uint32_t> combined_acc_hist(DET_CHANNELS, 0);

        for (const auto& [serial, hist] : device_histograms_) {
            for (size_t i = 0; i < DET_CHANNELS; i++) {
                combined_hist[i] += hist[i];
                combined_acc_hist[i] += device_accumulated_histograms_[serial][i];
            }
        }

        // Sum counts
        uint64_t counts = 0;
        for (auto c : combined_hist) counts += c;

        // Publish
        if (publish_sum_) {
            auto msg = std_msgs::msg::Int64();
            msg.data = counts;
            combined_sum_counts_pub_->publish(msg);
        }

        if (publish_histogram_) {
            auto raw_msg = createHistogramMessage(combined_hist);
            combined_raw_pub_->publish(raw_msg);

            auto sum_msg = createHistogramMessage(combined_acc_hist);
            combined_sum_spectrum_pub_->publish(sum_msg);
        }
    }

    std_msgs::msg::UInt32MultiArray createHistogramMessage(const std::vector<uint32_t>& histogram)
    {
        auto msg = std_msgs::msg::UInt32MultiArray();
        msg.data.resize(num_bins_, 0);

        int rebin_factor = DET_CHANNELS / num_bins_;
        for (int i = 0; i < num_bins_; i++) {
            uint32_t sum = 0;
            for (int j = 0; j < rebin_factor; j++) {
                int channel_idx = i * rebin_factor + j;
                if (channel_idx < DET_CHANNELS) {
                    sum += histogram[channel_idx];
                }
            }
            msg.data[i] = sum;
        }

        return msg;
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
    bool publish_individual_;
    bool publish_combined_;

    // Device state
    std::vector<hid_device*> device_handles_;
    std::vector<std::wstring> device_serials_;
    std::vector<int> device_gains_;
    std::vector<int> device_biases_;
    std::vector<int> device_llds_;

    // Per-device data
    std::map<std::string, std::vector<uint32_t>> device_histograms_;
    std::map<std::string, std::vector<uint32_t>> device_accumulated_histograms_;
    std::map<std::string, DevicePublishers> device_publishers_;

    // Combined publishers
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr combined_sum_counts_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr combined_sum_spectrum_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr combined_raw_pub_;
    
    rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr info_pub_;
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
