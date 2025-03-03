// C++
#include <fstream>
#include <string>

// yaml-cpp
#include <yaml-cpp/yaml.h>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "acc_lib/allan_acc.h"
#include "acc_lib/fitallan_acc.h"
#include "gyr_lib/allan_gyr.h"
#include "gyr_lib/fitallan_gyr.h"

class ImuCalib : public rclcpp::Node {
 public:
  explicit ImuCalib(const rclcpp::NodeOptions& options)
      : Node("imu_utils", options) {
    declare_parameter("imu_topic", "imu/data");
    get_parameter("imu_topic", imu_topic_);
    declare_parameter("imu_name", "xsens");
    get_parameter("imu_name", imu_name_);
    declare_parameter("bag_file", "/path/to/db3_file");
    get_parameter("bag_file", bag_file_);
    declare_parameter("data_save_path", "/path/to/data");
    get_parameter("data_save_path", data_save_path_);
    declare_parameter("max_cluster", 100);
    get_parameter("max_cluster", max_cluster_);

    if (*data_save_path_.end() != '/') data_save_path_ += "/";

    RCLCPP_INFO(this->get_logger(),
                "\033[0m\033[1;32mLoaded imu_topic: %s\033[0m",
                imu_topic_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "\033[0m\033[1;32mLoaded imu_name: %s\033[0m",
                imu_name_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "\033[0m\033[1;32mLoaded bag_file: %s\033[0m",
                bag_file_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "\033[0m\033[1;32mLoaded data_save_path: %s\033[0m",
                data_save_path_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "\033[0m\033[1;32mLoaded max_cluster: %d\033[0m", max_cluster_);

    gyr_x_ = std::make_shared<imu::AllanGyr>("gyr x", max_cluster_);
    gyr_y_ = std::make_shared<imu::AllanGyr>("gyr y", max_cluster_);
    gyr_z_ = std::make_shared<imu::AllanGyr>("gyr z", max_cluster_);
    acc_x_ = std::make_shared<imu::AllanAcc>("acc x", max_cluster_);
    acc_y_ = std::make_shared<imu::AllanAcc>("acc y", max_cluster_);
    acc_z_ = std::make_shared<imu::AllanAcc>("acc z", max_cluster_);
  }

  ~ImuCalib() {}

  void calib() {
    reader_.open(bag_file_);

    std::size_t imu_msg_count = 0;

    while (reader_.has_next()) {
      rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_.read_next();
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

      if (msg->topic_name == imu_topic_ ||
          msg->topic_name == "/" + imu_topic_) {
        sensor_msgs::msg::Imu::SharedPtr imu_msg =
            std::make_shared<sensor_msgs::msg::Imu>();
        imu_serialization_.deserialize_message(&serialized_msg, imu_msg.get());

        // logger
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_INFO_STREAM_THROTTLE(
            this->get_logger(), steady_clock, 3000,
            "add imu msg " + std::to_string(++imu_msg_count));

        double time = stamp2Sec(imu_msg->header.stamp);
        gyr_x_->pushRadPerSec(imu_msg->angular_velocity.x, time);
        gyr_y_->pushRadPerSec(imu_msg->angular_velocity.y, time);
        gyr_z_->pushRadPerSec(imu_msg->angular_velocity.z, time);
        acc_x_->pushMPerSec2(imu_msg->linear_acceleration.x, time);
        acc_y_->pushMPerSec2(imu_msg->linear_acceleration.y, time);
        acc_z_->pushMPerSec2(imu_msg->linear_acceleration.z, time);
      }
    }

    calib_impl();

    rclcpp::shutdown();
  }

 private:
  void calib_impl() {
    ///
    gyr_x_->calc();
    std::vector<double> gyro_v_x = gyr_x_->getVariance();
    std::vector<double> gyro_d_x = gyr_x_->getDeviation();
    std::vector<double> gyro_ts_x = gyr_x_->getTimes();

    gyr_y_->calc();
    std::vector<double> gyro_v_y = gyr_y_->getVariance();
    std::vector<double> gyro_d_y = gyr_y_->getDeviation();
    std::vector<double> gyro_ts_y = gyr_y_->getTimes();

    gyr_z_->calc();
    std::vector<double> gyro_v_z = gyr_z_->getVariance();
    std::vector<double> gyro_d_z = gyr_z_->getDeviation();
    std::vector<double> gyro_ts_z = gyr_z_->getTimes();

    std::cout << "Gyro x " << std::endl;
    imu::FitAllanGyr fit_gyr_x(gyro_v_x, gyro_ts_x, gyr_x_->getFreq());
    std::cout << "  bias " << gyr_x_->getAvgValue() / 3600 << " degree/s"
              << std::endl;
    std::cout << "-------------------" << std::endl;

    std::cout << "Gyro y " << std::endl;
    imu::FitAllanGyr fit_gyr_y(gyro_v_y, gyro_ts_y, gyr_y_->getFreq());
    std::cout << "  bias " << gyr_y_->getAvgValue() / 3600 << " degree/s"
              << std::endl;
    std::cout << "-------------------" << std::endl;

    std::cout << "Gyro z " << std::endl;
    imu::FitAllanGyr fit_gyr_z(gyro_v_z, gyro_ts_z, gyr_z_->getFreq());
    std::cout << "  bias " << gyr_z_->getAvgValue() / 3600 << " degree/s"
              << std::endl;
    std::cout << "-------------------" << std::endl;

    std::vector<double> gyro_sim_d_x = fit_gyr_x.calcSimDeviation(gyro_ts_x);
    std::vector<double> gyro_sim_d_y = fit_gyr_y.calcSimDeviation(gyro_ts_y);
    std::vector<double> gyro_sim_d_z = fit_gyr_z.calcSimDeviation(gyro_ts_z);

    writeData3(imu_name_ + "_sim_gyr", gyro_ts_x, gyro_sim_d_x, gyro_sim_d_y,
               gyro_sim_d_z);
    writeData3(imu_name_ + "_gyr", gyro_ts_x, gyro_d_x, gyro_d_y, gyro_d_z);

    std::cout << "==============================================" << std::endl;
    std::cout << "==============================================" << std::endl;

    acc_x_->calc();
    std::vector<double> acc_v_x = acc_x_->getVariance();
    std::vector<double> acc_d_x = acc_x_->getDeviation();
    std::vector<double> acc_ts_x = acc_x_->getTimes();

    acc_y_->calc();
    std::vector<double> acc_v_y = acc_y_->getVariance();
    std::vector<double> acc_d_y = acc_y_->getDeviation();
    std::vector<double> acc_ts_y = acc_y_->getTimes();

    acc_z_->calc();
    std::vector<double> acc_v_z = acc_z_->getVariance();
    std::vector<double> acc_d_z = acc_z_->getDeviation();
    std::vector<double> acc_ts_z = acc_z_->getTimes();

    std::cout << "acc x " << std::endl;
    imu::FitAllanAcc fit_acc_x(acc_v_x, acc_ts_x, acc_x_->getFreq());
    std::cout << "-------------------" << std::endl;

    std::cout << "acc y " << std::endl;
    imu::FitAllanAcc fit_acc_y(acc_v_y, acc_ts_y, acc_y_->getFreq());
    std::cout << "-------------------" << std::endl;

    std::cout << "acc z " << std::endl;
    imu::FitAllanAcc fit_acc_z(acc_v_z, acc_ts_z, acc_z_->getFreq());
    std::cout << "-------------------" << std::endl;

    std::vector<double> acc_sim_d_x = fit_acc_x.calcSimDeviation(acc_ts_x);
    std::vector<double> acc_sim_d_y = fit_acc_y.calcSimDeviation(acc_ts_x);
    std::vector<double> acc_sim_d_z = fit_acc_z.calcSimDeviation(acc_ts_x);

    double sensor_frequency =
        (gyr_x_->getFreq() + gyr_y_->getFreq() + gyr_z_->getFreq() +
         acc_x_->getFreq() + acc_y_->getFreq() + acc_z_->getFreq()) /
        6.0;

    writeData3(imu_name_ + "_sim_acc", acc_ts_x, acc_sim_d_x, acc_sim_d_y,
               acc_sim_d_z);
    writeData3(imu_name_ + "_acc", acc_ts_x, acc_d_x, acc_d_y, acc_d_z);

    writeYAML(data_save_path_, imu_name_, sensor_frequency, fit_gyr_x,
              fit_gyr_y, fit_gyr_z, fit_acc_x, fit_acc_y, fit_acc_z);
  }

  void writeData3(const std::string& sensor_name,
                  const std::vector<double>& gyro_ts_x,
                  const std::vector<double>& gyro_d_x,
                  const std::vector<double>& gyro_d_y,
                  const std::vector<double>& gyro_d_z) {
    std::ofstream out_t;
    std::ofstream out_x;
    std::ofstream out_y;
    std::ofstream out_z;
    out_t.open(data_save_path_ + "data_" + sensor_name + "_t.txt",
               std::ios::trunc);
    out_x.open(data_save_path_ + "data_" + sensor_name + "_x.txt",
               std::ios::trunc);
    out_y.open(data_save_path_ + "data_" + sensor_name + "_y.txt",
               std::ios::trunc);
    out_z.open(data_save_path_ + "data_" + sensor_name + "_z.txt",
               std::ios::trunc);
    out_t << std::setprecision(10);
    out_x << std::setprecision(10);
    out_y << std::setprecision(10);
    out_z << std::setprecision(10);

    for (std::size_t index = 0; index < gyro_ts_x.size(); ++index) {
      out_t << gyro_ts_x[index] << '\n';
      out_x << gyro_d_x[index] << '\n';
      out_y << gyro_d_y[index] << '\n';
      out_z << gyro_d_z[index] << '\n';
    }

    out_z.close();
    out_y.close();
    out_x.close();
    out_t.close();
  }

  void writeYAML(const std::string& data_path, const std::string& sensor_name,
                 double sensor_frequency, const imu::FitAllanGyr& gyr_x,
                 const imu::FitAllanGyr& gyr_y, const imu::FitAllanGyr& gyr_z,
                 const imu::FitAllanAcc& acc_x, const imu::FitAllanAcc& acc_y,
                 const imu::FitAllanAcc& acc_z) {
    auto FloatToScientific = [](double value) -> std::string {
      std::ostringstream oss;
      oss << std::setprecision(16) << std::scientific << value;
      return oss.str();
    };

    YAML::Node imu_params;
    imu_params["type"] = "IMU";
    imu_params["name"] = sensor_name;
    imu_params["frequency"] = sensor_frequency;
    imu_params["Gyr"]["unit"] = "rad/s";
    imu_params["Gyr"]["avg-axis"]["gyr_n"] =
        FloatToScientific((gyr_x.getWhiteNoise() + gyr_y.getWhiteNoise() +
                           gyr_z.getWhiteNoise()) /
                          3.0);
    imu_params["Gyr"]["avg-axis"]["gyr_w"] = FloatToScientific(
        (gyr_x.getBiasInstability() + gyr_y.getBiasInstability() +
         gyr_z.getBiasInstability()) /
        3.0);
    imu_params["Gyr"]["x-axis"]["gyr_n"] =
        FloatToScientific(gyr_x.getWhiteNoise());
    imu_params["Gyr"]["x-axis"]["gyr_w"] =
        FloatToScientific(gyr_x.getBiasInstability());
    imu_params["Gyr"]["y-axis"]["gyr_n"] =
        FloatToScientific(gyr_y.getWhiteNoise());
    imu_params["Gyr"]["y-axis"]["gyr_w"] =
        FloatToScientific(gyr_y.getBiasInstability());
    imu_params["Gyr"]["z-axis"]["gyr_n"] =
        FloatToScientific(gyr_z.getWhiteNoise());
    imu_params["Gyr"]["z-axis"]["gyr_w"] =
        FloatToScientific(gyr_z.getBiasInstability());
    imu_params["Acc"]["unit"] = "m/s^2";
    imu_params["Acc"]["avg-axis"]["acc_n"] =
        FloatToScientific((acc_x.getWhiteNoise() + acc_y.getWhiteNoise() +
                           acc_z.getWhiteNoise()) /
                          3.0);
    imu_params["Acc"]["avg-axis"]["acc_w"] = FloatToScientific(
        (acc_x.getBiasInstability() + acc_y.getBiasInstability() +
         acc_z.getBiasInstability()) /
        3.0);
    imu_params["Acc"]["x-axis"]["acc_n"] =
        FloatToScientific(acc_x.getWhiteNoise());
    imu_params["Acc"]["x-axis"]["acc_w"] =
        FloatToScientific(acc_x.getBiasInstability());
    imu_params["Acc"]["y-axis"]["acc_n"] =
        FloatToScientific(acc_y.getWhiteNoise());
    imu_params["Acc"]["y-axis"]["acc_w"] =
        FloatToScientific(acc_y.getBiasInstability());
    imu_params["Acc"]["z-axis"]["acc_n"] =
        FloatToScientific(acc_z.getWhiteNoise());
    imu_params["Acc"]["z-axis"]["acc_w"] =
        FloatToScientific(acc_z.getBiasInstability());

    std::ofstream ofs;
    ofs.open(data_path + sensor_name + "_imu_param.yaml");
    ofs << "%YAML:1.0" << std::endl;
    ofs << "---" << std::endl;
    ofs << imu_params;
    ofs.close();
  }

  template <typename T>
  double stamp2Sec(const T& stamp) {
    return rclcpp::Time(stamp).seconds();
  }

 private:
  std::shared_ptr<imu::AllanGyr> gyr_x_;
  std::shared_ptr<imu::AllanGyr> gyr_y_;
  std::shared_ptr<imu::AllanGyr> gyr_z_;
  std::shared_ptr<imu::AllanAcc> acc_x_;
  std::shared_ptr<imu::AllanAcc> acc_y_;
  std::shared_ptr<imu::AllanAcc> acc_z_;

  // pass by parameter server
  std::string imu_topic_;
  std::string imu_name_;
  std::string bag_file_;
  std::string data_save_path_;
  int max_cluster_;

  rosbag2_cpp::Reader reader_;
  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;

  auto imu_calib = std::make_shared<ImuCalib>(rclcpp::NodeOptions{});
  exec.add_node(imu_calib);

  std::thread calibthread(&ImuCalib::calib, imu_calib);

  exec.spin();

  calibthread.join();

  return 0;
}
