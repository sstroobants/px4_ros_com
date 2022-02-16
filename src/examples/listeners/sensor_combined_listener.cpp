/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 *           2018 PX4 Pro Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Sensor Combined uORB topic listener example
 * @file sensor_combined_listener.cpp
 * @addtogroup examples
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 * @author Vicente Monge
 */

 #include <rclcpp/rclcpp.hpp>
 #include <px4_msgs/msg/sensor_combined.hpp>

/**
 * @brief Sensor Combined uORB topic data callback
 */
class SensorCombinedListener : public rclcpp::Node
{
public:
	explicit SensorCombinedListener() : Node("sensor_combined_listener") {
		auto qos = rclcpp::QoS(
			// The "KEEP_LAST" history setting tells DDS to store a fixed-size buffer of values before they
			// are sent, to aid with recovery in the event of dropped messages.
			// "depth" specifies the size of this buffer.
			// In this example, we are optimizing for performance and limited resource usage (preventing
			// page faults), instead of reliability. Thus, we set the size of the history buffer to 1.
			rclcpp::KeepLast(1)
		);
		// From http://www.opendds.org/qosusages.html: "A RELIABLE setting can potentially block while
		// trying to send." Therefore set the policy to best effort to avoid blocking during execution.
		qos.best_effort();
		subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
			"fmu/sensor_combined/out",
			qos,
			[this](const px4_msgs::msg::SensorCombined::UniquePtr msg) {
			// std::cout << "\n\n\n\n\n\n\n\n";
			// std::cout << "RECEIVED SENSOR COMBINED DATA"   << std::endl;
			// std::cout << "============================="   << std::endl;
			// std::cout << "\n\n";
			// std::cout << "ts: "          << msg->timestamp    << std::endl;
			uint64_t diff = msg->timestamp - t_prev;
			// std::cout << "s: "			 << diff << std::endl;
			t_prev = msg->timestamp;
			sum = sum + diff;
			count = count + 1;

			if (count == 500) {
				std::cout << "avg ms: " << sum / count << std::endl;
				sum = 0;
				count = 0;
			}
			// t_prev_f = (float)
			// std::cout << "gyro_rad[0]: " << msg->gyro_rad[0]  << std::endl;
			// std::cout << "gyro_rad[1]: " << msg->gyro_rad[1]  << std::endl;
			// std::cout << "gyro_rad[2]: " << msg->gyro_rad[2]  << std::endl;
			// std::cout << "gyro_integral_dt: " << msg->gyro_integral_dt << std::endl;
			// std::cout << "accelerometer_timestamp_relative: " << msg->accelerometer_timestamp_relative << std::endl;
			// std::cout << "accelerometer_m_s2[0]: " << msg->accelerometer_m_s2[0] << std::endl;
			// std::cout << "accelerometer_m_s2[1]: " << msg->accelerometer_m_s2[1] << std::endl;
			// std::cout << "accelerometer_m_s2[2]: " << msg->accelerometer_m_s2[2] << std::endl;
			// std::cout << "accelerometer_integral_dt: " << msg->accelerometer_integral_dt << std::endl;
		}
		);
	}

private:
	rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr subscription_;

	uint64_t t_prev = 0;
	uint64_t sum = 0;
	float t_prev_f = 0;
	uint64_t count = 0;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting sensor_combined listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SensorCombinedListener>());

	rclcpp::shutdown();
	return 0;
}
