#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

// 모터 및 통신 설정 상수
const std::string DEVICENAME = "/dev/ttyUSB1";  // 모터 연결 포트
const int BAUDRATE = 115200;                    // 통신 속도
const float PROTOCOL_VERSION = 2.0;             // 프로토콜 버전
const int DXL_ID_DRIVE_1 = 1;                   // 첫 번째 모터 ID
const int DXL_ID_DRIVE_2 = 2;                   // 두 번째 모터 ID
const int TORQUE_ENABLE = 64;                   // 토크 활성화 주소
const int TORQUE_ENABLE_VALUE = 1;              // 토크 활성화 값
const int ADDR_PRO_GOAL_POSITION = 116;         // 모터 목표 위치 주소
const int ADDR_PRO_PRESENT_POSITION = 132;      // 모터 현재 위치 주소
const int CONTROL_FACTOR = 1023;                // 제어 신호 변환 상수

class PIDController {
public:
    PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), prev_error_(0), integral_(0) {}

    double compute(double error) {
        double p = kp_ * error;
        integral_ += error;
        double i = ki_ * integral_;
        double d = kd_ * (error - prev_error_);
        prev_error_ = error;
        return p + i + d;
    }

private:
    double kp_, ki_, kd_;
    double prev_error_;
    double integral_;
};

class BalanceController {
public:
    BalanceController(double kp, double ki, double kd)
    : pid_controller_(kp, ki, kd) {}

    double compute(double error) {
        return pid_controller_.compute(error);
    }

private:
    PIDController pid_controller_;
};

class ImuSubscriber : public rclcpp::Node {
public:
    ImuSubscriber()
    : Node("imu_subscriber"),
      balance_controller_(1.0, 0.1, 0.01)  // PID 파라미터를 적절하게 설정
    {
        motor_pub_ = this->create_publisher<std_msgs::msg::Float64>("/motor_command", 10);
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", rclcpp::SensorDataQoS(), std::bind(&ImuSubscriber::callback, this, std::placeholders::_1));

        portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME.c_str());
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        if (!portHandler_->openPort()) {
            RCLCPP_ERROR(this->get_logger(), "포트를 여는 데 실패했습니다!");
        } else {
            RCLCPP_INFO(this->get_logger(), "포트가 성공적으로 열렸습니다.");
        }

        if (!portHandler_->setBaudRate(BAUDRATE)) {
            RCLCPP_ERROR(this->get_logger(), "통신 속도를 설정하는 데 실패했습니다!");
        } else {
            RCLCPP_INFO(this->get_logger(), "통신 속도가 성공적으로 설정되었습니다.");
        }

        enableTorque(DXL_ID_DRIVE_1);
        enableTorque(DXL_ID_DRIVE_2);
    }

private:
    void enableTorque(int dxl_id) {
        RCLCPP_INFO(this->get_logger(), "모터 %d에 토크를 활성화합니다...", dxl_id);
        int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, dxl_id, TORQUE_ENABLE, TORQUE_ENABLE_VALUE, &dxl_error_);
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "모터 %d의 토크 활성화에 실패했습니다: %s", dxl_id, packetHandler_->getTxRxResult(dxl_comm_result));
        } else if (dxl_error_ != 0) {
            RCLCPP_ERROR(this->get_logger(), "모터 %d의 프로토콜 오류: %s", dxl_id, packetHandler_->getRxPacketError(dxl_error_));
        } else {
            RCLCPP_INFO(this->get_logger(), "모터 %d에 토크가 활성화되었습니다.", dxl_id);
        }
    }

    void callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
        RCLCPP_INFO(this->get_logger(), "IMU 데이터를 수신했습니다.");  // 디버깅 메시지 추가

        tf2::Quaternion quat(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);
        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        RCLCPP_INFO(this->get_logger(), "Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);  // 디버깅 메시지 추가

        double error_pitch = desired_pitch_ - pitch;
        RCLCPP_INFO(this->get_logger(), "Pitch 오차: %f", error_pitch);  // 디버깅 메시지 추가

        double control_signal_drive = balance_controller_.compute(error_pitch);
        RCLCPP_INFO(this->get_logger(), "제어 신호: %f", control_signal_drive);  // 디버깅 메시지 추가

        apply_control_signal(DXL_ID_DRIVE_1, control_signal_drive);
        apply_control_signal(DXL_ID_DRIVE_2, control_signal_drive);

        // 모터의 현재 위치를 읽어서 출력 (모터 1)
        int32_t present_position_1 = 0;
        int dxl_comm_result_1 = packetHandler_->read4ByteTxRx(portHandler_, DXL_ID_DRIVE_1, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&present_position_1, &dxl_error_);
        if (dxl_comm_result_1 != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "모터 %d의 현재 위치를 읽는 데 실패했습니다: %s", DXL_ID_DRIVE_1, packetHandler_->getTxRxResult(dxl_comm_result_1));
        } else if (dxl_error_ != 0) {
            RCLCPP_ERROR(this->get_logger(), "모터 %d의 프로토콜 오류: %s", DXL_ID_DRIVE_1, packetHandler_->getRxPacketError(dxl_error_));
        } else {
            RCLCPP_INFO(this->get_logger(), "모터 %d의 현재 위치: %d", DXL_ID_DRIVE_1, present_position_1);
        }

        // 모터의 현재 위치를 읽어서 출력 (모터 2)
        int32_t present_position_2 = 0;
        int dxl_comm_result_2 = packetHandler_->read4ByteTxRx(portHandler_, DXL_ID_DRIVE_2, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&present_position_2, &dxl_error_);
        if (dxl_comm_result_2 != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "모터 %d의 현재 위치를 읽는 데 실패했습니다: %s", DXL_ID_DRIVE_2, packetHandler_->getTxRxResult(dxl_comm_result_2));
        } else if (dxl_error_ != 0) {
            RCLCPP_ERROR(this->get_logger(), "모터 %d의 프로토콜 오류: %s", DXL_ID_DRIVE_2, packetHandler_->getRxPacketError(dxl_error_));
        } else {
            RCLCPP_INFO(this->get_logger(), "모터 %d의 현재 위치: %d", DXL_ID_DRIVE_2, present_position_2);
        }
    }

    void apply_control_signal(int dxl_id, double control_signal) {
        int32_t data = static_cast<int32_t>(control_signal * CONTROL_FACTOR);
        RCLCPP_INFO(this->get_logger(), "모터 %d에 제어 신호 적용: %d", dxl_id, data);
        int dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, dxl_id, ADDR_PRO_GOAL_POSITION, data, &dxl_error_);
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "모터 %d의 목표 위치 설정에 실패했습니다: %s", dxl_id, packetHandler_->getTxRxResult(dxl_comm_result));
        } else if (dxl_error_ != 0) {
            RCLCPP_ERROR(this->get_logger(), "모터 %d의 프로토콜 오류: %s", dxl_id, packetHandler_->getRxPacketError(dxl_error_));
        } else {
            RCLCPP_INFO(this->get_logger(), "모터 %d에 제어 신호가 적용되었습니다: %d", dxl_id, data);
        }

        std_msgs::msg::Float64 msg;
        msg.data = control_signal;
        motor_pub_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_pub_;        // 모터 명령 퍼블리셔
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_; // IMU 데이터 구독자
    BalanceController balance_controller_;                                  // 밸런스 컨트롤러
    double desired_pitch_ = 0.0;                                            // 목표로 하는 피치 값 (0도)
    dynamixel::PortHandler *portHandler_;                                   // 다이나믹셀 포트 핸들러
    dynamixel::PacketHandler *packetHandler_;                               // 다이나믹셀 패킷 핸들러
    uint8_t dxl_error_;                                                     // 다이나믹셀 오류 코드
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);  // ROS2 초기화
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ROS2 초기화 완료");
    auto node = std::make_shared<ImuSubscriber>();  // IMU 구독자 노드 생성
    RCLCPP_INFO(node->get_logger(), "노드 실행 중...");
    rclcpp::spin(node);  // 노드 실행
    rclcpp::shutdown();  // ROS2 종료
    return 0;
}
