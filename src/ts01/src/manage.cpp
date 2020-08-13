#include "../include/ts01/manage.hpp"

Manager::Manager(const rclcpp::NodeOptions &options)
    : Manager("", options) {}

Manager::Manager(const std::string &name_space,
                 const rclcpp::NodeOptions &options)
    : Node("ts_01_manager", name_space, options)
{
    // Initialize default demo parameters
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;

    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    // Publish
    publisher_status_ = this->create_publisher<std_msgs::msg::Bool>("ts01_status", qos);
    publisher_encoder_ = this->create_publisher<sensor_msgs::msg::JointState>("ts01_encoder", qos);
    publisher_di_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("ts01_di", qos);
    publisher_ai_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("ts01_ai", qos);
    publisher_count_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("ts01_counter", qos);
    std::cout << "Publish below topic" << std::endl;
    std::cout << "  /ts01_status" << std::endl;
    std::cout << "  /ts01_encoder" << std::endl;
    std::cout << "  /ts01_di" << std::endl;
    std::cout << "  /ts01_ai" << std::endl;
    std::cout << "  /ts01_counter" << std::endl;

    // Subscribe
    subscription_stage_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "xyz_stage/move", qos, std::bind(&Manager::topic_callback_stage, this, std::placeholders::_1));
    subscription_pullout_ = this->create_subscription<std_msgs::msg::Bool>(
        "pull_out", qos, std::bind(&Manager::topic_callback_pullout, this, std::placeholders::_1));
    std::cout << "Subscribe below topic" << std::endl;
    std::cout << "  /xyz_stage/move" << std::endl;
    std::cout << "  /pull_out" << std::endl;
}

void Manager::topic_callback_stage(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::cout << "I get msg: " << msg->data[0] << std::endl;
}

void Manager::topic_callback_pullout(const std_msgs::msg::Bool::SharedPtr msg)
{
    std::cout << "I get msg: " << msg->data << std::endl;
}

void Manager::initialize()
{
    //TS01の状態に関するPublish用msg
    msg_status.set__data(false);

    std::cout << "initial" << std::endl;
    //エンコーダーに関するPublish用msg
    msg_encoder.position.resize(ADOF);
    for (size_t i = 0; i < ADOF; ++i)
    {
        msg_encoder.position.push_back(0.0);
    }

    //DIに関するPublish用msg
    msg_di.data.resize(DIGITAL_INPUT);
    for (size_t i = 0; i < msg_di.data.size(); ++i)
    {
        msg_di.data[i] = 0;
    }

    // AIに関するPublish用msg
    msg_ai.data.resize(ANALOG_INPUT);
    for (size_t i = 0; i < msg_ai.data.size(); ++i)
    {
        msg_ai.data[i] = 0.0;
    }

    // countに関するPublish用msg
    msg_count.data.resize(COUNT_INPUT);
    for (size_t i = 0; i < msg_count.data.size(); ++i)
    {
        msg_count.data[i] = 0.0;
    }

    // 初期状態をとりあえずpublish
    this->publish();

    // TS-01へ接続
    RCLCPP_INFO(this->get_logger(), "Waiting for opening TS01");
    eyeexplorer.init_module();

    // TS-01がopen
    RCLCPP_INFO(this->get_logger(), "TS01 is opened");
    eyeexplorer.ts01.start_sampling(1000);

    msg_status.data = true;
}

void Manager::readData()
{
    eyeexplorer.ts01.read_autosampling_data(&eyeexplorer.input);
}

void Manager::setMessage()
{
    //エンコーダ
    int enc[ADOF];
    enc[0] = eyeexplorer.shift_range(eyeexplorer.input.ssi[0] >> 1, 0x0000fffff); //-2^19~2^19
    enc[1] = eyeexplorer.shift_range(eyeexplorer.input.ssi[1] >> 1, 0x00007ffff); //-2^17~2^17
    enc[2] = eyeexplorer.shift_range(eyeexplorer.input.ssi[2] >> 1, 0x00007ffff);
    enc[3] = eyeexplorer.shift_range(eyeexplorer.input.ssi[3] >> 1, 0x00003ffff); //
    enc[4] = eyeexplorer.shift_range(eyeexplorer.input.ssi[4] >> 1, 0x00003ffff); //
    for (size_t i = 0; i < ADOF; ++i)
    {
        msg_encoder.position[i] = eyeexplorer.RQ[i] * enc[i];
        // RCLCPP_INFO(node_logger, "encoder #%zd = %f, enc = %d", i, msg_encoder_.position[i], enc[i]);
    }
    msg_encoder.header.stamp = this->now();

    //DI
    for (size_t i = 0; i < msg_di.data.size(); i++)
    {
        //RCLCPP_INFO(node_logger, "DI #%zd = %d",i, eyeexplorer.input.din[i]);
        msg_di.data[i] = eyeexplorer.input.din[i];
    }

    //AI
    for (size_t i = 0; i < msg_ai.data.size(); i++)
    {
        //RCLCPP_INFO(node_logger, "AI #%zd = %f", i, eyeexplorer.input.v[i]);
        msg_ai.data[i] = eyeexplorer.input.v[i];
    }

    // Count
    for (size_t i = 0; i < msg_count.data.size(); i++)
    {
        //RCLCPP_INFO(node_logger, "count #%zd = %f", i, eyeexplorer.input.count[i]);
        msg_count.data[i] = eyeexplorer.input.count[i];
    }
}

void Manager::publish()
{
    //publish
    publisher_status_->publish(msg_status);
    publisher_encoder_->publish(msg_encoder);
    publisher_di_->publish(msg_di);
    publisher_ai_->publish(msg_ai);
    publisher_count_->publish(msg_count);
}

void Manager::detatch()
{
    eyeexplorer.cleanup_module();
}