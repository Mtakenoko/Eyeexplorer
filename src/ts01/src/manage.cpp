#include "../include/ts01/manage.hpp"

Manager::Manager(const rclcpp::NodeOptions &options)
    : Manager("", options)
{
    // Initialize default demo parameters
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;

    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    // Publish
    publisher_status_ = this->create_publisher<std_msgs::msg::Bool>(TOPIC_TS01_STATUS, qos);
    publisher_encoder_ = this->create_publisher<sensor_msgs::msg::JointState>(TOPIC_TS01_ENCODER, qos);
    publisher_di_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(TOPIC_TS01_DIN, qos);
    publisher_ai_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(TOPIC_TS01_AIN, qos);
    publisher_count_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(TOPIC_TS01_COUNTER, qos);

    // Subscribe
    subscription_dout_ = this->create_subscription<std_msgs::msg::Bool>(
        TOPIC_TS01_DOUT, qos, std::bind(&Manager::topic_callback_dout, this, std::placeholders::_1));
    subscription_aout_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        TOPIC_TS01_AOUT, qos, std::bind(&Manager::topic_callback_aout, this, std::placeholders::_1));
}

Manager::Manager(const std::string &name_space,
                 const rclcpp::NodeOptions &options)
    : Node("ts01_manager", name_space, options)
{
    // Initialize default demo parameters
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;

    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    // Publish
    publisher_status_ = this->create_publisher<std_msgs::msg::Bool>(TOPIC_TS01_STATUS, qos);
    publisher_encoder_ = this->create_publisher<sensor_msgs::msg::JointState>(TOPIC_TS01_ENCODER, qos);
    publisher_di_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(TOPIC_TS01_DIN, qos);
    publisher_ai_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(TOPIC_TS01_AIN, qos);
    publisher_count_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(TOPIC_TS01_COUNTER, qos);

    // Subscribe
    subscription_dout_ = this->create_subscription<std_msgs::msg::Bool>(
        TOPIC_TS01_DOUT, qos, std::bind(&Manager::topic_callback_dout, this, std::placeholders::_1));
    subscription_aout_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        TOPIC_TS01_AOUT, qos, std::bind(&Manager::topic_callback_aout, this, std::placeholders::_1));
}

void Manager::topic_callback_dout(const std_msgs::msg::Bool::SharedPtr msg)
{
    std::cout << "I get dout msg: " << msg->data << std::endl;

    for (int i = 0; i < TS01_DO_CH_NUM; i++)
    {
        this->dout[i] = msg->data;
    }
}

void Manager::topic_callback_aout(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::cout << "I get aout msg: " << msg->data[0] << std::endl;
    for (int i = 0; i < TS01_AO_CH_NUM; i++)
    {
        this->aout[i] = msg->data[i];
    }
}

int Manager::initialize()
{
    //TS01の状態に関するPublish用msg
    msg_status.set__data(false);

    //エンコーダーに関するPublish用msg
    msg_encoder.position.resize(ADOF);
    for (size_t i = 0; i < ADOF; ++i)
    {
        msg_encoder.position[i] = 0.0;
    }

    //DIに関するPublish用msg
    msg_di.data.resize(TS01_DI_CH_NUM);
    for (size_t i = 0; i < msg_di.data.size(); ++i)
    {
        msg_di.data[i] = 0;
    }

    // AIに関するPublish用msg
    msg_ai.data.resize(TS01_AI_CH_NUM);
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

    // DOに関するメンバ変数の初期化
    for (int i = 0; i < TS01_DO_CH_NUM; i++)
    {
        this->dout[i] = false;
    }

    // AOに関するメンバ変数の初期化
    for (int i = 0; i < TS01_AO_CH_NUM; i++)
    {
        this->aout[i] = 5.0;
    }

    // 初期状態をとりあえずpublish
    this->publish();

    // TS-01へ接続
    int opened = eyeexplorer.init_module();
    if (opened == 1)
    {
        msg_status.data = true;
    }
    return opened;
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
        // RCLCPP_INFO(this->get_logger(), "encoder #%zd = %f", i, eyeexplorer.RQ[i] * enc[i]);
    }
    msg_encoder.header.stamp = this->now();

    //DI
    for (size_t i = 0; i < msg_di.data.size(); i++)
    {
        // RCLCPP_INFO(this->get_logger(), "DI #%zd = %d",i, eyeexplorer.input.din[i]);
        msg_di.data[i] = eyeexplorer.input.din[i];
    }

    //AI
    for (size_t i = 0; i < msg_ai.data.size(); i++)
    {
        //RCLCPP_INFO(this->get_logger(), "AI #%zd = %f", i, eyeexplorer.input.v[i]);
        msg_ai.data[i] = eyeexplorer.input.v[i];
    }

    // Count
    for (size_t i = 0; i < msg_count.data.size(); i++)
    {
        //RCLCPP_INFO(this->get_logger(), "count #%zd = %f", i, eyeexplorer.input.count[i]);
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

void Manager::outputData()
{
    for (int i = 0; i < TS01_DO_CH_NUM; i++)
    {
        eyeexplorer.output.dout[i] = this->dout[i];
    }

    for (int i = 0; i < TS01_AO_CH_NUM; i++)
    {
        eyeexplorer.output.u[i] = (double)this->aout[i];
    }
    eyeexplorer.ts01.write_data(&eyeexplorer.output);
}

void Manager::detatch()
{
    eyeexplorer.cleanup_module();
}