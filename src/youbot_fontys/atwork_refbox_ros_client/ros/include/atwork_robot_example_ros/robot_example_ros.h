#define BOOST_DATE_TIME_POSIX_TIME_STD_CONFIG
#include <ros/ros.h>
#include <protobuf_comm/peer.h>

#include <atwork_pb_msgs/BeaconSignal.pb.h>
#include <atwork_pb_msgs/BenchmarkState.pb.h>
#include <atwork_pb_msgs/ConveyorBelt.pb.h>
#include <atwork_pb_msgs/Inventory.pb.h>
#include <atwork_pb_msgs/TaskInfo.pb.h>
#include <atwork_pb_msgs/RobotInfo.pb.h>
#include <atwork_pb_msgs/RobotStatusReport.pb.h>
#include <atwork_pb_msgs/Time.pb.h>
#include <atwork_pb_msgs/VersionInfo.pb.h>
#include <atwork_pb_msgs/LoggingStatus.pb.h>

//publisher
#include <atwork_ros_msgs/BenchmarkState.h>
#include <atwork_ros_msgs/TriggeredConveyorBeltStatus.h>
#include <atwork_ros_msgs/Inventory.h>
#include <atwork_ros_msgs/TaskInfo.h>
#include "std_msgs/String.h"

// subscribers
#include <atwork_ros_msgs/TriggeredConveyorBeltCommand.h>
#include <atwork_ros_msgs/LoggingStatus.h>
#include <atwork_ros_msgs/RobotStatusReport.h>

#include <boost/asio.hpp>
#include <boost/date_time.hpp>
#include <unistd.h>

#include <sstream>

using namespace protobuf_comm;
using namespace atwork_pb_msgs;

class RobotExampleROS
{
    public:
        /**
         * Ctor.
         */
        RobotExampleROS(const ros::NodeHandle &nh);

        /**
         * Dtor.
         */
        ~RobotExampleROS();

        void readParameters();

        void initializeRobot();

        /**
         * Beacon Signal
         *
         * This function sends the beacon signal
         * The function is called from the main loop
         */
        void sendBeacon();

    private:
        /**
         * Copy Ctor.
         */
        RobotExampleROS(const RobotExampleROS &other);

        /**
         * Assignment operator
         */
        RobotExampleROS &operator=(const RobotExampleROS &other);

        /**
         * Handler for send errors.
         *
         * This handler is called for send errors.
         * Print the message that should be send as ROS Warning
         *
         */
        void handleSendError(std::string msg);

        /**
         * Handler for receive errors.
         *
         * This handler is called for receive errors.
         * Print the message that should be received as a ROS Warning
         *
         */
        void handleReceiveError(boost::asio::ip::udp::endpoint &endpoint,
                                std::string msg);

        /**
         * Handler for receive messages .
         *
         * This handler is called for receiving messages.
         * Main function in this node.
         *
         */
        void handleMessage(boost::asio::ip::udp::endpoint &sender,
                            uint16_t component_id, uint16_t msg_type,
                            std::shared_ptr<google::protobuf::Message> msg);

        void TriggeredConveyorBeltCommandCB(atwork_ros_msgs::TriggeredConveyorBeltCommand msg);

        void LoggingStatusCB(atwork_ros_msgs::LoggingStatus msg);

        void RobotStatusReportCB(atwork_ros_msgs::RobotStatusReport msg);

    private:
        /**
         * ROS node handle.
         */
        ros::NodeHandle nh_;

        /**
         * Stores sequence of the bacon messages.
         */
        unsigned long seq_;

        /**
         * The interface to the Protobuf Broadcast peer to communicate with Refbox.
         */
        std::shared_ptr<ProtobufBroadcastPeer> peer_public_;

        /**
         * The interface to the Protobuf Broadcast peer to communicate with Team.
         */
        std::shared_ptr<ProtobufBroadcastPeer> peer_team_;

        /**
         * Stores robot name.
         */
        std::string robot_name_;

        /**
         * Stores team name.
         */
        std::string team_name_;

        /**
         * Publishers
         */
        //ros::Publisher attention_message_pub_;

        ros::Publisher benchmark_state_pub_;

        ros::Publisher inventory_pub_;

        ros::Publisher task_info_pub_;

        ros::Publisher conveyor_belt_status_pub_;

        /**
         * Subscribers
         */
        ros::Subscriber conveyor_belt_command_sub_;

        ros::Subscriber logging_status_sub_;

        ros::Subscriber robot_status_report_sub_;

        /**
         * Parameter to check if refbox is running on local or another machine.
         */
        bool remote_refbox_;

        /**
         * Parameter to store host name(IP address).
         */
        std::string host_name_;

        /**
         * Stores port number on which refbox is receiving or sending messages
         * when the refbox is running on another machine. 
         */
        int public_port_;

        /**
         * Stores port number on which robot/team is receiving or sending messages. 
         * when the refbox is running on another machine.
         */
        int team_port_;

        /**
         * Stores port number on which refbox is sending messages. 
         * when the refbox is running on local machine.
         */
        int public_send_port_;

        /**
         * Stores port number on which refbox is receiving messages. 
         * when the refbox is running on local machine.
         */
        int public_recv_port_;
 
        /**
         * Stores port number on which the team is sending messages. 
         * when the refbox is running on local machine.
         */
        int team_send_port_;
 
        /**
         * Stores port number on which team is receiving messages. 
         * when the refbox is running on local machine.
         */
        int team_recv_port_;
};
