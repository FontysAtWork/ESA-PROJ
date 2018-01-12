#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionserver_tutorial/FibonacciAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
class FibonacciAction
{
  protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<actionserver_tutorial::FibonacciAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    actionserver_tutorial::FibonacciFeedback feedback_;
    actionserver_tutorial::FibonacciResult result_;

  public:
    FibonacciAction(std::string name) : as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
                                        action_name_(name)
    {
        as_.start();
    }

    ~FibonacciAction(void)
    {
    }

    void executeCB(const actionserver_tutorial::FibonacciGoalConstPtr &goal)
    {
        // helper variables
        ros::Rate r(1);
        bool success = true;

        // push_back the seeds for the fibonacci sequence
        feedback_.sequence.clear();
        feedback_.sequence.push_back(0);
        feedback_.sequence.push_back(1);

        // publish info to the console for the user
        ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

        // start executing the action
        for (int i = 1; i <= goal->order; i++)
        {
            // check that preempt has not been requested by the client
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                success = false;
                break;
            }
            feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i - 1]);
            // publish the feedback
            as_.publishFeedback(feedback_);
            // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep();
        }

        if (success)
        {
            result_.sequence = feedback_.sequence;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fibonacci");

    MoveBaseClient ac("move_base", true);
    FibonacciAction fibonacci("fibonacci");

    /*while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }*/
/*
    std::vector<move_base_msgs::MoveBaseGoal> goals = createNavPoints();

    ROS_INFO("Sending goals");

    for (auto g : goals)
    {
        g.target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(g);
        ac.waitForResult();

        if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_WARN("The base failed to move");
        }
    }
    */
    ROS_INFO("hallo");
    ros::spin();

    return 0;
}
