#include <centauro_ros_nav/NavCtrlManager.h>

NavCtrlManager::NavCtrlManager ( std::string ns, double rate )
//   : _nh ( ns ),
{
    //initialization ROS node

    ROS_INFO_STREAM ("I am initializing the Node...");
    initROSNode(rate);

    ROS_INFO_STREAM ("Init Params...");
    initParams();

    //initialization done
    ROS_INFO_STREAM ("Initialization done.");
}


void NavCtrlManager::initROSNode(double rate){
    // init ROS node
    _rate = rate;
    _period = 1.0 / _rate;
    _timer = _nh.createTimer(ros::Duration(_period), &NavCtrlManager::main_loop, this, false, false);
    _time = 0.0;

    _ask_for_plan_srv = _nh.advertiseService("/ask_for_nav_plan", &NavCtrlManager::askNavPlan, this);

    _make_plan_srv = _nh.serviceClient<nav_msgs::GetPlan>("move_base/make_plan");
}

void NavCtrlManager::initParams(){
    _start.header.frame_id = "fixed_frame";
    _goal.target_pose.header.frame_id = "fixed_frame";
}

bool NavCtrlManager::askNavPlan(centauro_ros_nav::AskNavPlan::Request  &req, centauro_ros_nav::AskNavPlan::Response &res){

    ROS_WARN("Request received");
    _start.header.stamp = ros::Time::now();
    _goal.target_pose.header.stamp = ros::Time::now();

    //Fill Start Pos
    _start.pose.position = req.start.position;
    _start.pose.orientation = req.start.orientation;
    _planned_trajctory.request.start = _start;

    //Fill Goal Pos
    _goal.target_pose.pose.position = req.goal.position;
    _goal.target_pose.pose.orientation = req.goal.orientation;
    _planned_trajctory.request.goal = _goal.target_pose;

    ROS_WARN("From %f, %f TO %f, %f", _planned_trajctory.request.start.pose.position.x, _planned_trajctory.request.start.pose.position.y, _planned_trajctory.request.goal.pose.position.x, _planned_trajctory.request.goal.pose.position.y);

    res.completed = true;

    //Check if feasible
    if(!_make_plan_srv.call(_planned_trajctory))
        res.completed = false;

    if(_planned_trajctory.response.plan.poses.size() == 0)
        res.completed = false;

    ROS_WARN("Is feasbile? %d", res.completed);

    if(!res.completed)
        return false;

    //If feasible --> Execute !
    //TODO IMPROVE
    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
        std::cout << "Waiting for the move_base action server to come up" << std::endl;
    }

    // Cancel all previous goals and send the new one
    ac.cancelAllGoals();
    sleep(0.5);
    ac.sendGoal(_goal);
    std::cout << "Goal Sent !!" << std::endl;

    //Wait 40 seconds
    ac.waitForResult(ros::Duration(40.0));

    std::cout << "Final State: " << ac.getState().toString() << std::endl;

    if(ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED){
        ac.cancelAllGoals();
        sleep(0.5);
        ac.sendGoal(_goal);
        std::cout << "Goal Sent !!" << std::endl;

        //Wait 40 seconds
        ac.waitForResult(ros::Duration(40.0));
    }

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ac.cancelAllGoals();
        std::cout << "Hooray, the base moved to the goal" << std::endl;
        res.completed = true;
    }
    else{
        ac.cancelAllGoals();
        std::cout << "The base failed to move to the goal in 40 seconds" << std::endl;
        res.completed = false;
    }

    return res.completed;
}

void NavCtrlManager::main_loop(const ros::TimerEvent& timer){


    _time += _period; // update time
}

void NavCtrlManager::spin()
{
    _timer.start();
    //ROS_INFO_STREAM("ForceEstim started looping time " << 1.0/_period << "Hz");
    ros::spin();
}


NavCtrlManager::~NavCtrlManager(){
}
