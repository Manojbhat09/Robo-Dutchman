/*
 * arm_teleop_config_space.cpp
 *
 * This file allows the a joystick to interface with
 * the 4 dof hebi arm
 *
 * see http://wiki.ros.org/joy
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <hebiros/EntryListSrv.h>
#include <hebiros/AddGroupFromNamesSrv.h>
#include <hebiros/SizeSrv.h>

#define NODE_NAME "arm_teleop_config_space_node"
#define BUFFER_SIZE 100

// Hebi parameters
#define GROUP_NAME "arm_teleop_config_space_hebi_group"
#define FAMILY_NAME "robodutchman"
#define NAME_1 "shoulder"
#define NAME_2 "elbow"
#define NAME_3 "wrist1"
#define NAME_4 "wrist2"

// Degrees of freedom of the robot arm
#define DOF 4

// There are two modes of control
//      - one for moving the wrist
//      - one for moving the arm
#define MODE_ARM true
#define MODE_WRIST false
#define DEFAULT_MODE MODE_ARM

#define DEFAULT_PRIMARY_AXIS 1
#define DEFAULT_SECONDARY_AXIS 4
#define DEFAULT_MODE_BUTTON
// The control scale for each of the joints
#define DEFAULT_SCALE_1 (1.0/100)
#define DEFAULT_SCALE_2 (1.0/100)
#define DEFAULT_SCALE_3 (1.0/100)
#define DEFAULT_SCALE_4 (1.0/100)

class TeleopArm {
    public:
    TeleopArm(ros::NodeHandle nh) {
        this.nh = nh;

        // Get set parameter values
        nh.param("button_mode",  this.btn_mode, DEFAULT_MODE_BUTTON);
        nh.param("axis_primary", this.axis_primary, DEFAULT_PRIMARY_AXIS);
        nh.param("axis_secondary", this.axis_secondary, DEFAULT_SECONDARY_AXIS);
        nh.param("control_scale_1", this.scale[0], DEFAULT_SCALE_1);
        nh.param("control_scale_2", this.scale[1], DEFAULT_SCALE_2);
        nh.param("control_scale_3", this.scale[2], DEFAULT_SCALE_3);
        nh.param("control_scale_4", this.scale[3], DEFAULT_SCALE_4);

        this.recievedFirstHebiFb = false;
        this.recievedFirstJoyFb = false;
        this.mode = DEFAULT_MODE;

        this.hebiLookup();
        this.registerSubsAndPubs();
        this.initializeHebiCmdAndDb();
    }

    void step() {
        // Don't do anything until we have heard from
        // HEBI and the joysticks
        if (!this.recievedFirstHebiFb || !this.recievedFirstJoyFb) return;

        // Update target hebi positions from primary and secondary actuators
        this.hebi_cmd.position[this.primary_actuator] +=
            this.joy_fb->axes[this.axis_primary]
            * this.primary_scale;

        this.hebi_cmd.position[this.secondary_actuator] +=
            this.joy_fb->axes[this.secondary_primary]
            * this.secondary_scale;

        // Send hebi command
        this.hebi_cmd_pub.publish(this.hebi_cmd);
    }

    private:
    bool recievedFirstHebiFb;
    bool recievedFirstJoyFb;
    bool mode;

    ros::NodeHandle nh;

    // Messages
    sensor_msgs::Joy joy_fb;
    sensor_msgs::Joy joy_fb_prev;
    sensor_msgs::JointState hebi_fb;
    sensor_msgs::JointState hebi_cmd;

    // Joystick control axis
    int btn_mode;
    int axis_primary, axis_secondary;
    int primary_actuator, secondary_actuator;
    double primary_scale, secondary_scale;
    double scale[DOF];

    // Scale of control for each axis
    int control_scale[DOF];

    // Publishers and subscribers
    ros::Publisher hebi_cmd_pub;
    ros::Subscriber hebi_fb_sub;
    ros::Subscriber joy_fb_sub;

    void hebi_fb_callback(sensor_msgs::JointState data) {
        this.hebi_fb = data;
        if(!recievedFirstHebiFb) {
            this.hebi_cmd = data;
            recievedFirstHebiFb = true;
        }
    }

    void joy_fb_callback(sensor_msgs::Joy data) {
        this.joy_fb_prev = this.joy_fb;
        this.joy_fb = data;
        recievedFirstJoyFb = true;

        // Detect mode switch on button edge
        if (this.joy_fb->buttons[this.btn_mode]
                && ~this.joy_fb_prev->buttons[this.btn_mode]) {

            this.mode = !this.mode;

            // Set primary and secondary actuators and scales
            // based on current mode
            if (this.mode == MODE_ARM) {
                this.primary_actuator = 1;
                this.primary_scale = this.scale[0];

                this.secondary_actuator = 2;
                this.secondary_scale = this.scale[1];
            } else if (this.mode == MODE_WRIST) {
                this.primary_actuator = 3;
                this.primary_scale = this.scale[2];

                this.secondary_actuator = 4;
                this.secondary_scale = this.scale[3];
            }
        }
    }

    void hebiLookup() {
        //This code is taken from hebiros basic example 1

        //Create a client which uses the service to see the entry list of modules
        ros::ServiceClient entry_list_client =
            this.nh.serviceClient<EntryListSrv>(
            "/hebiros/entry_list");

        //Create a client which uses the service to create a group
        ros::ServiceClient add_group_client =
            this.nh.serviceClient<AddGroupFromNamesSrv>(
            "/hebiros/add_group_from_names");

        //Create a client which uses the service to find the size of a group
        ros::ServiceClient size_client =
            this.nh.serviceClient<SizeSrv>(
            "/hebiros/"+GROUP_NAME+"/size");

        EntryListSrv entry_list_srv;
        AddGroupFromNamesSrv add_group_srv;
        SizeSrv size_srv;

        //Call the entry_list service, displaying each module on the network
        //entry_list_srv.response.entry_list will now be populated with those modules
        entry_list_client.call(entry_list_srv);

        //Construct a group
        add_group_srv.request.group_name = GROUP_NAME;
        add_group_srv.request.names = {NAME_1,MANE_2,NAME_3,NAME_4};
        add_group_srv.request.families = {FAMILY_NAME};
        //Call the add_group_from_urdf service to create a group until it succeeds
        //Specific topics and services will now be available under this group's namespace
        //The group will exist until this node shutsdown
        while(!add_group_client.call(add_group_srv)) {}

        //Call the size service for the newly created group
        size_client.call(size_srv);
        ROS_INFO("%s has been created and has size %d",
                group_name.c_str(), size_srv.response.size);
    }

    void registerSubsAndPubs() {
        this.hebi_fb_sub= this.nh.subscribe(
            "/hebiros/"+GROUP_NAME+"/feedback/joint_state",
            BUFFER_SIZE, hebi_fb_callback);

        this.joy_fb_sub = this.nh.subscribe(
            "joy", BUFFER_SIZE, joy_fb_callback);

        this.hebi_cmd_pub = this.nh.advertise<sensor_msgs::JointState>(
            "/hebiros/"+GROUP_NAME+"/command/joint_state", BUFFER_SIZE);
    }

    void initializeHebiCmdAndFb() {
        this.hebi_cmd.name.push_back(FAMILY+"/"+NAME_1);
        this.hebi_cmd.name.push_back(FAMILY+"/"+NAME_2);
        this.hebi_cmd.name.push_back(FAMILY+"/"+NAME_3);
        this.hebi_cmd.name.push_back(FAMILY+"/"+NAME_4);
        this.hebi_cmd.position.resize(DOF);
        this.hebi_fb.position.reserve(DOF);
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh;

    TeleopArm t(nh);
    ros::Rate loop_rate(200):

    while(ros::ok()) {
        t.step();
        rps::spinOnce();
        loop_rate.sleep();
    }
}
