package com.katiekilroy.smachbros;

import android.os.Bundle;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;

import com.google.common.collect.Lists;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.android.view.VirtualJoystickView;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.android.view.visualization.layer.CameraControlLayer;
import org.ros.android.view.visualization.layer.LaserScanLayer;
import org.ros.android.view.visualization.layer.Layer;
import org.ros.android.view.visualization.layer.OccupancyGridLayer;
import org.ros.android.view.visualization.layer.PathLayer;
import org.ros.android.view.visualization.layer.PosePublisherLayer;
import org.ros.android.view.visualization.layer.PoseSubscriberLayer;
import org.ros.android.view.visualization.layer.RobotLayer;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class MainActivity extends RosActivity {


    //publisher and subscriber to check connectivity

//    private RosTextView<std_msgs.String> rosTextView;
//    private Talker talker;
//
//    public MainActivity() {
//        // The RosActivity constructor configures the notification title and ticker
//        // messages.
//        super("Smach Bros", "Smach Bros", URI.create("http://localhost:11311"));
//    }
//
//    @SuppressWarnings("unchecked")
//    @Override
//    public void onCreate(Bundle savedInstanceState) {
//        super.onCreate(savedInstanceState);
//        setContentView(R.layout.activity_main);
//        rosTextView = (RosTextView<std_msgs.String>) findViewById(R.id.text);
//        rosTextView.setTopicName("chatter");
//        rosTextView.setMessageType(std_msgs.String._TYPE);
//        rosTextView.setMessageToStringCallable(new MessageCallable<String, std_msgs.String>() {
//            @Override
//            public String call(std_msgs.String message) {
//                return message.getData();
//            }
//        });
//    }
//
//    @Override
//    protected void init(NodeMainExecutor nodeMainExecutor) {
//        talker = new Talker();
//
//        // At this point, the user has already been prompted to either enter the URI
//        // of a master to use or to start a master locally.
//
//        // The user can easily use the selected ROS Hostname in the master chooser
//        // activity.
//        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(getRosHostname());
//        nodeConfiguration.setMasterUri(getMasterUri());
//        nodeMainExecutor.execute(talker, nodeConfiguration);
//        // The RosTextView is also a NodeMain that must be executed in order to
//        // start displaying incoming messages.
//        nodeMainExecutor.execute(rosTextView, nodeConfiguration);
//    }
//}






    private VirtualJoystickView virtualJoystickView;
    private VisualizationView visualizationView;

    public MainActivity() {
        super("Teleop", "Teleop");
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.settings_menu, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.virtual_joystick_snap:
                if (!item.isChecked()) {
                    item.setChecked(true);
                    virtualJoystickView.EnableSnapping();
                } else {
                    item.setChecked(false);
                    virtualJoystickView.DisableSnapping();
                }
                return true;
            default:
                return super.onOptionsItemSelected(item);
        }
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        virtualJoystickView = (VirtualJoystickView) findViewById(R.id.virtual_joystick);
        visualizationView = (VisualizationView) findViewById(R.id.visualization);
        visualizationView.getCamera().jumpToFrame("map");
        visualizationView.onCreate(Lists.<Layer>newArrayList(new CameraControlLayer(),
                new OccupancyGridLayer("map"), new PathLayer("move_base/NavfnROS/plan"), new PathLayer(
                        "move_base_dynamic/NavfnROS/plan"), new LaserScanLayer("base_scan"),
                new PoseSubscriberLayer("simple_waypoints_server/goal_pose"), new PosePublisherLayer(
                        "simple_waypoints_server/goal_pose"), new RobotLayer("base_footprint")));
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        visualizationView.init(nodeMainExecutor);
        NodeConfiguration nodeConfiguration =
                NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(),
                        getMasterUri());
        nodeMainExecutor
                .execute(virtualJoystickView, nodeConfiguration.setNodeName("virtual_joystick"));
        nodeMainExecutor.execute(visualizationView, nodeConfiguration.setNodeName("android/map_view"));
    }

}