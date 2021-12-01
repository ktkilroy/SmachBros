package com.katiekilroy.smachbros;

//import org.apache.commons.logging.Log;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;


public class Listener extends AbstractNodeMain {

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("smach_bros/listener");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
//        final Log log = connectedNode.getLog();

        Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber("chatter", std_msgs.String._TYPE);
        subscriber.addMessageListener(new MessageListener<std_msgs.String>() {
            @Override
            public void onNewMessage(std_msgs.String message) {
//                log.info("I heard: \"" + message.getData() + "\"");
            }
        });
    }
}
//
//    implementation 'commons-validator:commons-validator:1.6'
//            implementation 'org.apache.httpcomponents:httpclient:4.5.10'
