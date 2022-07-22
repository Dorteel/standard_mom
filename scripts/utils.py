class RosNode():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self,
        name="test_node",
        sub_topic="listener",
        pub_topic="talker",
        sub_msg_type=Image,
        pub_msg_type=Image
        ):

        # Initialize node
        self.node_name = name
        rospy.init_node(self.node_name)

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(100)

        # Publishers
        self.publish_topic = pub_topic
        self.msg_to_publish = 0
        self.pub = rospy.Publisher(self.publish_topic, pub_msg_type, queue_size=100)

        # Subscribers
        self.subscribe_topic = sub_topic
        rospy.Subscriber(self.subscribe_topic, sub_msg_type, self.callback)
        self.msg = None



    def callback(self, msg):
        self.msg = msg
        rospy.loginfo("Received: {}".format(self.msg.data))


    def start(self):
        while not rospy.is_shutdown():
            # Publish our custom message.
            if self.msg:
                #print(self.msg)
                self.msg_to_publish = self.msg
                self.pub.publish(self.msg_to_publish)
            # rospy.loginfo("\tPublished: {}".format(self.msg_to_publish))
            # Sleep for a while before publishing new messages. Division is so rate != period.
            # if self.loop_rate:
            #     rospy.sleep(0.001)
            # else:
            #     rospy.sleep(1.0)
            self.loop_rate.sleep()