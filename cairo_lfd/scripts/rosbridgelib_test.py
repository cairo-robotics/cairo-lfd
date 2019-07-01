#!/usr/bin/python
import roslibpy
import rospy

if __name__ == "__main__":
    client = roslibpy.Ros(host="localhost", port=9090)
    talker = roslibpy.Topic(client, '/rosbridgelib_test', 'std_msgs/String')

    print client.is_connected
    def start_talking():
        while client.is_connected:
            talker.publish(roslibpy.Message({'data': 'This is a test.'}))
            print("Publishing...")
            rospy.sleep(1)
        talker.unadvertise()

    client.on_ready(start_talking)
    client.run_forever()