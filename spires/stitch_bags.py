import rosbag
import argparse
import os

def stitch_bags(bag1_path, bag2_path, output_path):
    print(f"Opening {bag1_path} ...")
    bag1 = rosbag.Bag(bag1_path, 'r')

    print(f"Opening {bag2_path} ...")
    bag2 = rosbag.Bag(bag2_path, 'r')

    print(f"Writing stitched bag to {output_path} ...")
    with rosbag.Bag(output_path, 'w') as outbag:
        # Write all messages from bag1 unchanged
        for topic, msg, t in bag1.read_messages():
            outbag.write(topic, msg, t)

        # Find end time of bag1
        bag1_end_time = bag1.get_end_time()

        # Write all messages from bag2, shifted in time
        for topic, msg, t in bag2.read_messages():
            # delta = rospy.Duration.from_sec(bag1_end_time - bag2.get_start_time())
            # shifted_time = t + delta
            # print("time delta:", delta, "shifted time:", shifted_time)
            outbag.write(topic, msg, t)

    bag1.close()
    bag2.close()
    print("Done.")

if __name__ == "__main__":
    import rospy

    parser = argparse.ArgumentParser(description='Stitch two ROS bags end-to-end.')
    parser.add_argument('bag1', help='First input bag file')
    parser.add_argument('bag2', help='Second input bag file')
    parser.add_argument('output', help='Output stitched bag file')

    args = parser.parse_args()
    stitch_bags(args.bag1, args.bag2, args.output)