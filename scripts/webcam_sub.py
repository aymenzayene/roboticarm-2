#!/usr/bin/env python3
# Description:
# - Subscribes to real-time streaming video from your built-in webcam.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np # NumPy library for numerical operations
import matplotlib.pyplot as plt # Import the matplotlib library for image visualization
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from std_msgs.msg import Float64MultiArray
# Import the custom service message
from cv_basics.srv import Centroid
def process_image(frame):
    # Convert the image to grayscale
    gimage = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply a binary threshold to the grayscale image
    blurred = cv2.GaussianBlur(gimage, (5, 5), 0)
    (retVal, newImg) = cv2.threshold(gimage, 220, 255, 0)

    # Calculate the moments of the selected contour (cnt)
    M = cv2.moments(newImg)

    # Calculate the centroid of the contour
    # The centroid coordinates are represented by 'cx' and 'cy'
    cx = int(M['m10'] / M['m00']) if M['m00'] != 0 else 0  # X-coordinate of the centroid
    cy = int(M['m01'] / M['m00']) if M['m00'] != 0 else 0  # Y-coordinate of the centroid

    # Print the centroid coordinates
    print("Centroid coordinates:", cx, cy)
    # Display the resulting binary image
    cv2.imshow("Binary Image", newImg)
    # Publish centroid data
    #centroid_data = Float64MultiArray()
    #centroid_data.data = [cx, cy]
    #centroid_pub.publish(centroid_data)
    # Create a service client and send the centroid data
    try:
        centroid_service = rospy.ServiceProxy('centroid_service', Centroid)
        resp = centroid_service(cx, cy)
        print("Response: ", resp.success)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    # Display the original image using matplotlib
    plt.imshow(frame)
    plt.imshow(gimage, cmap='gray')
    plt.scatter(cx, cy, c='red', marker='x', label='Centroid')
    plt.title('Original Image with Centroid Position')
    plt.xlabel('X Coordinate (pixels)')
    plt.ylabel('Y Coordinate (pixels)')
    plt.legend()
    plt.show()
    





def callback(data):
    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Output debugging information to the terminal
    rospy.loginfo("Receiving video frame")

    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data)

    # Display the original frame
    cv2.imshow("Original Frame", current_frame)

    # Process the image
    process_image(current_frame)

    cv2.waitKey(1)

def receive_message():
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    rospy.init_node('video_sub_py', anonymous=True)

    # Node is subscribing to the video_frames topic
    rospy.Subscriber('video_frames', Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    # Close down the video stream when done
    cv2.destroyAllWindows()

if __name__ == '__main__':
    receive_message()

