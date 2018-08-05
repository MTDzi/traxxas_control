# In case roslaunch fails

rosrun rosserial_python serial_node.py /dev/ttyACM0

rosrun topic_tools transform /joy /servo std_msgs/Int16 'int(180*(m.axes[0]+1)/2)'
rosrun topic_tools transform /joy /esc std_msgs/Int16 'int(102*m.axes[3])'

rosrun joy joy_node


# SD card
/media/ubuntu/3C52-B9FD


# ZED
/usr/local/zed/tools
