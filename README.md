# nmea_parser
A node that receives nmea data (GGA, RMC, HDT), parses it, and then publishes it.
Data is received via serial and the baud rate is automatically adjusted,
so you only need to match the serial port name connected to USB to run it.
This code only supports three data formats: GGA, RMC, and HDT, and you can add more freely.

# How to use

This code was written and tested in a ros2 humble environment.

1.install
2. colcon build
3. ros2 run nmea_parser pub_node
4. ros2 run nmea_parser sub_node
