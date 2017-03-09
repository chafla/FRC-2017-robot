package org.usfirst.frc.team852.robot;

public interface Constants {
    String CAMERA_GEAR_LOGGING_POSITION_TOPIC = "logging/camera/gear/alignment";
    String LIDAR_GEAR_LOGGING_POSITION_TOPIC = "logging/lidar/gear/distance";
    String LONG_LIDAR_LOGGING_POSITION_TOPIC = "logging/lidar/long/distance";
    String HEADING_LOGGING_POSITION_TOPIC = "logging/heading/degrees";
    String MQTT_TOPIC = "roborio/keyboard/command";
    String CAMERA_TOPIC = "camera/gear/x";
    String CAMERA_COMMAND = "camera/gear/x=command";
    String FRONT_LIDAR_TOPIC = "lidar/front/cm";
    String FRONT_LIDAR_COMMAND = "lidar/front/command";
    String REAR_LIDAR_TOPIC = "lidar/rear/cm";
    String REAR_LIDAR_COMMAND = "lidar/rear/command";
    String LEFT_LIDAR_TOPIC = "lidar/left/mm";
    String LEFT_LIDAR_COMMAND = "lidar/left/command";
    String RIGHT_LIDAR_TOPIC = "lidar/right/mm";
    String RIGHT_LIDAR_COMMAND = "lidar/right/command";
    String MQTT_HOSTNAME = "mqtt-turtle.local"; /*"10.8.52.14";*/
    String HEADING_TOPIC = "heading/degrees";
    String HEADING_COMMAND = "heading/command";
    int MQTT_PORT = 1883;
}
