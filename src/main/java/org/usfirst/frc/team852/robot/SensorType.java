package org.usfirst.frc.team852.robot;

// Notice the 3 different ways of defining the MsgGenerator value
public enum SensorType {
    // Using anonymous class
    CAMERA_GEAR(Robot.CAMERA_GEAR_LOGGING_POSITION_TOPIC,
                new MsgGenerator() {
                    @Override
                    public String getMesssage(Robot robot, String desc) {
                        return String.format("%d - %s",
                                             robot.getCurrentCameraGear().getX(),
                                             desc);
                    }
                }),

    // Using a lambda
    LIDAR_GEAR(Robot.LIDAR_GEAR_LOGGING_POSITION_TOPIC,
               (robot, logMsg) -> {
                   return String.format("L %d R %d - %s",
                                        robot.getCurrentLeftLidar().getMm(),
                                        robot.getCurrentRightLidar().getMm(),
                                        logMsg);
               }),

    // Using an abbreviated lambda with implicit return
    LIDAR_CLIMBER(Robot.LIDAR_CLIMBER_LOGGING_POSITION_TOPIC,
                  (robot, logMsg) ->
                          String.format("L %d R %d - %s",
                                        robot.getCurrentLeftLidar().getMm(),
                                        robot.getCurrentRightLidar().getMm(),
                                        logMsg)
    );


    final String topic;
    final MsgGenerator msgGenerator;

    SensorType(final String topic, final MsgGenerator msgGenerator) {
        this.topic = topic;
        this.msgGenerator = msgGenerator;
    }

    public String getTopic() {
        return this.topic;
    }

    public MsgGenerator getMsgGenerator() {
        return this.msgGenerator;
    }
}
