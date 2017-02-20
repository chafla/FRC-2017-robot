package org.usfirst.frc.team852.robot.strategy;

import org.usfirst.frc.team852.robot.Robot;
import org.usfirst.frc.team852.robot.data.CameraData;
import org.usfirst.frc.team852.robot.data.HeadingData;
import org.usfirst.frc.team852.robot.data.LidarData;
import org.usfirst.frc.team852.robot.navigation.HeadingError;

import static org.usfirst.frc.team852.robot.SensorType.*;

public class JvStrategy extends Strategy {

    private static final double THRESHHOLD_DEGREES = 1.5;
    private static final double PID_CORRECTION_STRAIGHT = 0.01;
    private static final double PID_CORRECTION_TURN = 0.1;

    private double x = 0;
    private double y = 0;
    private double rot = 0;
    private static final double frontTarget = 10;
    private static final double rearTarget = 10;
    private static final double turn = 45;
    private static final int upperLidarThreshold = 525;
    private static final int lowerLidarThreshold = 475;


    public JvStrategy(final Robot robot) {
        super(robot);
        this.reset();
    }

    @Override
    public void reset() {
        this.resetHeadingError();
    }

    // consider renaming these methods
    @Override
    public void onXboxA() {
        final Robot robot = this.getRobot();
        final CameraData cameraData = this.getCurrentCameraGear();
        final LidarData leftLidarData = this.getCurrentLeftLidar();
        final LidarData rightLidarData = this.getCurrentRightLidar();

        if (cameraData == null) {
            robot.rumble(1);
            robot.logMsg(CAMERA_GEAR, "Null camera data");
            return;
        }

        if (leftLidarData == null) {
            robot.rumble(1);
            robot.logMsg(LIDAR_GEAR, "Null left lidar data");
            return;
        }

        if (rightLidarData == null) {
            robot.rumble(1);
            robot.logMsg(LIDAR_GEAR, "Null right lidar data");
            return;
        }
        robot.rumble(0);
        final int lVal = leftLidarData.getValOnce();
        final int rVal = rightLidarData.getValOnce();
        final int xVal = cameraData.getValOnce();
        final int wVal = cameraData.getWidth();

        if (xVal == -1) {
            robot.logMsg(CAMERA_GEAR, "No camera data");
            robot.rumble(1);
        } else if (xVal < (wVal / 2 - wVal * 0.075)) {
            x = 0.15;
            robot.stopRandP();
        } else if (xVal > (wVal / 2 + wVal * 0.075)) {
            x = -0.15;
            robot.stopRandP();
        }
        else {
            if (x != 0)
                x = 0;
        }

        if (leftLidarData.getValOnce() == -1 || rightLidarData.getValOnce() == -1) {
            robot.logMsg(LIDAR_GEAR, "Out of range");
            robot.rumble(1);
        } else if (lVal > rVal + 10) {
            if (rot != 0.1)
                rot = 0.1;
            if (lVal > upperLidarThreshold && rVal > upperLidarThreshold && y != -0.15)
                y = -0.15;
            else if (lVal < lowerLidarThreshold && rVal < lowerLidarThreshold && y != 0.15)
                y = 0.15;
            else {
                if (y != 0)
                    y = 0;
            }
        } else if (lVal < rVal - 10) {
            if (rot != -0.1)
                rot = -0.1;
            if (lVal > upperLidarThreshold && rVal > upperLidarThreshold && y != -0.15)
                y = -0.15;
            else if (lVal < lowerLidarThreshold && rVal < lowerLidarThreshold && y != 0.15)
                y = 0.15;
            else {
                if (y != 0)
                    y = 0;
            }
        } else {
            if (rot != 0)
                rot = 0;
            if (lVal > upperLidarThreshold && rVal > upperLidarThreshold && y != -0.15)
                y = -0.15;
            else if (lVal < lowerLidarThreshold && rVal < lowerLidarThreshold && y != 0.15)
                y = 0.15;
            else {
                if (y != 0)
                    y = 0;
            }
        }

        robot.drive(x, y, rot, CAMERA_GEAR, x + " " + y + " " + rot);
        robot.rumble(0);

        if (x == 0 && y == 0 && rot == 0) {
            if (xVal < (wVal / 2 - 1))
                robot.moveRandPRight();
            else if (xVal > (wVal / 2 + 1))
                robot.moveRandPLeft();
            else
                robot.stopRandP();
        }
    }

    @Override
    public void onXboxB() {
        final Robot robot = this.getRobot();
        final CameraData cameraData = this.getCurrentCameraGear();

        if (cameraData == null) {
            robot.logMsg(CAMERA_GEAR, "Null camera data");
            return;
        }

        if (cameraData.isInvalid()) {
            System.out.println(cameraData.getAlreadyReadMsg());
            this.waitOnCameraGear(0);
            return;
        }

        final int xVal = cameraData.getValOnce();
        final int wVal = cameraData.getWidth();

        if (xVal == -1)
            robot.logMsg(CAMERA_GEAR, "No camera data");
        else if (xVal < (wVal / 2 - wVal * 0.1))
            robot.drive(.3, 0, 0, CAMERA_GEAR, "move left");
        else if (xVal > (wVal / 2 + wVal * 0.1))
            robot.drive(-0.3, 0, 0, CAMERA_GEAR, "move right");
        else
            robot.drive(0, 0, 0, CAMERA_GEAR, "chassis centered");
    }

    @Override
    public void onXboxX() {
        final Robot robot = this.getRobot();
        final HeadingData headingData = this.getCurrentHeading();

        if (headingData == null) {
            robot.logMsg(HEADING, "Null heading data");
            return;
        }


        if (headingData.isInvalid()) {
            //robot.logMsg(HEADING, headingData.getAlreadyReadMsg());
            //robot.drive(0, -0.2, 0, HEADING, "old data -> go straight");
            //return;
        }

        final double degrees = headingData.getDegreesOnce();

        // This will be set the first time through
        if (this.getHeadingError() == null)
            this.setHeadingError(new HeadingError(degrees));

        final double errorDegrees = this.getHeadingError().getError(degrees);

        final double turnSpeed;
        final String command;
        if (errorDegrees > THRESHHOLD_DEGREES) {
            // veered right, turn left, turnSpeed will be no less than -1
            turnSpeed = Math.max(-errorDegrees * PID_CORRECTION_STRAIGHT, -0.2);
            command = "Forward and counter-clockwise";

        } else if (errorDegrees < (THRESHHOLD_DEGREES * -1)) {
            // veered left, turn right, turnSpeed will be no more 1
            turnSpeed = Math.min(-errorDegrees * PID_CORRECTION_STRAIGHT, 0.2);
            command = "Forward and clockwise";
        } else {
            // On course, drive straight.
            turnSpeed = 0;
            command = "Forward";
        }
        robot.logMsg(HEADING, "error: " + errorDegrees + " turn speed: " + turnSpeed);
        robot.drive(0, -0.3, turnSpeed, HEADING, command);
    }

    @Override
    public void onXboxY() {
        final Robot robot = this.getRobot();
        final LidarData leftLidarData = this.getCurrentLeftLidar();
        final LidarData rightLidarData = this.getCurrentRightLidar();

        if (leftLidarData == null) {
            robot.logMsg(LIDAR_GEAR, "Null left lidar data");
            return;
        }

        if (rightLidarData == null) {
            robot.logMsg(LIDAR_GEAR, "Null right lidar data");
            return;
        }

        if (leftLidarData.isInvalid()) {
            System.out.println(leftLidarData.getAlreadyReadMsg());
            this.waitOnLeftLidar(0);
            return;
        }

        if (rightLidarData.isInvalid()) {
            System.out.println(rightLidarData.getAlreadyReadMsg());
            this.waitOnRightLidar(0);
            return;
        }

        final int lVal = leftLidarData.getValOnce();
        final int rVal = rightLidarData.getValOnce();

        // lidar driving, turn while moving forwards

        if (leftLidarData.getValOnce() == -1 || rightLidarData.getValOnce() == -1) {
            robot.logMsg(LIDAR_GEAR, "Out of range");
        } else if (lVal > rVal + 10) {
            if (lVal > 320 && rVal > 320)
                robot.drive(0, -0.3, -0.2, LIDAR_GEAR, "clockwise and forward");
            else if (lVal < 280 && rVal < 280)
                robot.drive(0, 0.3, -0.2, LIDAR_GEAR, "clockwise and backward");
            else
                robot.drive(0, 0, -0.2, LIDAR_GEAR, "clockwise");
        } else if (lVal < rVal - 10) {
            if (lVal > 320 && rVal > 320)
                robot.drive(0, -0.3, 0.2, LIDAR_GEAR, "counter-clockwise and forward");
            else if (lVal < 280 && rVal < 280)
                robot.drive(0, 0.3, 0.2, LIDAR_GEAR, "counter-clockwise and backward");
            else
                robot.drive(0, 0, 0.2, LIDAR_GEAR, "counter-clockwise");
        } else {
            if (lVal > 320 && rVal > 320)
                robot.drive(0, -0.3, 0, LIDAR_GEAR, "forward");
            else if (lVal < 290 && rVal < 290)
                robot.drive(0, 0.3, 0, LIDAR_GEAR, "backward");
            else
                robot.drive(0, 0, 0, LIDAR_GEAR, "centered");
        }

        // lidar driving, straight and turning in place
        /*
        if (lVal == -1 || rVal == -1)
            robot.logMsg(LIDAR_GEAR, "Out of range");
        else if (lVal > 410 && rVal > 410)
            robot.drive(0, -0.3, 0, LIDAR_GEAR, "forwards");
        else if (lVal < 390 && rVal < 390)
            robot.drive(0, 0.3, 0, LIDAR_GEAR, "backwards");
        else if (lVal > rVal + 10)
            robot.drive(0, 0, -0.25, LIDAR_GEAR, "rotate clockwise");
        else if (lVal < rVal - 10)
            robot.drive(0, 0, 0.25, LIDAR_GEAR, "rotate counter-clockwise");
        else
            robot.drive(0,0,0,LIDAR_GEAR,"centered");
        */
    }

    @Override
    public void onXboxLB() {
        this.getRobot().moveRandPLeft();
    }

    @Override
    public void onXboxRB() {
        this.getRobot().moveRandPRight();
    }

    @Override
    public void onXboxBack() {
        final Robot robot = this.getRobot();
        final HeadingData headingData = this.getCurrentHeading();

        if (headingData == null) {
            robot.logMsg(HEADING, "Null heading data");
            return;
        }

        if (headingData.isInvalid()) {
            //System.out.println(headingData.getAlreadyReadMsg());
            //this.waitOnHeading(0);
            //return;
        }

        final double degrees = headingData.getDegreesOnce();

        // This will be set the first time through
        if (this.getHeadingError() == null)
            this.setHeadingError(new HeadingError(degrees + turn));

        final double errorDegrees = this.getHeadingError().getError(degrees);

        final double turnSpeed;
        final String command;
        if (errorDegrees > THRESHHOLD_DEGREES) {
            // veered right, turn left, turnSpeed will be no less than -1
            turnSpeed = Math.max(-errorDegrees * PID_CORRECTION_TURN, -0.2);
            command = "Counter-clockwise";

        } else if (errorDegrees < (THRESHHOLD_DEGREES * -1)) {
            // veered left, turn right, turnSpeed will be no more 1
            turnSpeed = Math.min(-errorDegrees * PID_CORRECTION_TURN, 0.2);
            command = "Clockwise";
        } else {
            // On course, drive straight.
            turnSpeed = 0;
            command = "Aligned";
        }

        robot.drive(0, 0, turnSpeed, HEADING, command);
    }

    @Override
    public void onXboxStart() {
        // autonomous
        final Robot robot = this.getRobot();
        final HeadingData headingData = this.getCurrentHeading();
        final LidarData frontLidarData = this.getCurrentFrontLidar();
        final LidarData rearLidarData = this.getCurrentRearLidar();
        final LidarData leftLidarData = this.getCurrentLeftLidar();
        final LidarData rightLidarData = this.getCurrentRightLidar();

        if (leftLidarData == null) {
            robot.logMsg(LIDAR_GEAR, "Null left lidar data");
            return;
        }
        if (rightLidarData == null) {
            robot.logMsg(LIDAR_GEAR, "Null right lidar data");
            return;
        }
        if (headingData == null) {
            robot.logMsg(HEADING, "Null heading data");
            return;
        }
        if (frontLidarData == null) {
            robot.logMsg(HEADING, "Null heading data");
            return;
        }
        if (rearLidarData == null) {
            robot.logMsg(HEADING, "Null heading data");
            return;
        }

        final double fVal = frontLidarData.getValOnce();
        final double bVal = rearLidarData.getValOnce();
        final int lVal = leftLidarData.getValOnce();
        final int rVal = rightLidarData.getValOnce();

        final double degrees = headingData.getDegreesOnce();
        // This will be set the first time through
        if (this.getHeadingError() == null)
            this.setHeadingError(new HeadingError(degrees));

        final double errorDegrees = this.getHeadingError().getError(degrees);

        final double turnSpeed;
        final String command;
        if (fVal > frontTarget && bVal < rearTarget) {
            if (errorDegrees > THRESHHOLD_DEGREES) {
                // veered right, turn left, turnSpeed will be no less than -1
                turnSpeed = Math.max(-errorDegrees * PID_CORRECTION_STRAIGHT, -0.2);
                command = "Forward and counter-clockwise";

            } else if (errorDegrees < (THRESHHOLD_DEGREES * -1)) {
                // veered left, turn right, turnSpeed will be no more 1
                turnSpeed = Math.min(-errorDegrees * PID_CORRECTION_STRAIGHT, 0.2);
                command = "Forward and clockwise";
            } else {
                // On course, drive straight.
                turnSpeed = 0;
                command = "Forward";
            }
            robot.logMsg(HEADING, "error: " + errorDegrees + " turn speed: " + turnSpeed);
            robot.drive(0, -0.3, turnSpeed, HEADING, command);
            return;
        }

        if (false)//first time through
            this.setHeadingError(null);

        if (this.getHeadingError() == null)
            this.setHeadingError(new HeadingError(degrees + turn));

        if (errorDegrees > THRESHHOLD_DEGREES || errorDegrees < -THRESHHOLD_DEGREES) {
            if (errorDegrees > THRESHHOLD_DEGREES) {
                // veered right, turn left, turnSpeed will be no less than -1
                turnSpeed = Math.max(-errorDegrees * PID_CORRECTION_STRAIGHT, -0.2);
                command = "Counter-clockwise";

            } else if (errorDegrees < (THRESHHOLD_DEGREES * -1)) {
                // veered left, turn right, turnSpeed will be no more 1
                turnSpeed = Math.min(-errorDegrees * PID_CORRECTION_STRAIGHT, 0.2);
                command = "Clockwise";
            } else {
                // On course, drive straight.
                turnSpeed = 0;
                command = "Aligned";
            }

            robot.drive(0, 0, turnSpeed, HEADING, command);
            return;
        }

        if (false)// first time through
            resetHeadingError();

        if (rVal == -1 && lVal == -1) {
            if (errorDegrees > THRESHHOLD_DEGREES) {
                // veered right, turn left, turnSpeed will be no less than -1
                turnSpeed = Math.max(-errorDegrees * PID_CORRECTION_STRAIGHT, -0.2);
                command = "Forward and counter-clockwise";

            } else if (errorDegrees < (THRESHHOLD_DEGREES * -1)) {
                // veered left, turn right, turnSpeed will be no more 1
                turnSpeed = Math.min(-errorDegrees * PID_CORRECTION_STRAIGHT, 0.2);
                command = "Forward and clockwise";
            } else {
                // On course, drive straight.
                turnSpeed = 0;
                command = "Forward";
            }
            robot.logMsg(HEADING, "error: " + errorDegrees + " turn speed: " + turnSpeed);
            robot.drive(0, -0.3, turnSpeed, HEADING, command);
            return;
        }

        this.onXboxA();



    }

    @Override
    public void onXboxLS() {
        this.getRobot().retractPiston();
    }

    @Override
    public void onXboxRS() {
        this.getRobot().pushGear();
    }
}
