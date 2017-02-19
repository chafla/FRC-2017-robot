package org.usfirst.frc.team852.robot.strategy;

import org.usfirst.frc.team852.robot.Robot;
import org.usfirst.frc.team852.robot.data.CameraData;
import org.usfirst.frc.team852.robot.data.HeadingData;
import org.usfirst.frc.team852.robot.data.LidarData;
import org.usfirst.frc.team852.robot.navigation.HeadingFeedback;

import java.util.concurrent.atomic.AtomicReference;

import static org.usfirst.frc.team852.robot.SensorType.*;

public class JvStrategy implements Strategy {

    private final AtomicReference<HeadingFeedback> headingFeedbackRef = new AtomicReference<>();

    public JvStrategy() {
        this.reset();
    }

    private HeadingFeedback getHeadingFeedback() {
        return headingFeedbackRef.get();
    }

    private static final double THRESHHOLD_DEGREES = 1.5;
    private static final double PID_CORRECTION_STRAIGHT = 0.01;
    private static final double PID_CORRECTION_TURN = 0.1;

    @Override
    public void reset() {
        this.headingFeedbackRef.set(null);
    }

    @Override
    public void resetHeading() {
        this.headingFeedbackRef.set(null);
    }

    // consider renaming these methods
    @Override
    public void onXboxA(final Robot robot) {
        final CameraData cameraData = robot.getCurrentCameraGear();

        if (cameraData == null) {
            robot.logMsg(CAMERA_GEAR, "Null camera data");
            return;
        }

        if (cameraData.isInvalid()) {
            System.out.println(cameraData.getAlreadyReadMsg());
            robot.waitOnCameraGear(0);
            return;
        }

        final int xVal = cameraData.getValOnce();
        final int wVal = cameraData.getWidth();

        if (xVal == -1)
            robot.logMsg(CAMERA_GEAR, "No camera data");
        else if (xVal < (wVal / 2 - wVal * 0.1))
            robot.drive(.3, 0, 0, 0.1, CAMERA_GEAR, "turn left");
        else if (xVal > (wVal / 2 + wVal * 0.1))
            robot.drive(-0.3, 0, 0, 0.1, CAMERA_GEAR, "turn right");
        else
            robot.logMsg(CAMERA_GEAR, "centered");
    }

    @Override
    public void onXboxB(final Robot robot) {
        // v1 of lidar driving
        final LidarData leftLidarData = robot.getCurrentLeftLidar();
        final LidarData rightLidarData = robot.getCurrentRightLidar();

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
            robot.waitOnLeftLidar(0);
            return;
        }

        if (rightLidarData.isInvalid()) {
            System.out.println(rightLidarData.getAlreadyReadMsg());
            robot.waitOnRightLidar(0);
            return;
        }

        final int lVal = leftLidarData.getValOnce();
        final int rVal = rightLidarData.getValOnce();

        if (lVal == -1 || rVal == -1)
            robot.logMsg(LIDAR_GEAR, "Out of range");
        else if (lVal > 410 && rVal > 410)
            robot.drive(0, -0.3, 0, 0.1, LIDAR_GEAR, "forwards");
        else if (lVal < 390 && rVal < 390)
            robot.drive(0, 0.3, 0, 0.1, LIDAR_GEAR, "backwards");
        else if (lVal > rVal + 10)
            robot.drive(0, 0, -0.25, 0.1, LIDAR_GEAR, "rotate clockwise");
        else if (lVal < rVal - 10)
            robot.drive(0, 0, 0.25, 0.1, LIDAR_GEAR, "rotate counter-clockwise");
        else
            robot.logMsg(LIDAR_GEAR, "centered by drive");
    }

    @Override
    public void onXboxX(final Robot robot) {
        final HeadingData headingData = robot.getCurrentHeading();

        if (headingData == null) {
            robot.logMsg(HEADING, "Null heading data");
            return;
        }


        if (headingData.isInvalid()) {
            robot.logMsg(HEADING, headingData.getAlreadyReadMsg());
            robot.drive(0, -0.2, 0, 0.1, HEADING, "old data -> go straight");
            return;
        }

        final double degrees = headingData.getDegreesOnce();

        // This will be set the first time through
        if (this.getHeadingFeedback() == null)
            this.headingFeedbackRef.set(new HeadingFeedback(degrees));

        final double errorDegrees = this.getHeadingFeedback().getError(degrees);

        final double turnSpeed;
        final String command;
        if (errorDegrees > THRESHHOLD_DEGREES) {
            // veered right, turn left, turnSpeed will be no less than -1
            turnSpeed = Math.max(-errorDegrees * PID_CORRECTION_STRAIGHT, -0.25);
            command = "Forward and counter-clockwise";

        } else if (errorDegrees < (THRESHHOLD_DEGREES * -1)) {
            // veered left, turn right, turnSpeed will be no more 1
            turnSpeed = Math.min(-errorDegrees * PID_CORRECTION_STRAIGHT, 0.25);
            command = "Forward and clockwise";
        } else {
            // On course, drive straight.
            turnSpeed = 0;
            command = "Forward";
        }
        robot.logMsg(HEADING, "error: " + errorDegrees + " turn speed: " + turnSpeed);
        robot.drive(0, -0.3, turnSpeed, 0.1, HEADING, command);
    }

    @Override
    public void onXboxY(final Robot robot) {
        // v2 of lidar driving
        final LidarData leftLidarData = robot.getCurrentLeftLidar();
        final LidarData rightLidarData = robot.getCurrentRightLidar();

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
            robot.waitOnLeftLidar(0);
            return;
        }

        if (rightLidarData.isInvalid()) {
            System.out.println(rightLidarData.getAlreadyReadMsg());
            robot.waitOnRightLidar(0);
            return;
        }

        final int lVal = leftLidarData.getValOnce();
        final int rVal = rightLidarData.getValOnce();

        if (leftLidarData.getValOnce() == -1 || rightLidarData.getValOnce() == -1) {
            robot.logMsg(LIDAR_GEAR, "Out of range");
        } else if (lVal > rVal + 10) {
            if (lVal > 320 && rVal > 320)
                robot.drive(0, -0.3, -0.25, 0.1, LIDAR_GEAR, "clockwise and forward");
            else if (lVal < 280 && rVal < 280)
                robot.drive(0, 0.3, -0.25, 0.1, LIDAR_GEAR, "clockwise and backward");
            else
                robot.drive(0, 0, -0.25, 0.1, LIDAR_GEAR, "clockwise");
        } else if (lVal < rVal - 10) {
            if (lVal > 320 && rVal > 320)
                robot.drive(0, -0.3, 0.25, 0.1, LIDAR_GEAR, "counter-clockwise and forward");
            else if (lVal < 280 && rVal < 280)
                robot.drive(0, 0.3, 0.25, 0.1, LIDAR_GEAR, "counter-clockwise and backward");
            else
                robot.drive(0, 0, 0.25, 0.1, LIDAR_GEAR, "counter-clockwise");
        } else {
            if (lVal > 320 && rVal > 320)
                robot.drive(0, -0.3, 0, 0.1, LIDAR_GEAR, "forward");
            else if (lVal < 290 && rVal < 290)
                robot.drive(0, 0.3, 0, 0.1, LIDAR_GEAR, "backward");
        }
    }

    @Override
    public void onXboxLB(final Robot robot) {
        robot.moveRandPLeft();
    }

    @Override
    public void onXboxRB(final Robot robot) {
        robot.moveRandPRight();
        /*final CameraData cameraData = robot.getCurrentCameraGear();

        if (cameraData == null) {
            robot.logMsg(CAMERA_GEAR, "Null camera data");
            return;
        }
        if (cameraData.getTimestamp() <= robot.getCameraLastTime()) {
            robot.logMsg(CAMERA_GEAR, "Stale camera data");
            return;
        } else {
            robot.updateCameraLastTime();
        }

        final int xVal = cameraData.getX();
        final int wVal = cameraData.getWidth();
        final int error = 3;

        if (xVal == -1)
            robot.logMsg(CAMERA_GEAR, "No camera data");
        else if (xVal < (wVal / 2) - error)
            robot.moveRandPRight();
        else if (xVal > (wVal / 2) + error)
            robot.moveRandPRight();
        else
            robot.logMsg(CAMERA_GEAR, "centered by RandP");*/
    }

    @Override
    public void onXboxBack(Robot robot) {
        final HeadingData headingData = robot.getCurrentHeading();

        if (headingData == null) {
            robot.logMsg(HEADING, "Null heading data");
            return;
        }

        if (headingData.isInvalid()) {
            System.out.println(headingData.getAlreadyReadMsg());
            robot.waitOnHeading(0);
            return;
        }

        final double degrees = headingData.getDegreesOnce();
        final double turn = 45;

        // This will be set the first time through
        if (this.getHeadingFeedback() == null)
            this.headingFeedbackRef.set(new HeadingFeedback(degrees + turn));

        final double errorDegrees = this.getHeadingFeedback().getError(degrees);

        final double turnSpeed;
        final String command;
        if (errorDegrees > THRESHHOLD_DEGREES) {
            // veered right, turn left, turnSpeed will be no less than -1
            turnSpeed = Math.max(-errorDegrees * PID_CORRECTION_TURN, -0.25);
            command = "Forward and counter-clockwise";

        } else if (errorDegrees < (THRESHHOLD_DEGREES * -1)) {
            // veered left, turn right, turnSpeed will be no more 1
            turnSpeed = Math.min(-errorDegrees * PID_CORRECTION_TURN, 0.25);
            command = "Forward and clockwise";
        } else {
            // On course, drive straight.
            turnSpeed = 0;
            command = "Centered";
        }

        robot.drive(0, 0, turnSpeed, 0.1, HEADING, command);
    }

    @Override
    public void onXboxStart(Robot robot) {
        robot.climb();
    }

    @Override
    public void onXboxLS(Robot robot) {
        robot.retractPiston();
    }

    @Override
    public void onXboxRS(Robot robot) {
        robot.pushGear();
    }
}
