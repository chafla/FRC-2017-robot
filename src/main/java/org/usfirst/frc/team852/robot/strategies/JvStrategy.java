package org.usfirst.frc.team852.robot.strategies;

import org.usfirst.frc.team852.robot.Robot;
import org.usfirst.frc.team852.robot.data.CameraGearData;
import org.usfirst.frc.team852.robot.data.HeadingData;
import org.usfirst.frc.team852.robot.data.LidarData;

import static org.usfirst.frc.team852.robot.SensorType.*;

public class JvStrategy implements Strategy {

    double initialDegree = -1;

    @Override
    public void xboxAButtonPressed(final Robot robot) {
        final CameraGearData cameraGear = robot.getCurrentCameraGear();

        if (cameraGear == null || cameraGear.getTimestamp() <= robot.getCameraLastTime())
            return;

        robot.updateCameraLastTime();

        final int xVal = cameraGear.getX();
        final int wVal = cameraGear.getWidth();

        if (xVal == -1)
            System.out.println("No camera data");
        else if (xVal < (wVal / 2 - wVal * 0.1))
            robot.drive(.3, 0, 0, 0.1, CAMERA_GEAR, "turn left");
        else if (xVal > (wVal / 2 + wVal * 0.1))
            robot.drive(-0.3, 0, 0, 0.1, CAMERA_GEAR, "turn right");
        else
            robot.logMsg(CAMERA_GEAR, "centered");
    }

    @Override
    public void xboxBButtonPressed(final Robot robot) {
        // v1 of lidar driving
        final LidarData leftLidar = robot.getCurrentLeftLidar();
        final LidarData rightLidar = robot.getCurrentRightLidar();

        if (leftLidar == null || rightLidar == null) {
            System.out.println("Null object");
            return;
        }

        if (leftLidar.getTimestamp() <= robot.getLidarLeftLastTime()
                || rightLidar.getTimestamp() <= robot.getLidarRightLastTime()) {
            System.out.println("Time hasn't updated");
            return;
        }

        final int lVal = leftLidar.getMm();
        final int rVal = rightLidar.getMm();

        robot.updateLidarLeftLastTime();
        robot.updateLidarRightLastTime();

        if (lVal == -1 || rVal == -1)
            System.out.println("Out of range");
        else if (lVal > 410 && rVal > 410)
            robot.drive(0, -0.3, 0, 0.1, LIDAR_GEAR, "forwards");
        else if (lVal < 390 && rVal < 390)
            robot.drive(0, 0.3, 0, 0.1, LIDAR_GEAR, "backwards");
        else if (lVal > rVal + 10)
            robot.drive(0, 0, -0.25, 0.1, LIDAR_GEAR, "rotate clockwise");
        else if (lVal < rVal - 10)
            robot.drive(0, 0, 0.25, 0.1, LIDAR_GEAR, "rotate counter-clockwise");
        else
            robot.logMsg(LIDAR_GEAR, "centered");
    }

    @Override
    public void xboxXButtonPressed(final Robot robot) {
        final HeadingData heading = robot.getCurrentHeading();

        if (heading == null) {
            System.out.println("Null Object");
            return;
        }

        if (heading.getTimestamp() <= robot.getHeadingLastTime()) {
            System.out.println("Time hasn't updated");
            return;
        }

        if (initialDegree == -1)
            initialDegree = heading.getDegree();


        final double currentDegree = heading.getDegree();
        final double error = 0.1;
        final double pidconstant = 0.1;
        final double lowerBound = initialDegree - error;
        final double upperBound = initialDegree + error;
        double turnSpeed;
        String command;
        double difference = initialDegree - currentDegree;
        if (Math.abs(difference) > 270) {
            // deal with overlap
            if (initialDegree > 180) {
                // have veered right, turn cc (e.g. initial = 359, current = 1)
                turnSpeed = difference * pidconstant;
                if (turnSpeed < -1)
                    turnSpeed = -1;
                command = "Forwards and counter-clockwise";
            } else {
                // have veered left, turn c (e.g.) initial = 1, current = 359)
                turnSpeed = difference * pidconstant;
                if (turnSpeed > 1)
                    turnSpeed = 1;
                command = "Forwards and clockwise";
            }

        } else {
            if (currentDegree > upperBound) {
                // have veered right, initial = 120, current = 125
                turnSpeed = difference * pidconstant;
                if (turnSpeed < -1)
                    turnSpeed = -1;
                command = "Forwards and counter-clockwise";
            } else if (currentDegree < lowerBound) {
                // have veered left, initial = 270, current = 260
                turnSpeed = difference * pidconstant;
                if (turnSpeed > 1)
                    turnSpeed = 1;
                command = "Forwards and clockwise";
            } else {
                turnSpeed = 0;
                command = "Forwards";
            }
        }
        robot.drive(0, 0.3, turnSpeed, 0, HEADING, command);


    }

    @Override
    public void xboxYButtonPressed(final Robot robot) {
        // v2 of lidar driving
        final LidarData leftMsg = robot.getCurrentLeftLidar();
        final LidarData rightMsg = robot.getCurrentRightLidar();

        if (leftMsg == null || rightMsg == null
                || leftMsg.getTimestamp() <= robot.getLidarLeftLastTime()
                || rightMsg.getTimestamp() <= robot.getLidarRightLastTime())
            return;

        final int lVal = leftMsg.getMm();
        final int rVal = rightMsg.getMm();

        robot.updateLidarLeftLastTime();
        robot.updateLidarRightLastTime();

        if (leftMsg.getMm() == -1 || rightMsg.getMm() == -1) {
            System.out.println("Out of range");
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
}
