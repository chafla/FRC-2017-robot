package org.usfirst.frc.team852.robot.strategies;

import org.usfirst.frc.team852.robot.Robot;
import org.usfirst.frc.team852.robot.SensorType;
import org.usfirst.frc.team852.robot.data.LidarData;


/**
 * Created by Matt on 2/12/2017.
 * Strategy for driving robot autonomously
 */
public class MattStrategy implements Strategy {

    final private double DEFAULT_DELAY = 0.05;  // Default delay between movement actions

    @Override
    public void reset() {

    }

    public void xboxYButtonPressed(final Robot robot) {

        centerWithLidar(robot);

    }

    public void xboxXButtonPressed(final Robot robot) {


    }

    public void xboxAButtonPressed(final Robot robot) {

        approachGear(robot, 450);

    }

    public void xboxBButtonPressed(final Robot robot) {


    }

    public void centerWithLidar(final Robot robot) {
        final LidarData leftLidar = robot.getCurrentLeftLidar();
        final LidarData rightLidar = robot.getCurrentRightLidar();
        int leftMm;
        int rightMm;

        System.out.println("Centering");

        int TOLERANCE = 10;  // Possible allowed distance between the differences
        int MIN = 15;  // Needed with plexiglass causing strange issues
        int dif;
        int sign;
        if (leftLidar == null || rightLidar == null) {
            System.out.println("Lidar data is null.");
            // We've got no valid info, let's just go for whatever we can get
            robot.drive(0, 0, 0.35, DEFAULT_DELAY, SensorType.LIDAR_GEAR, "Searching for valid surface");
            return;

        } else {
            leftMm = leftLidar.getMm();
            rightMm = rightLidar.getMm();
        }
        // If value is out of range, rotate until we get valid data
        // Note: This might cause issues if we get random OOR vals

        // If we're at least getting one value, rotate appropriately
        if (leftMm < 0 && rightMm < 0) {
            System.out.println("Both lidar values are -1.");
            // TODO work out what to do here
        } else if ((leftMm < MIN && leftMm > 0) || rightMm < MIN && rightMm > 0) {
            System.out.println("Value was below minimum threshold.");
        } else if (leftMm > 0 && rightMm < 0)
            robot.drive(0, 0, -0.35, DEFAULT_DELAY, SensorType.LIDAR_GEAR, "Valid surface found for right lidar");

            // If we're at least getting one value, rotate appropriately
        else if (leftMm < 0 && rightMm > 0)
            robot.drive(0, 0, 0.35, DEFAULT_DELAY, SensorType.LIDAR_GEAR, "Valid surface found for left lidar");

        else {
            dif = rightMm - leftMm;

            // TODO If one side is out of range, slide or rotate

            if (Math.abs(dif) > TOLERANCE) {

                // Do distance stuff
                // Positive rotation is clockwise

                sign = dif > 0 ? 1 : -1;
                System.out.printf("Left lidar: %s, Right lidar: %s.\n", leftMm, rightMm);
                robot.drive(0, 0, sign * 0.20, DEFAULT_DELAY, SensorType.LIDAR_GEAR, "Aligning");
                robot.drive(0, 0, 0, 0, SensorType.LIDAR_GEAR, "Stopping");  // Stop movement

            } else {
                System.out.println("Lidar centered.");
                System.out.printf("Left lidar: %s, Right lidar: %s.\n", leftMm, rightMm);

            }
        }

    }

    // Get how far we want to rotate in order to continue driving straight
    // Drive forward
    // Check lidar values
    // If one becomes -1, re-correct self
    // Otherwise, return a double that represents the distance that we want to rotate while driving.
    // This is just useful for the lidar, all we need to do is keep straight
    public double getRotationAmount() {
        return 0.0;
    }

    public void approachGear(final Robot robot, int dist) {
        final LidarData leftLidar = robot.getCurrentLeftLidar();
        final LidarData rightLidar = robot.getCurrentRightLidar();

        boolean atDist = false;
        int TOLERANCE = 10;  // Possible allowed distance between the differences
        robot.drive(0, 0, 0, 0, SensorType.LIDAR_GEAR, "Stopping");  // Stop movement

        if (leftLidar == null || rightLidar == null) {
            this.centerWithLidar(robot);
        } else {

            int left = leftLidar.getMm();
            int right = rightLidar.getMm();
            int movementDir;

            if ((Math.abs(left - dist) < TOLERANCE) && (Math.abs(right - dist) < TOLERANCE)) {
                this.centerWithLidar(robot);
                System.out.printf("Left lidar: %s, Right lidar: %s.\n", left, right);
                System.out.printf("Lidar sitting at dist %s\n", dist);
            } else {

                this.centerWithLidar(robot);  // Make a small adjustment

                movementDir = (left - dist > 0 && right - dist > 0) ? 1 : -1;

                // Small bit of proportionality
                /*
                double yPwr = -movementDir * (Math.pow(0.3, (((Math.abs(left - dist) + (right - dist)) / 2) / dist)));

                robot.drive(0, yPwr, 0, DEFAULT_DELAY, SensorType.LIDAR_GEAR, String.format("Adjusting %s", movementDir > 0 ? "forwards" : "backwards"));
                */
                robot.drive(0, movementDir * -0.2 * (((left + right) / 2) / dist), 0, DEFAULT_DELAY, SensorType.LIDAR_GEAR, String.format("Adjusting %s", movementDir > 0 ? "forwards" : "backwards"));


            }

        }

    }

    public void approachGear() {


    }

    public void stopMovement(Robot robot) {
        robot.drive(0, 0, 0, 0, SensorType.LIDAR_GEAR, "Stopping");  // Stop movement

    }

}