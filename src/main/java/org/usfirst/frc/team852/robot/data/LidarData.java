package org.usfirst.frc.team852.robot.data;

public class LidarData extends GenericData {
    private final int val;

    public LidarData(int val) {
        this.val = val;
    }

    public int getVal() {
        return this.val;
    }
}