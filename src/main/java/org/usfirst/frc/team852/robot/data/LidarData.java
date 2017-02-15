package org.usfirst.frc.team852.robot.data;

public class LidarData extends GenericData {
    private final int mm;

    public LidarData(int mm) {
        this.mm = mm;
    }

    public int getMm() {
        return this.mm;
    }
}