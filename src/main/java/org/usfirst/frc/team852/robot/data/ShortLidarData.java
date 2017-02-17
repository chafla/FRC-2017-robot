package org.usfirst.frc.team852.robot.data;

public class ShortLidarData extends GenericData {
    private final int mm;

    public ShortLidarData(int mm) {
        this.mm = mm;
    }

    public int getMm() {
        return this.mm;
    }
}