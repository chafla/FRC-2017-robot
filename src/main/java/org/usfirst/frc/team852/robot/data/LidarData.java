package org.usfirst.frc.team852.robot.data;

public class LidarData extends GenericData {
    private final int val;

    public LidarData(final DataType dataType, final int val) {
        super(dataType);
        this.val = val;
    }

    public int getValOnce() {
        this.setInvalid();
        return this.val;
    }
}