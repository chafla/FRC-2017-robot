package org.usfirst.frc.team852.robot.data;

public class CameraData extends GenericData {
    private final int val;
    private final int width;

    public CameraData(final DataType dataType, final int val, final int width) {
        super(dataType);
        this.val = val;
        this.width = width;
    }

    public int getValOnce() {
        this.setInvalid();
        return this.val;
    }

    public int getWidth() {
        return this.width;
    }
}
