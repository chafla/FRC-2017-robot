package org.usfirst.frc.team852.robot.data;

public class CameraData extends GenericData {
    private final int x;
    private final int width;

    public CameraData(int x, int width) {
        this.x = x;
        this.width = width;
    }

    public int getX() {
        return x;
    }

    public int getWidth() {
        return width;
    }

}
