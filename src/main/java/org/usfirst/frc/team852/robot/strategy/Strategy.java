package org.usfirst.frc.team852.robot.strategy;

import org.usfirst.frc.team852.robot.Robot;
import org.usfirst.frc.team852.robot.data.CameraData;
import org.usfirst.frc.team852.robot.data.HeadingData;
import org.usfirst.frc.team852.robot.data.LidarData;
import org.usfirst.frc.team852.robot.navigation.HeadingError;

import java.util.concurrent.atomic.AtomicReference;

public abstract class Strategy {

    private final AtomicReference<CameraData> cameraGearRef = new AtomicReference<>();
    private final AtomicReference<LidarData> frontLidarRef = new AtomicReference<>();
    private final AtomicReference<LidarData> rearLidarRef = new AtomicReference<>();
    private final AtomicReference<LidarData> leftLidarRef = new AtomicReference<>();
    private final AtomicReference<LidarData> rightLidarRef = new AtomicReference<>();
    private final AtomicReference<HeadingData> headingRef = new AtomicReference<>();

    private final AtomicReference<HeadingError> headingErrorRef = new AtomicReference<>();

    private CameraData currentCameraGear = null;
    private LidarData currentFrontLidar = null;
    private LidarData currentRearLidar = null;
    private LidarData currentLeftLidar = null;
    private LidarData currentRightLidar = null;
    private HeadingData currentHeading = null;

    private final Robot robot;

    protected Strategy(final Robot robot) {
        this.robot = robot;
    }

    protected Robot getRobot() {
        return this.robot;
    }

    public void iterationInit() {
        this.currentCameraGear = this.getCameraGearRef().get();
        this.currentFrontLidar = this.getFrontLidarRef().get();
        this.currentRearLidar = this.getRearLidarRef().get();
        this.currentLeftLidar = this.getLeftLidarRef().get();
        this.currentRightLidar = this.getRightLidarRef().get();
        this.currentHeading = this.getHeadingRef().get();
    }

    protected HeadingError getHeadingError() {
        return headingErrorRef.get();
    }

    protected void setHeadingError(final HeadingError headingError) {
        this.headingErrorRef.set(headingError);
    }

    public void resetHeadingError() {
        this.headingErrorRef.set(null);
    }

    public CameraData getCurrentCameraGear() {
        return this.currentCameraGear;
    }

    public LidarData getCurrentLeftLidar() {
        return this.currentLeftLidar;
    }

    public LidarData getCurrentRightLidar() {
        return this.currentRightLidar;
    }

    public LidarData getCurrentRearLidar() {
        return this.currentRearLidar;
    }

    public LidarData getCurrentFrontLidar() {
        return this.currentFrontLidar;
    }

    public HeadingData getCurrentHeading() {
        return this.currentHeading;
    }

    public AtomicReference<CameraData> getCameraGearRef() {
        return this.cameraGearRef;
    }

    public AtomicReference<LidarData> getFrontLidarRef() {
        return this.frontLidarRef;
    }

    public AtomicReference<LidarData> getRearLidarRef() {
        return this.rearLidarRef;
    }

    public AtomicReference<LidarData> getLeftLidarRef() {
        return this.leftLidarRef;
    }

    public AtomicReference<LidarData> getRightLidarRef() {
        return this.rightLidarRef;
    }

    public AtomicReference<HeadingData> getHeadingRef() {
        return this.headingRef;
    }

    public void waitOnCameraGear(final long timeoutMillis) {
        synchronized (this.getCameraGearRef()) {
            try {
                this.getCameraGearRef().wait(timeoutMillis);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void waitOnFrontLidar(final long timeoutMillis) {
        synchronized (this.getFrontLidarRef()) {
            try {
                this.getFrontLidarRef().wait(timeoutMillis);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void waitOnRearLidar(final long timeoutMillis) {
        synchronized (this.getRearLidarRef()) {
            try {
                this.getRearLidarRef().wait(timeoutMillis);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void waitOnLeftLidar(final long timeoutMillis) {
        synchronized (this.getLeftLidarRef()) {
            try {
                this.getLeftLidarRef().wait(timeoutMillis);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void waitOnRightLidar(final long timeoutMillis) {
        synchronized (this.getRightLidarRef()) {
            try {
                this.getRightLidarRef().wait(timeoutMillis);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void waitOnHeading(final long timeoutMillis) {
        synchronized (this.getHeadingRef()) {
            try {
                this.getHeadingRef().wait(timeoutMillis);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void reset() {
    }

    public void onXboxA() {
    }

    public void onXboxB() {
    }

    public void onXboxX() {
    }

    public void onXboxY() {
    }

    public void onXboxLB() {
    }

    public void onXboxRB() {
    }

    public void onXboxBack() {
    }

    public void onXboxStart() {
    }

    public void onXboxLS() {
    }

    public void onXboxRS() {
    }

}
