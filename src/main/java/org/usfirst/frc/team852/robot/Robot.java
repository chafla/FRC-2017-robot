package org.usfirst.frc.team852.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import org.athenian.BaseMqttCallback;
import org.athenian.Utils;
import org.eclipse.paho.client.mqttv3.IMqttMessageListener;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.usfirst.frc.team852.robot.data.CameraGearData;
import org.usfirst.frc.team852.robot.data.LidarData;
import org.usfirst.frc.team852.robot.strategies.MattStrategy;
import org.usfirst.frc.team852.robot.strategies.Strategy;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

//import org.eclipse.paho.client.mqttv3.MqttException;


/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends SampleRobot {
    public static final String CAMERA_GEAR_LOGGING_POSITION_TOPIC = "logging/camera/gear/alignment";
    public static final String LIDAR_GEAR_LOGGING_POSITION_TOPIC = "logging/lidar/gear/distance";
    public static final String LIDAR_CLIMBER_LOGGING_POSITION_TOPIC = "logging/lidar/climber/distance";
    private static final String mqtt_topic = "roborio/keyboard/command";
    private static final String CAMERA_TOPIC = "camera/gear/x";
    private static final String LEFT_LIDAR_TOPIC = "lidar/left/mm";
    private static final String RIGHT_LIDAR_TOPIC = "lidar/right/mm";
    private static final String MQTT_HOSTNAME = "mqtt-turtle.local"; /*"10.8.52.14";*/
    private static final int MQTT_PORT = 1883;
    private static final int XBOX_A = 1;
    private static final int XBOX_B = 2;
    private static final int XBOX_X = 3;
    private static final int XBOX_Y = 4;
    private static final double s_deadZone = 0.05;
    final IMqttMessageListener messageListener = (topic, msg) -> {

        // This print prevents the Driver Station from connecting with the robot.
        // We don't know why.
        //      	 System.out.println("In messageArrived: topic=" + topic + ", msg=" + msg.getPayload());

        if (this.listenerEnabled(0 /* might want different flags for what to listen for? */)) {
            String cmdmsg = new String(msg.getPayload());
            if (cmdmsg != null) {
                try {
                    Integer x, width;
                    String[] vals = cmdmsg.split(":");
                    if (topic.equalsIgnoreCase("camera/gear/x") &&
                            vals.length == 2 &&
                            (x = this.valsGetInt(vals, 0)) != null &&
                            (width = this.valsGetInt(vals, 1)) != null &&
                            x.intValue() >= 0 && x.intValue() <= 1000 &&
                            width.intValue() > 0 && width.intValue() <= 1000) {

                        //cameraGearMessage.set(x.intValue(), width.intValue());
                    }
                }
                catch (Exception ex) {
                    System.out.println("Invalid message, topic=" + topic + ", msg=" + cmdmsg);
                }
            }
        }
    };
    // Drives practice robot with battery at FRONT
    private final CANTalon frontLeft = new CANTalon(4);
    private final CANTalon frontRight = new CANTalon(7);
    private final CANTalon rearLeft = new CANTalon(0);
    private final CANTalon rearRight = new CANTalon(3);
    private final Joystick stick1 = new Joystick(0);
    private final Joystick stick2 = new Joystick(1);
    private final Joystick xbox = new Joystick(2);
    private final AtomicReference<CameraGearData> cameraGearRef = new AtomicReference<>();
    private final AtomicReference<LidarData> leftLidarRef = new AtomicReference<>();
    private final AtomicReference<LidarData> rightLidarRef = new AtomicReference<>();
    private final RobotDrive robotDrive;
    private long cameraLastTime = 0;
    private long lidarLeftLastTime = 0;
    private long lidarRightLastTime = 0;
    private AtomicReference<MqttClient> clientRef = new AtomicReference<>();
    private CameraGearData currentCameraGear = null;
    private LidarData currentLeftLidar = null;
    private LidarData currentRightLidar = null;

    private final ExecutorService logExecutor = Executors.newFixedThreadPool(4);
    private final ExecutorService mqttExecutor = Executors.newFixedThreadPool(1);

    private final Strategy strategy = new MattStrategy();

	/*
     * Initialize a talon so that speed control will work.
	 * You can still use voltage-based control!
	 */

    public Robot() throws InterruptedException {
        // Set up the drive talons for velocity based PID on #0, use
        // this.setAllTalonsSpeedMode(true) to turn velocity based PID on,
        // or this.setAllTalonsSpeedMode(false ) to turn velocity based PID off.
        // Looks like maximum RPM (i.e joystick full up) is 400 RPM.

        this.initAllTalons();

        this.robotDrive = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);

        // Motors on one side are reversed, so unless the red/black wires are
        // reversed we switch them here. May need to use the right side
        // instead, but left side works with practice chassis.

        this.robotDrive.setInvertedMotor(MotorType.kFrontRight, true);
        this.robotDrive.setInvertedMotor(MotorType.kRearRight, true);

        //robotDrive.setExpiration(0.1);
        // If robotDrive.something isn't called every 0.1 second we are hosed. Set timeout to 600 seconds.
        // This is not particularly safe - the whole point of this is to handle a case of an infinite loop.
        // I think we can also call robotDrive.setSafetyEnabled(false) below to disable this??
        // (No, didn't seem to work.)
        this.robotDrive.setExpiration(600.0);

        // Set normal drive mode (don't use velocity PID)

        this.mqttExecutor.submit(() -> {
            // Create the mqtt client and subscribe to messages from the broker.
            // We modified createMqttClient in org.athenian.Utils.java so that it
            // sets the option to automatically reconnect if the connection fails!
            while (true) {
                final MqttClient client = Utils.createMqttClient(MQTT_HOSTNAME, MQTT_PORT, new BaseMqttCallback());
                if (client != null) {
                    clientRef.set(client);
                    break;
                }
                System.out.println("MqttClient: Error connecting to server");
                Utils.sleepSecs(1);
            }

            subscribeToTopics(getClient());

        });
    }

    private void initAllTalons() {
        // All sensors need to be reversed on the practice chassis.
        // This could change on the final chassis, use "test" mode to verify
        // that a postivie motor output corresponds with a positive encoder
        // velocity!

        this.initTalon(this.frontRight, true, 250);
        this.initTalon(this.frontLeft, true, 250);
        this.initTalon(this.rearRight, true, 250);
        this.initTalon(this.rearLeft, true, 360);  // new e4t encoder
    }

    private void initTalon(final CANTalon talon, final boolean reverse, final int codesPerRev) {
        // Set up the drive talons for velocity based PID
        // Choose sensor
        talon.setFeedbackDevice(FeedbackDevice.QuadEncoder); // this is default device
        talon.reverseSensor(reverse);  // "+" drive must have + speed - if not set this true!
        talon.configEncoderCodesPerRev(codesPerRev);  // New ones (e4t) are 360, old (e4p) are 250
        // Set up nominal and max output voltages
        talon.configNominalOutputVoltage(0.0f, 0.0f);
        talon.configPeakOutputVoltage(12.0f, -12.0f);
        // Use profile #0, init values
        talon.setProfile(0);
        talon.setF(1.7);
        talon.setP(2.25);
        talon.setI(.005);
        talon.setD(25.0);
    }

    /*
     * Autonomous mode
     *
     */
    @Override
    public void autonomous() {

    }

    /*
     * Test mode
     *
     */
    @Override
    public void test() {
        this.robotDrive.setSafetyEnabled(true);

        int loops = 0;
        while (isTest() && isEnabled()) {

            CANTalon t = this.rearRight;

            double v = this.stick1.getY();
            double motorout = t.getOutputVoltage() / t.getBusVoltage();
            double speed = t.getSpeed();
            double targetSpeed = 0.0;
            int err = 0;
            //s += "out:" + motorout;
            //s += "\tspd:" + t.getSpeed();

            if (this.stick1.getRawButton(1)) {
                // Speed mode
                targetSpeed = ((int) (Math.abs(v) * 10.0) / 3) * 100.0;

                t.changeControlMode(TalonControlMode.Speed);
                t.set(targetSpeed);
                err = t.getClosedLoopError();
                //s += "\terr:" + t.getClosedLoopError();
                //s += "\ttrg:" + targetSpeed;  // target speed
            } else {
                // percent voltage mode
                t.changeControlMode(TalonControlMode.PercentVbus);
                t.set(v);
            }
            if (++loops >= 50) {
                loops = 0;
                if (targetSpeed != 0.0)
                    System.out.printf("out:%4.2f spd:%5.1f err:%4d trg:%5.1f\n", motorout, speed, err, targetSpeed);
                else if (speed > 0.0001 || speed < -.0001)
                    System.out.printf("out:%4.2f spd:%5.1f\n", motorout, speed);
            }
        }

    }

    /**
     * Runs the motors with Mecanum drive.
     */
    @Override
    public void operatorControl() {
        this.robotDrive.setSafetyEnabled(true);

		/*
         * For speed mode (true), sets talon control mode to Speed and max output to 400 RPM.
		 * For normal mode (false), sets talon control mode to %vbus and max output to 1.0.
		 */

        boolean s2button1 = false;
        boolean speedMode = false;
        boolean goRight = true;

        while (isOperatorControl() && isEnabled()) {

            // Use the joystick X axis for lateral movement, Y axis for forward
            // movement, and Z axis for rotation.
            // This sample does not use field-oriented drive, so the gyro input
            // is set to zero.
            //robotDrive.mecanumDrive_Cartesian(stick.getX(), stick.getY(), stick.getZ(), 0);

            // These are set once per iteration of the loop
            this.currentCameraGear = this.cameraGearRef.get();
            this.currentLeftLidar = this.leftLidarRef.get();
            this.currentRightLidar = this.rightLidarRef.get();

            if (this.xbox.getRawButton(XBOX_A))
                this.strategy.xboxAButtonPressed(this);
            else if (this.xbox.getRawButton(XBOX_B))
                this.strategy.xboxBButtonPressed(this);
            else if (this.xbox.getRawButton(XBOX_Y))
                this.strategy.xboxYButtonPressed(this);

            // NOTE! Left/right movement may be reversed, may need to modify signs!

            // Drive using joysticks
            // This version drives two different ways. Slide stick 1's z-axis button to switch
            // between them.
            //Args are: (left(-1) - right(+1), backward(+1) - forward(-1),
            //				 rate-of-rotation(-1 - +1), gyro-angle-unused-here(double?))
            // robotDrive switches the sign of the forward-backward (Y) value so that
            // the stick full forward value of (-1) is switched to (+1), and the
            // stick full backward value of (+1) is switched to (-1).
            // (Airplanes are flown with the stick pulled BACK to put the nose UP,
            // that is why joysticks do it that way.)

            // Velocity drive needs a larger deadzone, and we can't extend Joystick.
            if (this.stick1.getZ() < 0)
                this.robotDrive.mecanumDrive_Cartesian(this.adjustDeadzone(this.stick1.getX()),
                                                       -this.adjustDeadzone(this.stick1.getY()),
                                                       this.adjustDeadzone(this.stick2.getX()),
                                                       0);
            else
                this.robotDrive.mecanumDrive_Cartesian((this.adjustDeadzone(this.stick1.getX()) + this.adjustDeadzone(this.stick2.getX())) / 2,
                                                       (this.adjustDeadzone(stick1.getY()) + this.adjustDeadzone(this.stick2.getY())) / 2,
                                                       (this.adjustDeadzone(this.stick2.getY()) - this.adjustDeadzone(this.stick1.getY())) / 2, 0);
            /*
            double x1 = stick1.getX();
            double y1 = stick1.getY();
            double x2 = stick2.getX();
            double y2 = stick2.getY();

            x1 = this.adjustDeadzone(x1); // this.adjustDeadzone(stick1.getX())
            y1 = this.adjustDeadzone(y1); // this.adjustDeadzone(stick1.getY())
            x2 = this.adjustDeadzone(x2);
            y2 = this.adjustDeadzone(y2);

            if (stick1.getZ() < 0)
                robotDrive.mecanumDrive_Cartesian(x1, y1, x2, 0);
            else
                robotDrive.mecanumDrive_Cartesian((x1 + x2) / 2, (y1 + y2) / 2, -(y1 - y2) / 2, 0);
            */
            Timer.delay(0.005); // wait 5ms to avoid hogging CPU cycles
        }
    }

    public CameraGearData getCurrentCameraGear() {
        return this.currentCameraGear;
    }

    public LidarData getCurrentLeftLidar() {
        return this.currentLeftLidar;
    }

    public LidarData getCurrentRightLidar() {
        return this.currentRightLidar;
    }

    public long getCameraLastTime() {
        return this.cameraLastTime;
    }

    public long getLidarLeftLastTime() {
        return this.lidarLeftLastTime;
    }

    public long getLidarRightLastTime() {
        return this.lidarRightLastTime;
    }

    public void updateCameraLastTime() {
        this.cameraLastTime = System.currentTimeMillis();
    }

    public void updateLidarLeftLastTime() {
        this.lidarLeftLastTime = System.currentTimeMillis();
    }

    public void updateLidarRightLastTime() {
        this.lidarRightLastTime = System.currentTimeMillis();
    }

    public MqttClient getClient() {
        return this.clientRef.get();
    }

    public void drive(final double x,
                      final double y,
                      final double rot,
                      final double delay,
                      final SensorType sensorType,
                      final String logMsg) {
        this.logMsg(sensorType, logMsg);
        this.robotDrive.mecanumDrive_Cartesian(x, y, rot, 0);
        Timer.delay(delay);
        this.robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
    }

    public void logMsg(final SensorType sensorType, final String desc) {
        this.logExecutor.submit(
                () -> {
                    try {
                        final String msg = sensorType.getMsgGenerator().getMesssage(Robot.this, desc);
                        final MqttClient client = this.getClient();
                        if (client != null)
                            client.publish(sensorType.getTopic(), new MqttMessage(msg.getBytes()));
                    }
                    catch (MqttException e1) {
                        e1.printStackTrace();
                    }
                });
    }

    // May want multiple listeners, one per topic - or one listener
    // which gets all topics. Not sure which at this point.

    private boolean listenerEnabled(final int listenerType) {
        // probably also want to check that an xbox button hasn't been pressed!
        // and may want a timeout here as well
        return isEnabled() && isOperatorControl();
    }

    private void subscribeToTopics(final MqttClient client) {

        System.out.println("Subscribing to topics");

        try {
            client.subscribe(CAMERA_TOPIC,
                             (topic, msg) -> {
                                 try {
                                     String cmdmsg = new String(msg.getPayload());
                                     String[] info = cmdmsg.split(":");
                                     int currloc = Integer.parseInt(info[0]);
                                     int width = Integer.parseInt(info[1]);
                                     cameraGearRef.set(new CameraGearData(currloc, width));
                                 }
                                 catch (Exception e) {
                                     e.printStackTrace();
                                 }
                             });
        }
        catch (MqttException e1) {
            e1.printStackTrace();
        }

        // subscribe to left lidar topic
        try {
            client.subscribe(LEFT_LIDAR_TOPIC,
                             (topic, msg) -> {
                                 try {
                                     final String cmdmsg = new String(msg.getPayload());
                                     final int dist = Integer.parseInt(cmdmsg);
                                     leftLidarRef.set(new LidarData(dist));
                                 }
                                 catch (Exception e) {
                                     System.out.println("Error in subscribe:" + e.getMessage());
                                     e.printStackTrace();
                                 }
                             });
        }
        catch (Exception e) {
            System.out.println("Error in subscribe:" + e.getMessage());
            e.printStackTrace();
        }

//        try {
//            client.subscribe(left_lidar, new IMqttMessageListener() {
//                @Override
//                public void messageArrived(String s, MqttMessage mqttMessage) throws Exception {
//                    try {
//                        client.publish("lidar/right/logging", new MqttMessage(mqttMessage.getPayload()));
//                    } catch (Exception e) {
//                        e.printStackTrace();
//                    }
//                }
//            });
//        } catch (Exception e) {
//            e.printStackTrace();
//        }

        // subscribe to right lidar topic
        try {
            client.subscribe(RIGHT_LIDAR_TOPIC,
                             (topic, msg) -> {
                                 try {
                                     final String cmdmsg = new String(msg.getPayload());
                                     final int dist = Integer.parseInt(cmdmsg);
                                     rightLidarRef.set(new LidarData(dist));
                                 }
                                 catch (Exception e) {
                                     System.out.println("Error in subscribe:" + e.getMessage());
                                     e.printStackTrace();
                                 }
                             });
        }
        catch (Exception e) {
            System.out.println("Error in subscribe:" + e.getMessage());
            e.printStackTrace();
        }
    }

    private Integer valsGetInt(final String[] vals, final int idx) {
        Integer ret;
        try {
            ret = new Integer(vals[idx]);
        }
        catch (Exception ex) {
            ret = null;
        }
        return ret;
    }


	/*
     * Add a dead zone around the joystick center, needed with
	 * PID velocity based control.
	 */

    public double adjustDeadzone(double val) {
        if (val >= -s_deadZone && val <= s_deadZone) val = 0.0;
        return val;
    }

    public boolean getXboxButton(XButton button) {

        return xbox.getRawButton(button.get());

    }

}


