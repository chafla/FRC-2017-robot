package org.usfirst.frc.team852.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import org.athenian.BaseMqttCallback;
import org.athenian.Utils;
import org.eclipse.paho.client.mqttv3.*;
import org.usfirst.frc.team852.robot.data.CameraData;
import org.usfirst.frc.team852.robot.data.DataType;
import org.usfirst.frc.team852.robot.data.HeadingData;
import org.usfirst.frc.team852.robot.data.LidarData;
import org.usfirst.frc.team852.robot.strategy.JvStrategy;
import org.usfirst.frc.team852.robot.strategy.Strategy;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

import static java.lang.String.format;

//import org.eclipse.paho.client.mqttv3.MqttException;


/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends SampleRobot {
    private static final int XBOX_A = 1;
    private static final int XBOX_B = 2;
    private static final int XBOX_X = 3;
    private static final int XBOX_Y = 4;
    private static final int XBOX_LB = 5;
    private static final int XBOX_RB = 6;
    private static final int XBOX_Back = 7;
    private static final int XBOX_Start = 8;
    private static final int XBOX_LS = 9;
    private static final int XBOX_RS = 10;

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
    private final CANTalon frontLeft = new CANTalon(3);
    private final CANTalon frontRight = new CANTalon(2);
    private final CANTalon rearLeft = new CANTalon(0);
    private final CANTalon rearRight = new CANTalon(1);
    private final CANTalon rackAndPinion = new CANTalon(5);
    private final CANTalon climber = new CANTalon(4);
    private final DoubleSolenoid piston = new DoubleSolenoid(0, 1);
    private final DigitalInput rightLimitSwitch = new DigitalInput(0);
    private final DigitalInput leftLimitSwitch = new DigitalInput(1);
    private final Relay ring = new Relay(0);
    private final Joystick stick1 = new Joystick(0);
    private final Joystick stick2 = new Joystick(1);
    private final Joystick xbox = new Joystick(2);
    private final AtomicReference<CameraData> cameraGearRef = new AtomicReference<>();
    private final AtomicReference<LidarData> frontLidarRef = new AtomicReference<>();
    private final AtomicReference<LidarData> rearLidarRef = new AtomicReference<>();
    private final AtomicReference<LidarData> leftLidarRef = new AtomicReference<>();
    private final AtomicReference<LidarData> rightLidarRef = new AtomicReference<>();
    private final AtomicReference<HeadingData> headingRef = new AtomicReference<>();
    private final ExecutorService logExecutor = Executors.newFixedThreadPool(4);
    private final ExecutorService mqttExecutor = Executors.newFixedThreadPool(1);
    private final Strategy strategy = new JvStrategy();
    private final AtomicReference<MqttClient> mqttClientRef = new AtomicReference<>();

    private CameraData currentCameraGear = null;
    private LidarData currentFrontLidar = null;
    private LidarData currentRearLidar = null;
    private LidarData currentLeftLidar = null;
    private LidarData currentRightLidar = null;
    private HeadingData currentHeading = null;

    private final RobotDrive robotDrive;
    // PRA private final MqttReconnect mqttReconnect;

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
        this.piston.set(DoubleSolenoid.Value.kOff);
        this.robotDrive = new RobotDrive(this.frontLeft, this.rearLeft, this.frontRight, this.rearRight);

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

        this.mqttExecutor.submit((Runnable) () -> {
            final String url = format("tcp://%s:%d", Constants.MQTT_HOSTNAME, Constants.MQTT_PORT);

            final MqttConnectOptions opts = new MqttConnectOptions();
            opts.setCleanSession(true);
            opts.setAutomaticReconnect(true);
            opts.setConnectionTimeout(30);

            while (true) {
                this.mqttClientRef.set(Utils.createMqttClient(url,
                                                              new BaseMqttCallback() {
                                                                  @Override
                                                                  public void connectComplete(boolean reconnect, String url) {
                                                                      super.connectComplete(reconnect, url);
                                                                      subscribeToTopics(getMqttClient());
                                                                  }
                                                              }));
                if (this.mqttClientRef.get() != null) {
                    /*
                    DisconnectedBufferOptions bufferOpts = new DisconnectedBufferOptions();
                    bufferOpts.setBufferEnabled(true);
                    bufferOpts.setBufferSize(100); // 100 message buffer
                    bufferOpts.setDeleteOldestMessages(true); // Purge oldest messages when buffer is full
                    bufferOpts.setPersistBuffer(false); // Do not buffer to disk
                    this.mqttClientRef.get().setBufferOpts(bufferOpts);
                    */

                    try {
                        System.out.println(format("Connecting to MQTT broker at %s...", url));
                        this.mqttClientRef.get().connect(opts);
                        System.out.println(format("Connected to %s", url));
                        break;
                    }
                    catch (MqttException e) {
                        System.out.println(format("Cannot connect to MQTT broker at %s [%s]", url, e.getMessage()));
                        e.printStackTrace();
                        this.mqttClientRef.set(null);
                    }
                }

                System.out.println("Error connecting to MQTT broker");
                Utils.sleepSecs(1);
            }
        });

        /* PRA
        this.mqttReconnect = new MqttReconnect(Constants.MQTT_HOSTNAME,
                                               Constants.MQTT_PORT,
                                               30, this::subscribeToTopics);
        this.mqttReconnect.start();
        */
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
        final CameraData cameraData = getCurrentCameraGear();
        final LidarData frontLidarData = getCurrentFrontLidar();
        final LidarData rearLidarData = getCurrentRearLidar();

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

        ring.set(Relay.Value.kForward);

        while (isOperatorControl() && isEnabled()) {

            // Use the joystick X axis for lateral movement, Y axis for forward
            // movement, and Z axis for rotation.
            // This sample does not use field-oriented drive, so the gyro input
            // is set to zero.
            //robotDrive.mecanumDrive_Cartesian(stick.getX(), stick.getY(), stick.getZ(), 0);

            // These are set once per iteration of the loop
            this.currentCameraGear = this.cameraGearRef.get();
            this.currentFrontLidar = this.frontLidarRef.get();
            this.currentRearLidar = this.rearLidarRef.get();
            this.currentLeftLidar = this.leftLidarRef.get();
            this.currentRightLidar = this.rightLidarRef.get();
            this.currentHeading = this.headingRef.get();

            if (this.xbox.getRawButton(XBOX_A))
                this.strategy.onXboxA(this);
            else if (this.xbox.getRawButton(XBOX_B))
                this.strategy.onXboxB(this);
            else if (this.xbox.getRawButton(XBOX_X))
                this.strategy.onXboxX(this);
            else if (this.xbox.getRawButton(XBOX_Y))
                this.strategy.onXboxY(this);
            else if (this.xbox.getRawButton(XBOX_LB))
                this.strategy.onXboxLB(this);
            else if (this.xbox.getRawButton(XBOX_RB))
                this.strategy.onXboxRB(this);
            else if (this.xbox.getRawButton(XBOX_Back))
                this.strategy.onXboxBack(this);
            else if (this.xbox.getRawButton(XBOX_Start))
                this.strategy.onXboxStart(this);
            else if (this.xbox.getRawButton(XBOX_LS))
                this.strategy.onXboxLS(this);
            else if (this.xbox.getRawButton(XBOX_RS))
                this.strategy.onXboxRS(this);


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
            Timer.delay(0.005); // wait 5ms to avoid hogging CPU cycles
        }
    }

    public CameraData getCurrentCameraGear() {
        return this.currentCameraGear;
    }

    public LidarData getCurrentFrontLidar() {
        return this.currentFrontLidar;
    }

    public LidarData getCurrentRearLidar() {
        return this.currentRearLidar;
    }

    public LidarData getCurrentLeftLidar() {
        return this.currentLeftLidar;
    }

    public LidarData getCurrentRightLidar() {
        return this.currentRightLidar;
    }

    public HeadingData getCurrentHeading() {
        return this.currentHeading;
    }

    public MqttClient getMqttClient() {
        return this.mqttClientRef.get();
        // PRA return this.mqttReconnect.getMqttClient();
    }

    public void waitOnCameraGear(final long timeoutMillis) {
        synchronized (this.cameraGearRef) {
            try {
                this.cameraGearRef.wait(timeoutMillis);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void waitOnFrontLidar(final long timeoutMillis) {
        synchronized (this.frontLidarRef) {
            try {
                this.frontLidarRef.wait(timeoutMillis);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void waitOnRearLidar(final long timeoutMillis) {
        synchronized (this.rearLidarRef) {
            try {
                this.rearLidarRef.wait(timeoutMillis);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void waitOnLeftLidar(final long timeoutMillis) {
        synchronized (this.leftLidarRef) {
            try {
                this.leftLidarRef.wait(timeoutMillis);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void waitOnRightLidar(final long timeoutMillis) {
        synchronized (this.rightLidarRef) {
            try {
                this.rightLidarRef.wait(timeoutMillis);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void waitOnHeading(final long timeoutMillis) {
        synchronized (this.headingRef) {
            try {
                this.headingRef.wait(timeoutMillis);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
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

    public void pushGear() {
        this.piston.set(DoubleSolenoid.Value.kForward);
    }

    public void retractPiston() {
        this.piston.set(DoubleSolenoid.Value.kReverse);
    }

    public void moveRandPRight() {
        if (!this.rightLimitSwitch.get()) { // consider switching to while
            this.rackAndPinion.set(0.2); // may need to reverse
            Timer.delay(0.01);
            this.rackAndPinion.set(0);
        }
    }

    public void moveRandPLeft() {
        if (!this.leftLimitSwitch.get()) {
            this.rackAndPinion.set(-0.2);
            Timer.delay(0.01);
            this.rackAndPinion.set(0);
        }
    }

    public void climb() {
        climber.set(0.3);
    }

    public void logMsg(final SensorType sensorType, final String desc) {
        this.logExecutor.submit(
                () -> {
                    try {
                        final String msg = sensorType.getMsgGenerator().getMesssage(Robot.this, desc);
                        final MqttClient client = this.getMqttClient();
                        if (client != null)
                            client.publish(sensorType.getTopic(), new MqttMessage(msg.getBytes()));
                    }
                    catch (Throwable e) {
                        System.out.println(format("Error in logMsg() [%s - %s]",
                                                  e.getClass().getSimpleName(),
                                                  e.getLocalizedMessage()));
                        e.printStackTrace();
                    }
                });
    }

    private boolean listenerEnabled(final int listenerType) {
        // probably also want to check that an xbox button hasn't been pressed!
        // and may want a timeout here as well
        return isEnabled() && isOperatorControl();
    }

    private void subscribeToTopics(final MqttClient client) {
        System.out.println("Subscribing to topics");
        try {
            client.subscribe(Constants.CAMERA_TOPIC,
                             (topic, msg) -> {
                                 final String[] info = new String(msg.getPayload()).split(":");
                                 final int currloc = Integer.parseInt(info[0]);
                                 final int width = Integer.parseInt(info[1]);
                                 this.cameraGearRef.set(new CameraData(DataType.CameraGear, currloc, width));
                                 synchronized (this.cameraGearRef) {
                                     this.cameraGearRef.notifyAll();
                                 }
                             });

            client.subscribe(Constants.FRONT_LIDAR_TOPIC,
                             (topic, msg) -> {
                                 final int dist = Integer.parseInt(new String(msg.getPayload()));
                                 this.frontLidarRef.set(new LidarData(DataType.FrontLidar, dist));
                                 synchronized (this.frontLidarRef) {
                                     this.frontLidarRef.notifyAll();
                                 }
                             });

            client.subscribe(Constants.REAR_LIDAR_TOPIC,
                             (topic, msg) -> {
                                 final int dist = Integer.parseInt(new String(msg.getPayload()));
                                 this.rearLidarRef.set(new LidarData(DataType.RearLidar, dist));
                                 synchronized (this.rearLidarRef) {
                                     this.rearLidarRef.notifyAll();
                                 }
                             });

            client.subscribe(Constants.LEFT_LIDAR_TOPIC,
                             (topic, msg) -> {
                                 final int dist = Integer.parseInt(new String(msg.getPayload()));
                                 this.leftLidarRef.set(new LidarData(DataType.LeftLidar, dist));
                                 synchronized (this.leftLidarRef) {
                                     this.leftLidarRef.notifyAll();
                                 }
                             });

            client.subscribe(Constants.RIGHT_LIDAR_TOPIC,
                             (topic, msg) -> {
                                 final int dist = Integer.parseInt(new String(msg.getPayload()));
                                 this.rightLidarRef.set(new LidarData(DataType.RightLidar, dist));
                                 synchronized (this.rightLidarRef) {
                                     this.rightLidarRef.notifyAll();
                                 }
                             });

            client.subscribe(Constants.HEADING_TOPIC,
                             (topic, msg) -> {
                                 final double degree = Double.parseDouble(new String(msg.getPayload()));
                                 this.headingRef.set(new HeadingData(degree));
                                 synchronized (this.headingRef) {
                                     this.headingRef.notifyAll();
                                 }
                             });
        }
        catch (MqttException e) {
            System.out.println(format("Error in subscribeToTopics() [%s - %s]",
                                      e.getClass().getSimpleName(),
                                      e.getLocalizedMessage()));
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


