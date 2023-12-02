package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

// test comment from Christopher
public class HWProfile {
    /* Public OpMode members. */
    public MotorEx motorLeftFront   =   null;
    public MotorEx motorRightFront  =   null;
    public MotorEx motorLeftRear    =   null;
    public MotorEx motorRightRear   =   null;
    public DcMotorEx motorRightLift   =   null;
    public DcMotorEx motorLeftLift    =   null;
//    public DcMotorEx winchMotor    =   null;

//    public DcMotorEx lamp   =   null;

    public RevIMU imu =                 null;
    public DistanceSensor sensorCone;


    public DcMotor motorLF   = null;
    public DcMotor  motorLR  = null;
    public DcMotor  motorRF     = null;
    public DcMotor  motorRR    = null;
    //    public DcMotor motorLeftLift = null;
//    public DcMotor motorRightLift = null;
//    public BNO055IMU imu = null;
    public Servo servoGrabber = null;
    public Servo servoFinger = null;
    public Servo clawAxis = null;
    public Servo droneLauncher = null;
    public Servo launcherServo = null;
    public Servo bucketAxisServo = null;

    public DcMotorEx slidesMotor = null;
    //    public final DistanceSensor armSensor = null;


    public final double DRIVE_TICKS_PER_INCH = 44;      //temporary values => To be updated
    public final int LIFT_RESET = 0;
    public final int LIFT_LOW_JUNCTION = 380;
    public final int LIFT_MID_JUNCTION = 730;
    public final int LIFT_HIGH_JUNCTION = 1270;
    public final int LIFT_MAX_HEIGHT = 1300;
    public final int LIFT_CONE_5 = 160;
    public final double CONE_WAIT_TIME = 4;
    public final int LIFT_CONE_4 = 140;
    public final int LIFT_CONE_3 = 105;
    public final int LIFT_CONE_2 = 60;
    public final int LIFT_CONE_1 = 0;       // can use LIFT_RESET instead of this level
    public final double LIFT_POWER_UP = 1;
    public final double LIFT_POWER_DOWN = 1;
    public final double LIFT_POSITION_TOLERANCE = 10;
    public final double LIFT_kP = 0.005;
    public final double LIFT_kI = 0.005;
    public final double LIFT_kD = 1.05;
    public final double CONE_DISTANCE = 5;
    public final double LIFT_kF = 0.7;
    public final double WAIT_DRIVE_TO_CONE = 1;
<<<<<<< HEAD
    public final double CLAW_OPEN = .25;
=======
    public final double CLAW_OPEN = .35;
    public final double CLAW_CLOSE = 0.5;
>>>>>>> parent of 1f5742e (yay)
    public final double TURN_SPEED = 0.5;
    public final double WINCH_POWER = 1;

    public final double DRIVE_TO_CONE_POWER = 0.2;
    public final double STRAFE_FACTOR = 1.1;
    public final double FINGER_IN = 0.5;
    public final double FINGER_OUT = 0.7;



    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public HWProfile(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define Motors utilizing FTCLib class
        motorLeftFront = new MotorEx(hwMap, "motorLF", Motor.GoBILDA.RPM_312);
        motorLeftFront.setRunMode(Motor.RunMode.RawPower);
        motorLeftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLeftFront.setInverted(false);
        motorLeftFront.resetEncoder();

        motorLeftRear = new MotorEx(hwMap, "motorLR", Motor.GoBILDA.RPM_312);
        motorLeftRear.setRunMode(Motor.RunMode.RawPower);
        motorLeftRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLeftRear.setInverted(false);
        motorLeftRear.resetEncoder();

        motorRightFront = new MotorEx(hwMap, "motorRF", Motor.GoBILDA.RPM_312);
        motorRightFront.setRunMode(Motor.RunMode.RawPower);
        motorRightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setInverted(true);
        motorRightFront.resetEncoder();

        motorRightRear = new MotorEx(hwMap, "motorRR", Motor.GoBILDA.RPM_312);
        motorRightRear.setRunMode(Motor.RunMode.RawPower);
        motorRightRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorRightRear.setInverted(false);
        motorRightRear.resetEncoder();




        // Define and Initialize Motors
        motorLF = hwMap.get(DcMotor.class, "motorLF");
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLF.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setPower(0);


        motorLR = hwMap.get(DcMotor.class, "motorLR");
        motorLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLR.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        motorLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLR.setPower(0);


        motorRF = hwMap.get(DcMotor.class, "motorRF");
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setDirection(DcMotor.Direction.REVERSE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setPower(0);


        motorRR = hwMap.get(DcMotor.class, "motorRR");
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRR.setDirection(DcMotor.Direction.REVERSE);
        motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRR.setPower(0);

//        lamp = hwMap.get(DcMotorEx.class, "lamp");
//        lamp.setPower(0);

        motorLeftLift = hwMap.get(DcMotorEx.class, "motorLeftLift");
        motorLeftLift.setDirection(DcMotorEx.Direction.REVERSE);
        motorLeftLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftLift.setTargetPosition(0);
        motorLeftLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorLeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftLift.setPower(0);               // set motor power

        motorRightLift = hwMap.get(DcMotorEx.class, "motorRightLift");
        motorRightLift.setDirection(DcMotorEx.Direction.FORWARD);
        motorRightLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRightLift.setTargetPosition(0);
        motorRightLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorRightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightLift.setPower(0);               // set motor power

        slidesMotor = hwMap.get(DcMotorEx.class, "motorSlides");
        slidesMotor.setDirection(DcMotorEx.Direction.FORWARD);
        slidesMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setTargetPosition(0);
        slidesMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesMotor.setPower(0);               // set motor power

//        winchMotor = hwMap.get(DcMotorEx.class, "winchMotor");
//        winchMotor.setDirection(DcMotorEx.Direction.REVERSE);
//        winchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        winchMotor.setTargetPosition(0);
//        winchMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        winchMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        servoGrabber = hwMap.get(Servo.class, "servoGrabber");
//        servoFinger = hwMap.get(Servo.class, "servoFinger");
        launcherServo = hwMap.get(Servo.class, "launcherServo");
        droneLauncher = hwMap.get(Servo.class, "droneLauncher");
        bucketAxisServo = hwMap.get(Servo.class, "servoBucketAxis");
        clawAxis = hwMap.get(Servo.class, "clawAxisServo");
        // init distance sensor
//        sensorCone = hwMap.get(DistanceSensor.class, "sensorCone");

        // imu init
        imu = new RevIMU(hwMap);
        imu.init();

        /*
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
         */

    }
}  // end of HWProfile Class