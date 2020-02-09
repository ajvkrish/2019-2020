package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Hardware definitions and access for a robot with a four-motor
 * drive train and a gyro sensor.
 */
public class Robot {
    static final double     COUNTS_PER_MOTOR_REV    = 145.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    public final DcMotor lf, lr, rf, rr,slide;


    public final Servo grabber,track;

    private final BNO055IMU imu;

    private double headingOffset = 0.0;
    private Orientation angles;
    private Acceleration gravity;
    private int armLevel0 = 0;
    private int armLevel1 = 0;
    private int armLevel2 = 0;

    public Robot(final HardwareMap _hardwareMap, final Telemetry _telemetry) {
        hardwareMap = _hardwareMap;
        telemetry = _telemetry;





//        trayL = hardwareMap.servo.get("trayL");
//        trayR = hardwareMap.servo.get("trayR");

        grabber = hardwareMap.servo.get("grabber");
        track = hardwareMap.servo.get("track");

        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lr = hardwareMap.dcMotor.get("lr");
        rr = hardwareMap.dcMotor.get("rr");


        slide = hardwareMap.dcMotor.get("slide");


        armLevel1 = armLevel0 - 100;
        armLevel2 = armLevel1 - 100;

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);





        setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE, lf, lr, rf, rr);




        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    private void setMotorMode(DcMotor.RunMode mode, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setMode(mode);
        }
    }

    private void setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(mode);
        }
    }

    public void runUsingEncoders() {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, lf, lr, rf, rr);
    }

    public void runWithoutEncoders() {
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, lf, lr, rf, rr);
    }

    /**
     * @return true if the gyro is fully calibrated, false otherwise
     */
    public boolean isGyroCalibrated() {
        return imu.isGyroCalibrated();
    }

    /**
     * Fetch all once-per-time-slice values.
     * <p>
     * Call this either in your OpMode::loop function or in your while(opModeIsActive())
     * loops in your autonomous. It refresh gyro and other values that are computationally
     * expensive.
     */
    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        gravity = imu.getGravity();
    }

    /**
     * @return the raw heading along the desired axis
     */
    private double getRawHeading() {
        return angles.firstAngle;
    }

    /**
     * @return the robot's current heading in radians
     */
    public double getHeading() {
        return (getRawHeading() - headingOffset) % (2.0 * Math.PI);
    }

    /**
     * @return the robot's current heading in degrees
     */
    public double getHeadingDegrees() { return Math.toDegrees(getHeading()); }

    /**
     * Set the current heading to zero.
     */
    public void resetHeading() {
        headingOffset = getRawHeading();
    }

    /**
     * Find the maximum absolute value of a set of numbers.
     *
     * @param xs Some number of double arguments
     * @return double maximum absolute value of all arguments
     */
    private static double maxAbs(double... xs) {
        double ret = Double.MIN_VALUE;
        for (double x : xs) {
            if (Math.abs(x) > ret) {
                ret = Math.abs(x);
            }
        }
        return ret;
    }

    /**
     * Set motor powers
     * <p>
     * All powers will be scaled by the greater of 1.0 or the largest absolute
     * value of any motor power.
     *
     * @param _lf Left front motor
     * @param _lr Left rear motor
     * @param _rf Right front motor
     * @param _rr Right rear motor
     */
    public void setMotors(double _lf, double _lr, double _rf, double _rr) {
        final double scale = maxAbs(1.0, _lf, _lr, _rf, _rr);
        lf.setPower(_lf / scale);
        lr.setPower(_lr / scale);
        rf.setPower(_rf / scale);
        rr.setPower(_rr / scale);
    }

    public void driveInches(double speed, double inches){
        lf.setPower(speed);
        lr.setPower(speed);
        rf.setPower(speed);
        rr.setPower(speed);

        lf.setTargetPosition((int) (inches * COUNTS_PER_INCH));
        lr.setTargetPosition((int) (inches * COUNTS_PER_INCH));
        rf.setTargetPosition((int) (inches * COUNTS_PER_INCH));
        rr.setTargetPosition((int) (inches * COUNTS_PER_INCH));

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

//    public void setArmPosition(int position)
//    {
//        switch(position)
//        {
//            case 0:
//                arm.setTargetPosition(armLevel0);
//                break;
//            case 1:
//                arm.setTargetPosition(armLevel1);
//                break;
//            case 2:
//                arm.setTargetPosition(armLevel2);
//                break;
//            default:
//                arm.setTargetPosition(armLevel0);
//                break;
//        }
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm.setPower(1);
//    }



    public void stopRobot(){
        lf.setPower(0);
        rf.setPower(0);
        lr.setPower(0);
        rr.setPower(0);
    }

    public void drive(double speed) {
        stopRobot();
        setMotors(speed, speed, speed, speed);

    }

    public void gyroDrive(double x, double y) {
        //final double rotation = Math.pow(controller.right_stick_x, 3.0)/1.5;
        double direction = Math.atan2(x, y) + (getHeading());
        double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        double lf = speed * Math.sin(direction + Math.PI / 4.0);
        double rf = speed * Math.cos(direction + Math.PI / 4.0);
        double lr = speed * Math.cos(direction + Math.PI / 4.0);
        double rr = speed * Math.sin(direction + Math.PI / 4.0);

        setMotors(lf, lr, rf, rr);
    }

    public void gyroDriveSlow(double x, double y) {
        //final double rotation = Math.pow(controller.right_stick_x, 3.0)/1.5;
        double direction = Math.atan2(x, y) + (getHeading());
        double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        double lf = .5 * speed * Math.sin(direction + Math.PI / 4.0);
        double rf = .5 * speed * Math.cos(direction + Math.PI / 4.0);
        double lr = .5 * speed * Math.cos(direction + Math.PI / 4.0);
        double rr = .5 * speed * Math.sin(direction + Math.PI / 4.0);

        setMotors(lf, lr, rf, rr);
    }

    public void pivot() {

    }




    public void driveDistance(double power, int distance) {
        {

            lf.setTargetPosition(distance + lf.getCurrentPosition());
            rf.setTargetPosition(distance + rf.getCurrentPosition());
            lr.setTargetPosition(distance + lr.getCurrentPosition());
            rr.setTargetPosition(distance + rr.getCurrentPosition());

            lf.setPower(power);
            rf.setPower(power);
            lr.setPower(power);
            rr.setPower(power);

            lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);




            stopRobot();
        }
    }

    public void strafeDistance(double power, int distance, String direction)
    {
        lf.setTargetPosition(distance + lf.getCurrentPosition());
        rf.setTargetPosition(distance + rf.getCurrentPosition());
        lr.setTargetPosition(distance + lr.getCurrentPosition());
        rr.setTargetPosition(distance + rr.getCurrentPosition());

        if(direction.charAt(0) == 'r' && direction.charAt(0) == 'R') {
            lf.setPower(power);
            rf.setPower(power*-1);
            lr.setPower(power*-1);
            rr.setPower(power);
        }

        if(direction.charAt(0) == 'l' && direction.charAt(0) == 'L') {
            lf.setPower(power*-1);
            rf.setPower(power);
            lr.setPower(power);
            rr.setPower(power*-1);
        }


        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);





        stopRobot();

    }

    public void setTrackDirection(String direction)
    {
        if(direction.charAt(0) == 'r' || direction.charAt(0) == 'R')
        {
            track.setDirection(Servo.Direction.REVERSE);
        }
        else if(direction.charAt(0) == 'l' || direction.charAt(0) == 'L')
        {
            track.setDirection(Servo.Direction.FORWARD);
        }
        else if(direction.charAt(0) == 's' || direction.charAt(0) == 'S')
        {
            track.setPosition(track.getPosition());
        }
    }

    public void setSlidePosition(int stack)
    {
        slide.setPower(.5);
        switch (stack)
        {
            case 0 :
                slide.setTargetPosition(0); // Move the slides to bottom position
                break;
            case 1 :
                slide.setTargetPosition(0); // Move the slides to above tray height so block will pass
                break;
            case 2 :
                slide.setTargetPosition(1000); // Move the slides for the 2nd block on the stack
                break;
            case 3 :
                slide.setTargetPosition(0); // Move the slides for the 3rd block on the stack
                break;
            case 4 :
                slide.setTargetPosition(0); // Move the slides for the 4th block on the stack
                break;
            case 5 :
                slide.setTargetPosition(0); // Move the slides for the 5th block on the stack
                break;
            case 6 :
                slide.setTargetPosition(0); // Move the slides for the 6th block on the stack
                break;
            case 7 :
                slide.setTargetPosition(0); // Move the slides for the 7th block on the stack
                break;
            case 8 :
                slide.setTargetPosition(0); // Move the slides for the 8th block on the stack
                break;
            case 9 :
                slide.setTargetPosition(0); // Move the slides for the 9th block on the stack
                break;
            case 10 :
                slide.setTargetPosition(0); // Move the slides for the 10th block on the stack
                break;
            default :
                slide.setTargetPosition(0); //Move slide all the way down.
                break;

        }
    }
}

