package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Mecanum teleop (with an optional arcade mode)
 * * Left stick controls x/y translation.
 * * Right stick controls rotation about the z axis
 * * When arcade mode is enabled (press "a"), translation direction
 * becomes relative to the field as opposed to the robot. You can
 * reset the forward heading by pressing "x".
 */
@TeleOp(name = "Mecanum")
public class Mecanum extends OpMode {
    private Robot robot;
    private Controller controller;
    private Controller controller2;
    private boolean arcadeMode = false;
    private boolean slowMode = false;
    private int gyroCalibratedCount = 0;
    private int armPosition = 0;
    private int slidePosition = 0;
    private int lastDirection = 0;
    private int trayCounter = 0;
    private int grabberCounter = 0;
    private int stack = 0;

    private double grabberPosition = 0.0;
    private double intakeLeftPosition = .65;
    private double intakeRightPosition = .5;
    private double trayServoPosition = .8;
    private double rotatorArmPosition = 0.5;
    private MecanumState mecanumState = MecanumState.STOP;
    private long startTime = 0;

    //Notes:
//sequence:
//intakes opening
//raise to 1st position
//close intakes
//open grabber
//run intakes out
//
//wait time in

    // 0 - 360
    private int degreesOpen = 0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        controller = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        robot.slide.setTargetPosition(robot.slide.getCurrentPosition());

    }

    @Override
    public void init_loop() {
        controller.update();
        if (controller.AOnce()) {
            arcadeMode = !arcadeMode;
        }
        if (controller.YOnce()) {
            slowMode = !slowMode;
        }
        telemetry.addData("Gyro Ready?", robot.isGyroCalibrated() ? "YES" : "no.");
        telemetry.addData("Arcade Mode (a)", arcadeMode ? "YES" : "no.");
        telemetry.addData("Slow Mode (s)", slowMode ? "YES" : "no.");
        telemetry.update();


    }

    public static double governor = 0.8;

    @Override
    public void loop() {
        controller.update();
        controller2.update();
        robot.loop();


        if (controller.XOnce()) {
            robot.resetHeading();
        }
        if (controller.AOnce()) {
            arcadeMode = !arcadeMode;
        }
        if (controller.YOnce()) {
            slowMode = !slowMode;
        }
        telemetry.addData("Arcade Mode (a)", arcadeMode ? "YES" : "no.");
        telemetry.addData("Slow Mode (s)", slowMode ? "YES" : "no.");
        telemetry.addData("Heading (reset: x)", robot.getHeadingDegrees());

        final double x = Math.pow(controller.left_stick_x*-1, 3.0);
        final double y = Math.pow(controller.left_stick_y *-1, 3.0);

//        if (mecanumState == mecanumState.STOP) {
//            if (controller2.dpadUpOnce() && armPosition < 2)
//                armPosition++;
//
//            if (controller2.dpadDownOnce() && armPosition > 0)
//                armPosition--;
//
//            robot.setArmPosition(armPosition);
//        }

        if (mecanumState == mecanumState.STOP) {
            if(controller2.BOnce())
            {
                stack++;
            }
            if(controller2.dpadUpOnce())
            {
                robot.setSlidePosition(stack);
            }
            if(controller2.dpadDownOnce())
            {
                stack = 0;
                robot.setSlidePosition(stack);
            }

        }

        telemetry.addData("Stack", stack);





        final double rotation = Math.pow(controller.right_stick_x*-1, 3.0)/1.5;
        final double direction = Math.atan2(x, y) + (arcadeMode ? robot.getHeading() : 0.0);
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        final double lf = (slowMode ? governor*.2 : governor) * speed * Math.sin(direction + Math.PI / 4.0) + rotation;
        final double rf = (slowMode ? governor*.2 : governor) * speed * Math.cos(direction + Math.PI / 4.0) - rotation;
        final double lr = (slowMode ? governor*.2 : governor) * speed * Math.cos(direction + Math.PI / 4.0) + rotation;
        final double rr = (slowMode ? governor*.2 : governor) * speed * Math.sin(direction + Math.PI / 4.0) - rotation;

        robot.setMotors(lf, lr, rf, rr);

        // Grabber code
        if (mecanumState == mecanumState.STOP) {
            if (controller2.XOnce()) {
//                if (grabberCounter % 2 == 0) {
//                    //close grabber
//                    grabberPosition = 0.0;
//                    grabberCounter++;
//                } else {
//                    //open grabber
//                    grabberPosition = 1;
//                    grabberCounter++;
//                }
                robot.grabber.setPosition(robot.grabber.getPosition() + .05);

            }
        }

        telemetry.addData("Grabber", grabberPosition);

        //tray code
        if (controller.BOnce()) {
            if (trayCounter % 2 == 0) {
                trayServoPosition = 1.0;
                trayCounter++;
            } else {
                trayServoPosition = 0.0;
                trayCounter++;
            }

        }


//        robot.trayL.setPosition(trayServoPosition);
//        robot.trayR.setPosition(tray
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//        ServoPosition);
        telemetry.addData("TrayL", trayServoPosition);
        telemetry.addData("TrayR", trayServoPosition);




        if (mecanumState == mecanumState.STOP)
        {
            if(controller2.AOnce())
            {
                robot.track.setPosition(0);
            }
        }

        telemetry.addData("TrackServo", robot.track.getPosition());

        if(mecanumState == mecanumState.STOP)
        {
            robot.slide.setPower(.2);
            robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(controller2.left_stick_y == 0.0)
            {
              //  robot.slide.setTargetPosition(robot.slide.getCurrentPosition());
            }
            else if (controller2.left_stick_y > .5)
            {
                robot.slide.setTargetPosition(robot.slide.getCurrentPosition() + 50);
            }
            else if(controller2.left_stick_y < .5)
            {
                robot.slide.setTargetPosition(robot.slide.getCurrentPosition() - 50);
            }




        }
        telemetry.addData("LEFT JOYSTICK", controller2.left_stick_y);
        telemetry.addData("Slide current position", robot.slide.getCurrentPosition());
        telemetry.addData("Slide Target Position", robot.slide.getTargetPosition());
        telemetry.addData("Slide power", robot.slide.getPower());


        if (mecanumState == mecanumState.STOP) {
            if (controller2.right_stick_x > 0.5) {
                robot.track.setPosition(.25);
            } else if (controller2.right_stick_x < -0.5) {
                robot.track.setPosition(.75);
            } else {
                robot.setTrackDirection("stop");
            }
        }
        telemetry.addData("Controller2 RightStick X", controller2.right_stick_x);
//        if(controller2.BOnce()) {
//            startTime = System.currentTimeMillis();
//            mecanumState = mecanumState.INITIAL;
//        }
//        stateMachine();

        telemetry.addData("Slide Encoder", robot.slide.getCurrentPosition());
        telemetry.update();
    }




    public void openIntake() {
        intakeLeftPosition = 0.00;
        intakeRightPosition = 0.35;
    }

    public void closeIntake() {
        intakeLeftPosition = 0.65;
        intakeRightPosition = 0.00;
    }

    public void closeGrabber() {
        grabberPosition = 0.035;
    }

    public void openGrabber() {
        grabberPosition = 0.35;
    }

    public void setGrabberPosition() {
        robot.grabber.setPosition(grabberPosition);
        telemetry.addData("Grabber", grabberPosition);
    }



    //Notes:
//sequence:
//intakes opening0
//raise to 1st position
//close intakes
//open grabber
//run intakes out
//
//wait time in

//    public void stateMachine() {
//        //boolean event = false;
//
//        telemetry.addData("State", mecanumState.toString());
//
//        switch (mecanumState) {
//
//            case INITIAL:
//                mecanumState = MecanumState.OPEN_INTAKE;
//                openIntake();
//                setIntakePosition();
//                startTime = System.currentTimeMillis();
//                break;
//
//            case OPEN_INTAKE:
//
//                if (System.currentTimeMillis() > startTime + 500) {
//                    mecanumState = MecanumState.RAISE_ARM;
//                    robot.setArmPosition(1);
//                    robot.setArmPosition(2);
//                    startTime = System.currentTimeMillis();
//                }
//
//                break;
//            case RAISE_ARM:
//                if (System.currentTimeMillis() > startTime + 750) {
//                    mecanumState = MecanumState.CLOSE_INTAKE;
//                    closeIntake();
//                    setIntakePosition();
//                    startTime = System.currentTimeMillis();
//                }
//
//                break;
//            case CLOSE_INTAKE:
//                if (System.currentTimeMillis() > startTime + 500) {
//                    mecanumState = MecanumState.OPEN_GRABBER;
//                    openGrabber();
//                    setGrabberPosition();
//                    startTime = System.currentTimeMillis();
//                }
//
//                break;
//            case OPEN_GRABBER:
//                if (System.currentTimeMillis() > startTime + 500) {
//                    mecanumState = MecanumState.RUN_INTAKES_OUT;
//                    runIntakesOut();
//                    startTime = System.currentTimeMillis();
//                }
//
//                break;
//            case RUN_INTAKES_OUT:
//                if (System.currentTimeMillis() > startTime + 500) {
//                    robot.setArmPosition(0);
//                    mecanumState = MecanumState.LOWER_ARM;
//                }
//
//                break;
//            case LOWER_ARM:
//                if (System.currentTimeMillis() > startTime + 500) {
//                    mecanumState = MecanumState.STOP;
//                    startTime = System.currentTimeMillis();
//                }
//
//                break;
//            case STOP:
//                break;
//        }
//    }
}

