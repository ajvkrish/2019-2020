package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Mecanum teleop (with an optional arcade mode)
 * * Left stick controls x/y translation.
 * * Right stick controls rotation about the z axis
 * * When arcade mode is enabled (press "a"), translation direction
 * becomes relative to the field as opposed to the robot. You can
 * reset the forward heading by pressing "x".
 */
@TeleOp(name = "Motor Test")
public class MotorTest extends OpMode {
    private Robot robot;
    private Controller controller;
    private int motorNum=0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.runUsingEncoders();
        controller = new Controller(gamepad1);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {
        controller.update();

        if (controller.XOnce()) {
            motorNum++;
            if (motorNum > 3)
                motorNum = 0;
        }
        telemetry.addData("Selected Motor: Motor ", motorNum);
        switch(motorNum)
        {
            case 0:
                robot.setMotors(controller.left_stick_y/20,0,0,0);
                telemetry.addData("Position lf: ",robot.lf.getCurrentPosition());
                break;
            case 1:
                robot.setMotors(0, controller.left_stick_y/20,0,0);
                telemetry.addData("Position lr: ",robot.lr.getCurrentPosition());
                break;
            case 2:
                robot.setMotors(0,0, controller.left_stick_y/20,0);
                telemetry.addData("Position rf: ",robot.rf.getCurrentPosition());
                break;
            case 3:
                robot.setMotors(0,0,0, controller.left_stick_y/20);
                telemetry.addData("Position rr: ",robot.rr.getCurrentPosition());
                break;
            default:
                telemetry.addData("Position: ", "none");
                robot.setMotors(0,0,0,0);
        }
        telemetry.update();
    }
}
