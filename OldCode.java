package org.firstinspires.ftc.teamcode;

public class OldCode {


    //stop

    //drive


    // Ensure that the opmode is still active
    // if (opModeIsActive()) {

    // Determine new target position, and pass to motor controller
//            newLfTarget = robot.lf.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
//            newRfTarget = robot.rf.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
//            newLrTarget = robot.lr.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
//            newRrTarget = robot.rr.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
//            robot.lf.setTargetPosition(newLfTarget);
//            robot.rf.setTargetPosition(newRfTarget);
//            robot.lr.setTargetPosition(newLrTarget);
//            robot.rr.setTargetPosition(newRrTarget);

    // Turn On RUN_TO_POSITION
//            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    // reset the timeout time and start motion.
    //   runtime.reset();
    // robot.lf.setPower(Math.abs(speed));
//            robot.rf.setPower(Math.abs(speed));
//            robot.lr.setPower(Math.abs(speed));
//            robot.rr.setPower(Math.abs(speed));

    // keep looping while we are still active, and there is time left, and both motors are running.
    // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
    // its target position, the motion will stop.  This is "safer" in the event that the robot will
    // always end the motion as soon as possible.
    // However, if you require that BOTH motors have finished their moves before the robot continues
    // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while(getRuntime()< timeoutS || robot.rr.isBusy()||robot.lr.isBusy()||robot.lf.isBusy()||robot.rf.isBusy()){
//                if(Math.abs(robot.lf.getCurrentPosition()-newLfTarget) > 0.5){
//                    robot.lf.setPower(0);
//                }
//                if(Math.abs(robot.lf.getCurrentPosition()-newLfTarget) > 0.5){
//                    robot.lf.setPower(0);
//                }
//                if(Math.abs(robot.lf.getCurrentPosition()-newLfTarget) > 0.5){
//                    robot.lf.setPower(0);
//                }
//                if(Math.abs(robot.lf.getCurrentPosition()-newLfTarget) > 0.5){
//                    robot.lf.setPower(0);
//                }
//            }

//            if() {
//                while (opModeIsActive() &&
//                        (robot.lf.getCurrentPosition() -  < newLfTarget || robot.rf.getCurrentPosition() < newRfTarget || robot.lr.getCurrentPosition() < newLrTarget || robot.rr.getCurrentPosition() < newRrTarget) &&
//                        (robot.lf.isBusy() && robot.rf.isBusy() && robot.lr.isBusy() && robot.rr.isBusy())) {
//
//                    // Display it for the driver.
////                    telemetry.addData("Path1", "Running to %7d :%7d", newLfTarget, newRfTarget, newLrTarget, newRrTarget);
////                    telemetry.addData("Path2", "Running at %7d :%7d",
////                            robot.lf.getCurrentPosition(),
////                            robot.rf.getCurrentPosition(),
////                            robot.lr.getCurrentPosition(),
////                            robot.rr.getCurrentPosition());
////                    telemetry.update();
//                }
//
    // Stop all motion;


    // Turn off RUN_TO_
    // POSITION

    //  sleep(250);   // optional pause after each move
    //  }
    //  }
    //turn
}// encoderDrive(TURN_SPEED,   3, -3, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//  encoderDrive(DRIVE_SPEED, 6, 6, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

// robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
// robot.rightClaw.setPosition(0.0);
//sleep(1000);     // pause for servos to move
//        robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
