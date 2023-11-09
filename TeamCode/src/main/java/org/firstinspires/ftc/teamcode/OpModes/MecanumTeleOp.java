package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveClass;

@TeleOp(name = "Teleop Mode", group = "Competition")

public class MecanumTeleOp extends LinearOpMode {
    private final static HWProfile robot = new HWProfile();

    @Override
    public void runOpMode(){
        double v1, v2, v3, v4, robotAngle;
        double theta;
        double theta2 = 180;
        double r;
        double power=1;
        double rightX, rightY;
        double TargetRotation = 0;
        double OldRotation = 0;
        int liftPos = 0;
        boolean rotateEnabled = false;
        boolean fieldCentric = true;
        int targetPosition = 0;
        LinearOpMode opMode = this;
        boolean passthroughMode = false;
        double liftPower = robot.LIFT_POWER_DOWN;
        ElapsedTime elapsedTime = new ElapsedTime();
        int clawAxisPos = 0;
        boolean bCooldown = false;
        double RFrotatePower = robot.TURN_SPEED;
        double LFrotatePower = -robot.TURN_SPEED;
        double LRrotatePower = -robot.TURN_SPEED;
        double RRrotatePower = robot.TURN_SPEED;


        robot.init(hardwareMap);

        DriveClass drive = new DriveClass(robot, opMode);

        telemetry.addData("Ready to Run: ","GOOD LUCK");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            /*******************************************
             ****** Mecanum Drive Control section ******
             *******************************************/
            if (fieldCentric) {             // verify that the user hasn't disabled field centric drive
                theta = robot.imu.getAbsoluteHeading() + 90;
//                        robot.imu.getAngularOrientation().firstAngle + 90;
            } else {
                theta = 0;      // do not adjust for the angular position of the robot
            }

            robotAngle = Math.atan2(gamepad1.left_stick_y, (-gamepad1.left_stick_x)) - Math.PI / 4;
            rightX = -gamepad1.right_stick_x;
            rightY = -gamepad1.right_stick_y;
            r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);

            v1 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY);
            v2 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY);
            v3 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY);
            v4 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY);

            if(((robot.imu.getAbsoluteHeading() - OldRotation >= TargetRotation && TargetRotation > 0) || (robot.imu.getAbsoluteHeading() - OldRotation <= TargetRotation && TargetRotation < 0)) && rotateEnabled) {
                rotateEnabled = false;
                TargetRotation = 0;
                OldRotation = 0;
            }

            if(!rotateEnabled) {
                robot.motorLF.setPower(com.qualcomm.robotcore.util.Range.clip((v1), -power, power));
                robot.motorRF.setPower(com.qualcomm.robotcore.util.Range.clip((v2), -power, power));
                robot.motorLR.setPower(com.qualcomm.robotcore.util.Range.clip((v3), -power, power));
                robot.motorRR.setPower(com.qualcomm.robotcore.util.Range.clip((v4), -power, power));
            } else {
                robot.motorLF.setPower(LFrotatePower);
                robot.motorRF.setPower(RFrotatePower);
                robot.motorLR.setPower(LRrotatePower);
                robot.motorRR.setPower(RRrotatePower);
            }

            // Control which direction is forward and which is backward from the driver POV
 /*           if (gamepad1.y && (currentTime.time() - buttonPress) > 0.3) {
                if (theta2 == 180) {
                    theta2 = 0;
                } else {
                    theta2 = 180;
                }
                buttonPress = currentTime.time();
            }   // end if (gamepad1.x && ...)
*/

            if (gamepad1.dpad_up){
                targetPosition = targetPosition + 20;
                liftPower = 1;
            } else if (gamepad1.dpad_down) {
                targetPosition = targetPosition - 20;
                liftPower = 1;
            }

            /* Limit the range of the lift so as not to damage the robot */
            targetPosition = Range.clip(targetPosition, robot.LIFT_RESET, robot.LIFT_MAX_HEIGHT);

            drive.liftPosition(targetPosition, liftPower);

            /* Claw Control */
            if(gamepad1.right_bumper || gamepad2.right_bumper) {
//                elapsedTime.reset();
                drive.openClaw();
            } else if (gamepad1.left_bumper || gamepad2.left_bumper){
                drive.closeClaw();
            }

            if(gamepad1.right_trigger > .1) {
                liftPos += 500;
            } else if (gamepad1.left_trigger > .1) {
                liftPos -= 500;
            }

            if(gamepad1.y) {
                robot.bucketAxisServo.setPosition(.5);
            } else if(gamepad1.a) {
                robot.bucketAxisServo.setPosition(0.01);
            }

            if(gamepad1.b && bCooldown == false) {
                if(clawAxisPos == 0) {
                    robot.launcherServo.setPosition(.75);
                    clawAxisPos = 1;
                } else {
                    robot.launcherServo.setPosition(.4);
                    clawAxisPos = 0;
                }

                bCooldown = true;
            }

            if(!gamepad2.b && bCooldown == true) {
                bCooldown = false;
            }

            if(gamepad1.x && passthroughMode == false) {
                elapsedTime.reset();
                passthroughMode = true;
                drive.closeClaw();
                sleep(200);
                robot.launcherServo.setPosition(.05);
            }

            if(passthroughMode) {
                if(elapsedTime.time() > 1.75 && elapsedTime.time() < 2) {
                    drive.openClaw();
                } else if(elapsedTime.time() > 2) {
                    robot.launcherServo.setPosition(.4);
                    passthroughMode = false;
                }
            }

            if (gamepad1.dpad_up){
                targetPosition += 100;
                liftPower = 1;
            } else if (gamepad1.dpad_down) {
                targetPosition -= 100;
                liftPower = 1;
            } else if (gamepad1.dpad_left) {
                liftPos = 0;
                drive.resetBucket();
            } else if (gamepad1.dpad_right) {
                liftPos = 5000;
            }

//            if(robot.sensorCone.getDistance(DistanceUnit.CM) <= robot.CONE_DISTANCE) {
//                if(elapsedTime.time() >= robot.CONE_WAIT_TIME) {
//                    drive.closeClaw();
//                }
//            }

            liftPos = Range.clip(liftPos, 0, 5250);
            robot.slidesMotor.setTargetPosition((int) liftPos);
            robot.slidesMotor.setPower(1);

            // 90 degree turn

            // Provide user feedback
//            telemetry.addData("V1 = ", v1);
//            telemetry.addData("elapsed time = ", elapsedTime.time());
//            telemetry.addData("V2 = ", v2);
//            telemetry.addData("V3 = ", v3);
//            telemetry.addData("V4 = ", v4);
//            telemetry.addData("Motor Left Lift = ", robot.motorLeftLift.getCurrentPosition());
//            telemetry.addData("Motor Right Lift = ", robot.motorRightLift.getCurrentPosition());
//            telemetry.addData("Theta = ", theta);
//            telemetry.addData("Theta2 = ", theta);
//            telemetry.addData("IMU Value: ", theta);
//            telemetry.addData("robot rotation: ", OldRotation);
//            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of MSTeleop class