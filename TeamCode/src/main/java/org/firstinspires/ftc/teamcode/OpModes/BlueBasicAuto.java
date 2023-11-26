package org.firstinspires.ftc.teamcode.OpModes;
/*
 */

import static org.firstinspires.ftc.teamcode.OpModes.BlueAutoVariables.DEBUGposition;
import static org.firstinspires.ftc.teamcode.OpModes.BlueAutoVariables.pos1move1;
import static org.firstinspires.ftc.teamcode.OpModes.BlueAutoVariables.pos1move2;
import static org.firstinspires.ftc.teamcode.OpModes.BlueAutoVariables.pos1move3;
import static org.firstinspires.ftc.teamcode.OpModes.BlueAutoVariables.pos1move4;
import static org.firstinspires.ftc.teamcode.OpModes.BlueAutoVariables.pos1rot1;
import static org.firstinspires.ftc.teamcode.OpModes.BlueAutoVariables.state2heading1;
import static org.firstinspires.ftc.teamcode.OpModes.BlueAutoVariables.state2liftpos1;
import static org.firstinspires.ftc.teamcode.OpModes.BlueAutoVariables.state2move1;
import static org.firstinspires.ftc.teamcode.OpModes.BlueAutoVariables.state2move2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveClass;

@Autonomous(name = "Blue Autonomous", group = "Concept")

public class BlueBasicAuto extends LinearOpMode {

    FtcDashboard dashboard;
//    WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 1");

    /**
    VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(camera, visionProcessors);

    AprilTagProcessor myAprilTagProcessor;


    myAprilTagProcessor = new AprilTagProcessor.Builder\()
            .setTagLibrary(myAprilTagLibrary)
    .setDrawTagID(true)
    .setDrawTagOutline(true)
    .setDrawAxes(true)
    .setDrawCubeProjection(true)
    .build();
    private static final String[] LABELS = {
            "circle",
            "star",
            "triangle"
    };
     **/

    private final static HWProfile robot = new HWProfile();
    private LinearOpMode opMode = this;

    DriveClass drive = new DriveClass(robot, opMode);
    int position = 1;

    @Override

    public void runOpMode() {
        ElapsedTime elapsedTime = new ElapsedTime();

        /*
         * Setup the initial state of the robot
         */

//        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder().build();
//        aprilTag.setDecimation(2);
//        VisionPortal visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(aprilTag)
//                .build();
        State autoState = State.DETECT_POSITION;
        robot.init(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        /* Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        drive.closeClaw();

        // detect position of init pixel
        while(!opModeIsActive() && !isStopRequested()) {


        }  // end of while

        while(opModeIsActive()){

            switch (autoState) {
                case TEST:
                    autoState = State.DETECT_POSITION;
                    break;

                case DETECT_POSITION:
                    telemetry.addData("PLACE POSITION = ", "TBD");
                    telemetry.update();
                    if (DEBUGposition == 0) {

                    }
                    else {
                        position = DEBUGposition;
                    }
                    autoState = State.STATE_1;
                    break;

                case STATE_1:
                    telemetry.addData("STATE =  ", "1");
                    drive.closeClaw();
                    sleep(500);
                    drive.raiseClawToMid();
                    sleep(125);
                    if (position == 1) {
                        telemetry.addData("POSITION =  ", "LEFT");
                        drive.newDriveDistance(.5, 180, pos1move1);
                        sleep(200);
                        drive.newDriveDistance(.5, -90, pos1move2);
                        sleep(200);
                        drive.PIDRotate(pos1rot1,2);
                        sleep(10);
                        drive.newDriveDistance(.5, 180, pos1move3);
                        sleep(200);
                        drive.lowerClaw();
                        sleep(250);
                        drive.openClaw();
                        sleep(200);
                        drive.raiseClawToMid();
                        sleep(200);
                        drive.newDriveDistance(.5, 90, pos1move4);
                    } else if (position == 2) {
                        telemetry.addData("POSITION =  ", "CENTER");
                        drive.newDriveDistance(.5, 180, 21);
                        sleep(10);
                        drive.lowerClaw();
                        sleep(100);
                        drive.openClaw();
                        drive.newDriveDistance(.75, 0, 5);
                    } else if (position == 3) {
                        telemetry.addData("POSITION =  ", "RIGHT");
                        drive.newDriveDistance(.5, 180, 29);
                        sleep(100);
                        drive.newDriveDistance(.78, 90, 4);
                        sleep(200);
                        drive.PIDRotate(-90,2);
                        sleep(10);
                        drive.lowerClaw();
                        sleep(100);
                        drive.openClaw();
                    }
                    sleep(300);
                    drive.raiseClawToMid();
                    sleep(350);
                    drive.PIDRotate(90, 1);
                    autoState = State.STATE_2;

                    break;


                case STATE_2:
                    telemetry.addData("STATE =  ", "2");
                    drive.liftPos(state2liftpos1);
                    drive.newDriveDistance(.5, 0, state2move1);
                    drive.newDriveDistance(.5, state2heading1, state2move2);
                    sleep(400);
                    drive.bucketScore();
//                    drive.bucketScore();
                    sleep(1250);
                    drive.resetBucket();
                    sleep(150);
                    drive.liftPos(0);

//                    if(elapsedTime.time() > 1 && elapsedTime.time() < 1.5) {
//                        drive.openClaw();
//                    } else if(elapsedTime.time() > 1.75) {
//                        robot.launcherServo.setPosition(.4);
//                    }
                    autoState = State.STATE_3;
                    break;
                case STATE_3:
                    telemetry.addData("STATE =  ", "3");

                    autoState = State.HALT;
                    break;

                case PARK:
                    telemetry.addData("STATE =  ", "PARK");
                    autoState = State.HALT;

                    break;

                case HALT:
                    telemetry.addData("STATE =  ", "HALT");
                    // Stop all motors
                    sleep(1000);
                    drive.newMotorsHalt();

                    // End the program
                    requestOpModeStop();

                    break;
            }   // end of the switch state


        } // end of while(opModeIsActive())
        // End the program
        requestOpModeStop();

    }

    enum State {
        TEST, DETECT_POSITION, STATE_1, STATE_2, STATE_3, PARK, HALT
    }   // end of enum State

    private void initAprilTag() {

    }
}