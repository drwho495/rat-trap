package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
//@Autonomous(name = "Blue Autonomous", group = "Concept")

@Config
public class BlueAutoVariables {
    public static double pos1move1 = 27;
    public static int DEBUGposition = 27;
    public static double pos1move2 = 3;

    public static double pos1rot1 = -90;
    public static double pos1move3 = 3;
    public static double pos1move4 = 14;
    public static double state2move1 = 17;
    public static double state2heading1 = -35;
    public static double state2move2 = 17;
    public static int state2liftpos1 = 14;

}
