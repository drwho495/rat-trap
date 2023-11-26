package com.example.meepmeeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(760);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.7, 60, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(13,35,Math.toRadians(270)))
                                .turn(Math.toRadians(-90))
                                // open claw i love jesus christ
                                .lineToLinearHeading(new Pose2d(12,60,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(50,42,Math.toRadians(0)))
                                // Score i love jesus christ
                                .lineToLinearHeading(new Pose2d(40,35,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(35,53,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(11,60,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-40,60,Math.toRadians(0)))
//                                // Start intaking i love god
                                .lineToLinearHeading(new Pose2d(-60,35,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-40,60,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(35,60,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(47,42,Math.toRadians(0)))
                                // Score i love jesus christ
                                .lineToLinearHeading(new Pose2d(47,63,Math.toRadians(0)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}


/*   Left
                             .lineToLinearHeading(new Pose2d(10,35,Math.toRadians(270)))
                             .turn(Math.toRadians(90))
                                        // open claw i love jesus christ
                                        .lineToLinearHeading(new Pose2d(10,60,Math.toRadians(0)))
                                        .lineToLinearHeading(new Pose2d(50,42,Math.toRadians(0)))
                                        // Score i love jesus christ
                                        .lineToLinearHeading(new Pose2d(40,35,Math.toRadians(0)))
                                        .lineToLinearHeading(new Pose2d(35,53,Math.toRadians(0)))
                                        .lineToLinearHeading(new Pose2d(11,60,Math.toRadians(0)))
                                        .lineToLinearHeading(new Pose2d(-40,60,Math.toRadians(0)))
                                        // Start intaking OwO
                                        .lineToLinearHeading(new Pose2d(-60,35,Math.toRadians(0)))
                                        .lineToLinearHeading(new Pose2d(-40,60,Math.toRadians(0)))
                                        .lineToLinearHeading(new Pose2d(35,60,Math.toRadians(0)))
                                        .lineToLinearHeading(new Pose2d(47,42,Math.toRadians(0)))
                                        // Score i love jesus christ
                                        .lineToLinearHeading(new Pose2d(47,63,Math.toRadians(0)))
                                        .build()

   Center
                                   // open claw i love jesus christ
                                .lineToLinearHeading(new Pose2d(10,60,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(50,42,Math.toRadians(0)))
                                // Score i love jesus christ
                                .lineToLinearHeading(new Pose2d(40,35,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(35,53,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(11,60,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-40,60,Math.toRadians(0)))
                                // Start intaking OwO
                                .lineToLinearHeading(new Pose2d(-60,35,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-40,60,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(35,60,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(47,42,Math.toRadians(0)))
                                // Score i love jesus christ
                                .lineToLinearHeading(new Pose2d(47,63,Math.toRadians(0)))
                                .build()

   Right
      .lineToLinearHeading(new Pose2d(13,35,Math.toRadians(270)))
                                .turn(Math.toRadians(-90))
                                // open claw i love jesus christ
                                .lineToLinearHeading(new Pose2d(12,60,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(50,42,Math.toRadians(0)))
                                // Score i love jesus christ
                                .lineToLinearHeading(new Pose2d(40,35,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(35,53,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(11,60,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-40,60,Math.toRadians(0)))
                                // Start intaking OwO
                                .lineToLinearHeading(new Pose2d(-60,35,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-40,60,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(35,60,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(47,42,Math.toRadians(0)))
                                // Score i love jesus christ
                                .lineToLinearHeading(new Pose2d(47,63,Math.toRadians(0)))
                                .build()
 */

