package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

public class MeepMeepMain {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(90)))
                        .splineToConstantHeading(new Vector2d(36, -24), Math.toRadians(90))

                        .splineToConstantHeading(new Vector2d(36, -12), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(45, -12), Math.toRadians(-90))
                        .lineToConstantHeading(new Vector2d(45, -48))

                        .splineToConstantHeading(new Vector2d(45, -12), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(53, -12), Math.toRadians(-90))
                        .lineTo(new Vector2d(53, -48))

                        .splineToConstantHeading(new Vector2d(53, -12), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(61, -12), Math.toRadians(-90))
                        .lineTo(new Vector2d(61, -45))

//                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(61, -55, Math.toRadians(90)))
                        .splineToConstantHeading(new Vector2d(10,-34), Math.toRadians(90))
                        .lineTo(new Vector2d(10, -32))
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(38,-55), Math.toRadians(-90))
                        .lineTo(new Vector2d(38, -61))
                        .setReversed(false)
                        .lineTo(new Vector2d(38, -55))

                        .splineToConstantHeading(new Vector2d(7,-34), Math.toRadians(90))
                        .lineTo(new Vector2d(7, -32))
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(38,-55), Math.toRadians(-90))
                        .lineTo(new Vector2d(38, -61))
                        .setReversed(false)
                        .lineTo(new Vector2d(38, -55))

                        .splineToConstantHeading(new Vector2d(4,-34), Math.toRadians(90))
                        .lineTo(new Vector2d(4, -32))
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(38,-55), Math.toRadians(-90))
                        .lineTo(new Vector2d(38, -61))
                        .setReversed(false)
                        .lineTo(new Vector2d(38, -55))

                        .splineToConstantHeading(new Vector2d(1,-34), Math.toRadians(90))
                        .lineTo(new Vector2d(1, -32))
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(38,-55), Math.toRadians(-90))
                        .lineTo(new Vector2d(38, -61))
                        .setReversed(false)

                        .splineToConstantHeading(new Vector2d(5, -32), Math.toRadians(90))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}