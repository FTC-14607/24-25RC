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

        RoadRunnerBotEntity myBot = getSpecimenPath(meepMeep);
        RoadRunnerBotEntity myBot2 = getSamplePath(meepMeep);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(myBot2)
                .start();
    }

    static RoadRunnerBotEntity getSpecimenPath(MeepMeep meepMeep) {
        return new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(90)))

                // score first pre-loaded specimen
                .splineToConstantHeading(new Vector2d(10, -34), Math.toRadians(90))
                .lineTo(new Vector2d(10, -32))
                .setReversed(true)

                // move three alliance samples to human player
                .splineToConstantHeading(new Vector2d(36, -28), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(36, -12), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(45, -12), Math.toRadians(-90))
                .lineTo(new Vector2d(45, -48))

                .splineToConstantHeading(new Vector2d(45, -12), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(53, -12), Math.toRadians(-90))
                .lineTo(new Vector2d(53, -48))

                .splineToConstantHeading(new Vector2d(53, -12), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(61, -12), Math.toRadians(-90))
                .lineTo(new Vector2d(61, -45))

                // score three specimens
//                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(61, -55, Math.toRadians(90)))
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

                // move to truss for Ascent Level 1
                .splineToConstantHeading(new Vector2d(-38, -30), Math.toRadians(90))
                .splineTo(new Vector2d(-26,-12), Math.toRadians(0))

                .build());
    }

    static RoadRunnerBotEntity getSamplePath(MeepMeep meepMeep) {
        Pose2d scoreSamplePose = new Pose2d(-53, -53, Math.toRadians(225));
        return new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(-90)))

                // score first pre-loaded sample
                .setReversed(true)
                .splineToLinearHeading(scoreSamplePose, Math.toRadians(225))

                // move three alliance samples to human player
                .splineToLinearHeading(new Pose2d(-49, -45, Math.toRadians(-90)), Math.toRadians(90))
                .setReversed(false)
                .splineToLinearHeading(scoreSamplePose, Math.toRadians(225))

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-57, -45, Math.toRadians(-90)), Math.toRadians(90))
                .setReversed(false)
                .splineToLinearHeading(scoreSamplePose, Math.toRadians(225))


                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-57, -25.5, Math.toRadians(0)), Math.toRadians(180))
                .setReversed(false)
                .splineToLinearHeading(scoreSamplePose, Math.toRadians(225))

                // park for Ascent Level 1
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-26, 12, Math.toRadians(0)), 0)

                .build());
    }
}