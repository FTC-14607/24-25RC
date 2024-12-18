package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;

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
//                .addEntity(myBot2)
                .start();
    }

    static RoadRunnerBotEntity getSpecimenPath(MeepMeep meepMeep) {
        MarkerCallback driveMode = () -> {};
        MarkerCallback prepareScoreSpecimen = () -> {}; // robot.startPrepareSpecimenDeposit()
        MarkerCallback releaseScoredSpecimen = () -> {}; // robot.openUpperClaw();
        MarkerCallback preparePickupSpecimen = () -> {}; // robot.startPrepareSpecimenPickup()
        MarkerCallback pickupSpecimen = () -> {}; // robot.pickupSpecimen();
        MarkerCallback prepareAscent = () -> {}; // robot.prepareAscent1();

        Pose2d startPose = new Pose2d(12, -60, Math.toRadians(90));

        Pose2d trussPose =     new Pose2d(10, -32, Math.toRadians(90));
        Pose2d trussInterval = new Pose2d(3, 0, 0); // dist between scored specimens
        Pose2d trussBackup =   new Pose2d(0, 2, 0); // dist in front of truss to go straight for

        Pose2d pickupPose =   new Pose2d(38, -61, Math.toRadians(-90));
        Pose2d pickupBackup = new Pose2d(0, 6, 0); // dist in front of wall to go straight for

        Pose2d parkIntermediate = new Pose2d(-38, -30, Math.toRadians(90));
        Pose2d parkPose = new Pose2d(-26, -12, 0);

        return new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPose)

                // score first pre-loaded specimen
                .splineToConstantHeading(trussPose.minus(trussBackup).vec(), trussPose.getHeading())
                .addTemporalMarker(0, prepareScoreSpecimen)
                .lineTo(trussPose.vec())
                .addDisplacementMarker(releaseScoredSpecimen)
                .setReversed(true)

                // move three alliance samples to human player
                .addTemporalMarker(2, driveMode)
                .splineToConstantHeading(new Vector2d(36, -28), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(36, -12), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(45, -12), Math.toRadians(-90))
                .lineTo(new Vector2d(45, -48))

                .splineToConstantHeading(new Vector2d(45, -12), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(53, -12), Math.toRadians(-90))
                .lineTo(new Vector2d(53, -48))

                .splineToConstantHeading(new Vector2d(53, -12), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(61, -12), Math.toRadians(-90))
                .lineTo(new Vector2d(61, -51))

                // score three specimens
//                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(61, -55, Math.toRadians(90)))
                .setReversed(false)
                .splineToConstantHeading(pickupPose.plus(pickupBackup).vec(), pickupPose.getHeading())
                        .addTemporalMarker(10.4, preparePickupSpecimen)
                .lineTo(pickupPose.vec())
                        .addDisplacementMarker(pickupSpecimen)
                .lineTo(pickupPose.plus(pickupBackup).vec())

                .splineToConstantHeading(trussPose.minus(trussBackup).minus(trussInterval.times(1)).vec(), trussPose.getHeading())
                .addTemporalMarker(12.5, prepareScoreSpecimen)
                .lineTo(trussPose.minus(trussInterval.times(1)).vec())
                .addDisplacementMarker(releaseScoredSpecimen)
                .setReversed(true)
                .splineToConstantHeading(pickupPose.plus(pickupBackup).vec(), pickupPose.getHeading())
                .lineTo(pickupPose.vec())
                .setReversed(false)
                .lineTo(pickupPose.plus(pickupBackup).vec())

                .splineToConstantHeading(trussPose.minus(trussBackup).minus(trussInterval.times(2)).vec(), trussPose.getHeading())
                .addTemporalMarker(14.8, prepareScoreSpecimen)
                .lineTo(trussPose.minus(trussInterval.times(2)).vec())
                .addDisplacementMarker(releaseScoredSpecimen)
                .setReversed(true)
                .splineToConstantHeading(pickupPose.plus(pickupBackup).vec(), pickupPose.getHeading())
                .lineTo(pickupPose.vec())
                .setReversed(false)
                .lineTo(pickupPose.plus(pickupBackup).vec())

                .splineToConstantHeading(trussPose.minus(trussBackup).minus(trussInterval.times(3)).vec(), trussPose.getHeading())
                .addTemporalMarker(19, prepareScoreSpecimen)
                .lineTo(trussPose.minus(trussInterval.times(3)).vec())
                .addDisplacementMarker(releaseScoredSpecimen)
                .setReversed(true)

                // move to truss for Ascent Level 1
                .splineToConstantHeading(parkIntermediate.vec(), parkIntermediate.getHeading())
                .addDisplacementMarker(prepareAscent)
                .splineTo(parkPose.vec(), parkPose.getHeading())

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