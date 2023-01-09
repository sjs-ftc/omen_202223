package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        Pose2d startPose = new Pose2d(-36, -64, 3*Math.PI/2);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToLinearHeading(new Pose2d(-36,-12,Math.PI))
                                .lineToLinearHeading(new Pose2d(-51,-12,Math.toRadians(180+20)))
                                .waitSeconds(.5)
                                .lineToLinearHeading(new Pose2d(-60,-12,Math.PI))
                                .waitSeconds(.5)
                                .lineToLinearHeading(new Pose2d(-51,-12,Math.toRadians(180+20)))
                                .waitSeconds(.5)
                                .lineToLinearHeading(new Pose2d(-60,-12,Math.PI))
                                .waitSeconds(.5)
                                .lineToLinearHeading(new Pose2d(-51,-12,Math.toRadians(180+20)))
                                .waitSeconds(.5)
                                .lineToLinearHeading(new Pose2d(-60,-12,Math.PI)).waitSeconds(.5)
                                .lineToLinearHeading(new Pose2d(-51,-12,Math.toRadians(180+20)))
                                .waitSeconds(.5)
                                .lineToLinearHeading(new Pose2d(-60,-12,Math.PI))
                                .waitSeconds(.5)
                                .lineToLinearHeading(new Pose2d(-51,-12,Math.toRadians(180+20)))
                                .waitSeconds(.5)
                                .lineToLinearHeading(new Pose2d(-60,-12,Math.PI))
                                .waitSeconds(.5)
                                .lineToLinearHeading(new Pose2d(-51,-12,Math.toRadians(180+20)))
                                .waitSeconds(.5)
                                .lineToLinearHeading(new Pose2d(-12,-12,Math.PI))


                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}