package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Angler;
import org.firstinspires.ftc.teamcode.drive.AutonomousActions;
import org.firstinspires.ftc.teamcode.drive.Claw;
import org.firstinspires.ftc.teamcode.drive.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous
public class AutonomousRedRight extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousActions actions = new AutonomousActions(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Claw claw = new Claw(this, hardwareMap);
        Angler angler = new Angler(this, hardwareMap);
        Lift lift = new Lift(this, hardwareMap, telemetry);

        Pose2d startPose = new Pose2d(34, -64, 3*Math.PI/2);

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(34,-14,0))
                .addTemporalMarker(-COLLECT_PAUSE,()-> {
                    angler.setAngle(STACK_5);
                    claw.collect();
                })
                .lineToLinearHeading(new Pose2d(60,-13,0))
                .addTemporalMarker(() -> {
                    claw.closeClaw();
                    angler.slowAngle(SAFE_ANGLE);
                })
                .waitSeconds(.3)
                .lineToLinearHeading(new Pose2d(51,-13,Math.toRadians(-20)))
                .build();


        waitForStart();

        drive.followTrajectorySequence(trajectorySequence);

        actions.depositHigh(lift,angler,claw);
        angler.setAngle(STACK_4);
        double startWait = getRuntime();
        while (opModeIsActive() && getRuntime() < startWait + COLLECT_PAUSE) {
        }
        claw.collect();

        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(60,-13,0))
                .addTemporalMarker(() -> {
                    claw.closeClaw();
                    angler.slowAngle(SAFE_ANGLE);
                })
                .waitSeconds(.3)
                .lineToLinearHeading(new Pose2d(51,-13,Math.toRadians(-20)))
                .build();

        drive.followTrajectorySequence(trajectorySequence);

        actions.depositHigh(lift,angler,claw);
        angler.setAngle(STACK_3);
        startWait = getRuntime();
        while (opModeIsActive() && getRuntime() < startWait + COLLECT_PAUSE) {
        }
        claw.collect();

        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(10,-12,0))
                .build();

        drive.followTrajectorySequence(trajectorySequence);

        angler.setAngle(SAFE_ANGLE);
        claw.closeClaw();

        /*
        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .turn(Math.toRadians(30))
                .forward(12)
                .addTemporalMarker(() -> {
                    claw.closeClaw();
                    angler.slowAngle(SAFE_ANGLE);
                })
                .waitSeconds(2)
                .back(12)
                .turn(Math.toRadians(30))
                .build();

        drive.followTrajectorySequence(trajectorySequence);

        actions.depositHigh(lift,angler,claw);
        angler.setAngle(STACK_3);
        startWait = getRuntime();
        while (getRuntime() < startWait + COLLECT_PAUSE) {
        }
        claw.collect();

        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .turn(Math.toRadians(-30))
                .build();

        drive.followTrajectorySequence(trajectorySequence);

         */
    }
}
