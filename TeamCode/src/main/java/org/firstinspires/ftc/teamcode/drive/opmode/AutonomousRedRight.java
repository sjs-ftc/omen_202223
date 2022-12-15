package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Angler;
import org.firstinspires.ftc.teamcode.drive.Claw;
import org.firstinspires.ftc.teamcode.drive.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class AutonomousRedRight extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousActions actions = new AutonomousActions(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Claw claw = new Claw(this, hardwareMap);
        Angler angler = new Angler(this, hardwareMap);
        Lift lift = new Lift(this, hardwareMap, telemetry);

        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(new Pose2d(36, 60, Math.PI/2))
                .back(48)
                .UNSTABLE_addTemporalMarkerOffset(-COLLECT_PAUSE,()-> {
                    angler.setAngle(STACK_5);
                    double startWait = getRuntime();
                    while (getRuntime() < startWait + COLLECT_PAUSE) {
                    }
                    claw.collect();
                })
                .turn(Math.toRadians(-90))
                .forward(24)
                .addTemporalMarker(() -> {
                    claw.closeClaw();
                    angler.slowAngle(SAFE_ANGLE);
                })
                .back(12)
                .waitSeconds(.1)
                .turn(Math.toRadians(30))
                .build();


        waitForStart();

        drive.followTrajectorySequence(trajectorySequence);

        actions.depositHigh(lift,angler,claw);
        angler.setAngle(STACK_4);
        double startWait = getRuntime();
        while (getRuntime() < startWait + COLLECT_PAUSE) {
        }
        claw.collect();

        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .turn(Math.toRadians(-30))
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
    }
}
