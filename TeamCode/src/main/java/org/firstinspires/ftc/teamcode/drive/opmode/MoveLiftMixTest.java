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
public class MoveLiftMixTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousActions actions = new AutonomousActions(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Claw claw = new Claw(this, hardwareMap);
        Angler angler = new Angler(this, hardwareMap);
        Lift lift = new Lift(this, hardwareMap, telemetry);

        waitForStart();

        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(36, 60, Math.PI/2))
                .forward(2)
                .UNSTABLE_addTemporalMarkerOffset(-COLLECT_PAUSE+1,()-> {
                    angler.slowAngle(STACK_5);
                    double startWait = getRuntime();
                    while (getRuntime() < startWait + COLLECT_PAUSE) {
                    }
                    claw.collect();
                })
                .waitSeconds(5)
                .back(24)
                .build();


        drive.followTrajectorySequence(trajectory);

        trajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeRight(24)
                .addTemporalMarker(() -> {
                    claw.closeClaw();
                    angler.slowAngle(SAFE_ANGLE);
                })
                .addTemporalMarker(()-> {
                    actions.depositHigh(lift,angler,claw);
                    angler.setAngle(STACK_4);
                    double startWait = getRuntime();
                    while (getRuntime() < startWait + COLLECT_PAUSE) {
                    }
                    claw.collect();
                })
                .build();

        drive.followTrajectorySequence(trajectory);

    }
}
