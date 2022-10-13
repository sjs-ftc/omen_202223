package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TrajectoryTest extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(36,-60, Math.PI/2);

        drive.setPoseEstimate(startPose);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(24)
                .build();

        TrajectorySequence redLeft = drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.PI/2))
                .strafeLeft(24)
                .waitSeconds(1)
                .forward(48)
                .turn(Math.toRadians(90))
                .back(36)
                .waitSeconds(1)
                .forward(36)
                .waitSeconds(1)
                .back(36)
                .waitSeconds(1)
                .forward(12)
                .turn(Math.toRadians(90))
                .forward(24)
                .build();

        TrajectorySequence redRight = drive.trajectorySequenceBuilder(new Pose2d(36, -60, Math.PI/2))
                .strafeRight(24)
                .waitSeconds(1)
                .forward(48)
                .turn(Math.toRadians(-90))
                .back(36)
                .waitSeconds(1)
                .forward(36)
                .waitSeconds(1)
                .back(36)
                .waitSeconds(1)
                .forward(12)
                .turn(Math.toRadians(-90))
                .forward(24)
                .build();

        TrajectorySequence blueRight = drive.trajectorySequenceBuilder(new Pose2d(-36, 60, 3*Math.PI/2))
                .strafeRight(24)
                .waitSeconds(1)
                .forward(48)
                .turn(Math.toRadians(-90))
                .back(36)
                .waitSeconds(1)
                .forward(36)
                .waitSeconds(1)
                .back(36)
                .waitSeconds(1)
                .forward(12)
                .turn(Math.toRadians(-90))
                .forward(24)
                .build();

        TrajectorySequence blueLeft = drive.trajectorySequenceBuilder(new Pose2d(36, 60, 3*Math.PI/2))
                .strafeLeft(24)
                .waitSeconds(1)
                .forward(48)
                .turn(Math.toRadians(90))
                .back(36)
                .waitSeconds(1)
                .forward(36)
                .waitSeconds(1)
                .back(36)
                .waitSeconds(1)
                .forward(12)
                .turn(Math.toRadians(90))
                .forward(24)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(redLeft);

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
