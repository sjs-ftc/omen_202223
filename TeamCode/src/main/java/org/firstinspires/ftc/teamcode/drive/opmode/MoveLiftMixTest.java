package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Angler;
import org.firstinspires.ftc.teamcode.drive.AutonomousActions;
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

        Pose2d startPose = new Pose2d(36, 60, Math.PI/2);

        drive.setPoseEstimate(startPose);

        waitForStart();

        Trajectory trajectory = drive.trajectoryBuilder(startPose)
                .forward(24)
                .build();


        angler.slowAngle(SAFE_ANGLE);

        drive.followTrajectory(trajectory);

        actions.depositHigh(lift,angler,claw);



    }
}
