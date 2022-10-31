package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.Claw;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive")
public class Main extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Claw claw = new Claw();
        Servo rightClaw, leftClaw;
        rightClaw = hardwareMap.servo.get("rightClaw");
        leftClaw = hardwareMap.servo.get("leftClaw");

        boolean clawPos = false;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            if (gamepad1.a) {
                Log.d("Claw State", "A pressed: ");
                if (clawPos) {
                   claw.closeClaw(rightClaw,leftClaw);
                   clawPos = false;
                }
                else {
                    claw.openClaw(rightClaw,leftClaw);
                    clawPos = true;
                }
            }
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                            //Math.pow(Math.E,4*(-gamepad1.left_stick_y+1)),
                            //Math.pow(Math.E,4*(-gamepad1.left_stick_x+1)),
                            //Math.pow(Math.E,4*(-gamepad1.right_stick_x+1))
                    )
            );

            drive.update();
        }
    }
}
