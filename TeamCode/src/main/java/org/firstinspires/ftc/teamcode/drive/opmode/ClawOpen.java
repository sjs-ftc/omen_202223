package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This is a simple routine to test translational drive capabilities.
 */
@Disabled
@Config
@Autonomous(name = "ClawOpen", group = "drive")
public class ClawOpen extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    Servo rightClaw, leftClaw;
    public static double rightServoPosition = 0.0;
    public static double leftServoPosition = 1.0;


    @Override
    public void runOpMode() throws InterruptedException {

        rightClaw = hardwareMap.servo.get("rightClaw");
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw.setPosition(rightServoPosition);
        leftClaw.setPosition(leftServoPosition);

        waitForStart();
        runtime.reset();

        //rightServoPosition = 0.4;
        //leftServoPosition = 0.5;
        //rightClaw.setPosition(rightServoPosition);
        //leftClaw.setPosition(leftServoPosition);
        //sleep(2000);

        rightServoPosition = 0.0;
        leftServoPosition = 1.0;
        rightClaw.setPosition(rightServoPosition);
        leftClaw.setPosition(leftServoPosition);
        sleep(2000);

        rightServoPosition = 0.69;
        leftServoPosition = 0.25;
        rightClaw.setPosition(rightServoPosition);
        leftClaw.setPosition(leftServoPosition);
        sleep(2000);

        rightServoPosition = 0.0;
        leftServoPosition = 1.0;
        rightClaw.setPosition(rightServoPosition);
        leftClaw.setPosition(leftServoPosition);
        sleep(2000);

        rightServoPosition = 0.69;
        leftServoPosition = 0.25;
        rightClaw.setPosition(rightServoPosition);
        leftClaw.setPosition(leftServoPosition);
        sleep(2000);
    }
}