package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.Angler;
import org.firstinspires.ftc.teamcode.drive.Claw;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous()
public class AngleTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Angler angler = new Angler(this, hardwareMap);
        Claw claw = new Claw(this, hardwareMap);

        angler.setAngle(HORIZ_ANGLE);
        claw.collect();

        waitForStart();

        int i = 0;
        while (!isStopRequested()) {
            angler.setAngle(GROUND_ANGLE);

            //angler.setAngle(1.0);

            /*
                    angler.setAngle(180);
                    wait(1000);
                    angler.setAngle(45);
                    wait(1000);
                    angler.setAngle(90);
                    wait(1000);


             */
        }
    }
}