package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.Angler;
import org.firstinspires.ftc.teamcode.drive.Claw;
import org.firstinspires.ftc.teamcode.drive.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive")
public class Main extends LinearOpMode {

    public static double drivePower = 5.0/10.0; //power to motors

    private final int START = 150;
    private Gamepad lastGamepad1 = gamepad1;
    private Gamepad lastGamepad2 = gamepad2;
    private boolean clawPos = false;

        public void moveArm(Lift lift, Angler angler) {

            int height;
            if (gamepad1.y && !lastGamepad1.y) {
                height = 650;
                lift.goToHeight(height);
                angler.setAngle(.25);
            }
            else if (gamepad1.x && !lastGamepad1.x) {
                height = 340;
                lift.goToHeight(height);
                angler.setAngle(.25);
            }
            else if (gamepad1.b && !lastGamepad1.b) {
                height = 600;
                lift.goToHeight(height);
                angler.setAngle(.25);
            }
            else if (gamepad1.a && !lastGamepad1.a) {
                height = START;
                lift.goToHeight(height);
                angler.setAngle(.25);
            }
        }

        public void manageClaw(Claw claw) throws InterruptedException {
            if (gamepad1.options && !lastGamepad1.options) {
                if (clawPos) {
                    claw.closeClaw();
                    clawPos = false;
                }
                else {
                    claw.openClaw();
                    clawPos = true;
                }
                Log.d("Claw State", "A pressed: ");
                claw.closeClaw();
                clawPos = false;
            }
        }

        public void drive(SampleMecanumDrive drive) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -Math.pow(gamepad1.left_stick_y,3) * drivePower,
                            -Math.pow(gamepad1.left_stick_x,3) * drivePower,
                            -Math.pow(gamepad1.right_stick_x,3) * drivePower
                    )
            );
        }


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Claw claw = new Claw(hardwareMap);
        Angler angler = new Angler(hardwareMap);
        Lift lift = new Lift(hardwareMap, telemetry);

        waitForStart();

        while (!isStopRequested()) {
            manageClaw(claw);
            moveArm(lift, angler);
            drive(drive);

            lastGamepad1 = gamepad1;
            drive.update();
        }
    }
}
