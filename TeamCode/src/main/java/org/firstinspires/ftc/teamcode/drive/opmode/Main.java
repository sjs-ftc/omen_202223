package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.Angler;
import org.firstinspires.ftc.teamcode.drive.Claw;
import org.firstinspires.ftc.teamcode.drive.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;

@Disabled
@TeleOp(group = "drive")
public class Main extends LinearOpMode {

    private void dropCone(Lift lift, Angler angler, Claw claw) {
        claw.dropCone();
        double startWait = getRuntime();
        while (getRuntime() < startWait + PAUSE_TIME) {
        }
        claw.closeClaw();
        angler.slowAngle(SAFE_ANGLE);
        lift.goToHeight(MINIMUM_HEIGHT);
        while (opModeIsActive() && !(lift.getHeight() >= (MINIMUM_HEIGHT - TICK_ERROR) && lift.getHeight() <= (MINIMUM_HEIGHT + TICK_ERROR))) {
            lift.goToHeight(MINIMUM_HEIGHT);
        }
        claw.closeClaw();
    }

    public void moveArm(Lift lift, Angler angler, Claw claw) {
        int height;
        if (gamepad2.y) {
            claw.closeClaw();
            angler.slowAngle(SAFE_ANGLE);
            lift.goToHeight(HIGH_JUNCTION);
            while (opModeIsActive() && !(lift.getHeight() >= (HIGH_JUNCTION - TICK_ERROR) && lift.getHeight() <= (HIGH_JUNCTION + TICK_ERROR))) {
                lift.goToHeight(HIGH_JUNCTION);
            }
            angler.slowAngle(HORIZ_ANGLE);
            return;

        }
        if (gamepad2.x) {
            claw.closeClaw();
            angler.slowAngle(SAFE_ANGLE);
            lift.goToHeight(LOW_JUNCTION);
            while (opModeIsActive() && !(lift.getHeight() >= (LOW_JUNCTION - TICK_ERROR) && lift.getHeight() <= (LOW_JUNCTION + TICK_ERROR))) {
                lift.goToHeight(LOW_JUNCTION);
            }
            angler.slowAngle(HORIZ_ANGLE);
            return;
        }
        if (gamepad2.b) {
            claw.closeClaw();
            angler.slowAngle(SAFE_ANGLE);
            lift.goToHeight(MID_JUNCTION);
            while (opModeIsActive() && !(lift.getHeight() >= (MID_JUNCTION - TICK_ERROR) && lift.getHeight() <= (MID_JUNCTION + TICK_ERROR))) {
                lift.goToHeight(MID_JUNCTION);
            }
            angler.slowAngle(HORIZ_ANGLE);
            return;
        }
        if (gamepad2.a) {
            claw.closeClaw();
            angler.slowAngle(SAFE_ANGLE);
            lift.goToHeight(GROUND_JUNCTION);
            while (opModeIsActive() && !(lift.getHeight() >= (GROUND_JUNCTION - TICK_ERROR) && lift.getHeight() <= (GROUND_JUNCTION + TICK_ERROR))) {
                lift.goToHeight(GROUND_JUNCTION);
            }
            angler.slowAngle(GROUND_ANGLE);
            return;
        }

        if (gamepad2.dpad_down) {
            dropCone(lift, angler, claw);
        }

        if (gamepad2.dpad_left) {
            angler.setAngle(GROUND_ANGLE);
            double startWait = getRuntime();
            while (getRuntime() < startWait + COLLECT_PAUSE) {
            }
            claw.collect();
        }

        if (gamepad2.dpad_up) {
            claw.closeClaw();
            angler.slowAngle(SAFE_ANGLE);
        }

    }

    private void drive(SampleMecanumDrive drive, Gamepad gamepad) {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -Math.pow(gamepad.left_stick_y,3) * DRIVE_POWER,
                        -Math.pow(gamepad.left_stick_x,3) * DRIVE_POWER,
                        -Math.pow(gamepad.right_stick_x,3) * DRIVE_POWER
                )
        );
    }

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Claw claw = new Claw(this, hardwareMap);
        Angler angler = new Angler(this, hardwareMap);
        Lift lift = new Lift(this, hardwareMap, telemetry);

        angler.setAngle(SAFE_ANGLE);

        waitForStart();

        while (!isStopRequested()) {
            moveArm(lift, angler, claw);
            drive(drive, gamepad1);


            drive.update();
        }

        angler.setAngle(.90);
    }
}
