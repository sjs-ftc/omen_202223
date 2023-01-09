package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Angler;
import org.firstinspires.ftc.teamcode.drive.Claw;
import org.firstinspires.ftc.teamcode.drive.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;


@TeleOp(group = "drive")
public class FSMMain extends LinearOpMode {

    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_RETRACT,
        LIFT_COLLECT
    }
    
    private void drive(SampleMecanumDrive drive, Gamepad gamepad1) {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -Math.pow(gamepad1.left_stick_y,3) * DRIVE_POWER,
                        -Math.pow(gamepad1.left_stick_x,3) * DRIVE_POWER,
                        -Math.pow(gamepad1.right_stick_x,3) * DRIVE_POWER
                )
        );
    }

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Claw claw = new Claw(this, hardwareMap);
        Angler angler = new Angler(this, hardwareMap);
        Lift lift = new Lift(this, hardwareMap, telemetry);
        LiftState liftState = LiftState.LIFT_COLLECT;
        ElapsedTime liftTimer = new ElapsedTime();

        angler.setAngle(SAFE_ANGLE);

        waitForStart();

        boolean firstState = true;
        boolean clawOpen = false;
        liftTimer.reset();

        while (!isStopRequested()) {
            switch (liftState) {
                case LIFT_START: {
                    if (gamepad1.y) {
                        lift.setTargetHeight(HIGH_JUNCTION);
                        claw.closeClaw();
                        angler.setAngle(HORIZ_ANGLE);
                        liftState = LiftState.LIFT_EXTEND;
                    }
                    else if (gamepad1.x) {
                        lift.setTargetHeight(MID_JUNCTION);
                        claw.closeClaw();
                        angler.setAngle(HORIZ_ANGLE);
                        liftState = LiftState.LIFT_EXTEND;
                    }
                    else if (gamepad1.b) {
                        lift.setTargetHeight(LOW_JUNCTION);
                        claw.closeClaw();
                        angler.setAngle(HORIZ_ANGLE);
                        liftState = LiftState.LIFT_EXTEND;
                    }
                    else if (gamepad1.a) {
                        claw.collect();
                        liftState = LiftState.LIFT_COLLECT;
                    }
                    else if (gamepad1.dpad_left) {
                        liftState = LiftState.LIFT_COLLECT;
                        liftTimer.reset();
                        firstState = true;
                        clawOpen = false;
                    }
                    break;
                }
                case LIFT_EXTEND: {
                    if (lift.stalling) {
                        liftState = LiftState.LIFT_DUMP;
                    }
                    break;
                }
                case LIFT_DUMP: {
                    if (gamepad1.dpad_down) {
                        claw.dropCone();
                        liftTimer.reset();
                        liftState = LiftState.LIFT_RETRACT;
                        firstState = true;
                    }
                    break;
                }
                case LIFT_RETRACT: {
                    if (firstState && liftTimer.seconds() >= DROP_PAUSE) {
                        claw.closeClaw();
                        lift.setTargetHeight(MINIMUM_HEIGHT);
                        liftTimer.reset();
                        firstState = false;
                    }
                    else if (liftTimer.seconds() >= DROP_PAUSE) {
                        angler.setAngle(GROUND_ANGLE);
                    }
                    if (lift.getHeight() >= (MINIMUM_HEIGHT - TICK_ERROR) && lift.getHeight() <= (MINIMUM_HEIGHT + TICK_ERROR)) {
                        liftState = LiftState.LIFT_COLLECT;
                        liftTimer.reset();
                        firstState = true;
                        clawOpen = false;
                    }
                    break;
                    }
                case LIFT_COLLECT: {
                    if (gamepad1.dpad_up) {
                        claw.closeClaw();
                        angler.setAngle(SAFE_ANGLE);
                        liftState = LiftState.LIFT_START;
                    }
                    if (firstState) {
                        angler.setAngle(GROUND_ANGLE);
                        liftTimer.reset();
                        firstState = false;
                    }
                    if (!clawOpen && liftTimer.seconds() >= 1.5*COLLECT_PAUSE) {
                        claw.collect();
                        firstState = false;
                        clawOpen = true;
                    }
                    break;
                }
            }

            drive(drive, gamepad1);

            lift.update();
            drive.update();
        }

        angler.setAngle(.90);
    }
}
