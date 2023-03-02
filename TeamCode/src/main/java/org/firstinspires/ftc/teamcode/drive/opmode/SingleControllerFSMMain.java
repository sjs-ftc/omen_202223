package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Angler;
import org.firstinspires.ftc.teamcode.drive.Claw;
import org.firstinspires.ftc.teamcode.drive.Distances;
import org.firstinspires.ftc.teamcode.drive.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;


@TeleOp(name="SingleControllerFSMMain", group = "a")
public class SingleControllerFSMMain extends LinearOpMode {

    double drivePower = DRIVE_POWER/4;

    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_RETRACT,
        LIFT_COLLECT
    }

    private void drive(SampleMecanumDrive drive, Gamepad gamepad) {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -Math.pow(gamepad.left_stick_y,1) * drivePower,
                        -Math.pow(gamepad.left_stick_x,1) * drivePower,
                        -Math.pow(gamepad.right_stick_x,1) * drivePower
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
        ElapsedTime buttonTimer = new ElapsedTime();
        ElapsedTime buttonTimer2 = new ElapsedTime();
        ElapsedTime buttonTimer3 = new ElapsedTime();


        int clawAngle = 0;
        angler.setAngle(GROUND_ANGLE);

        waitForStart();

        boolean firstState = true;
        boolean clawOpen = false;
        liftTimer.reset();
        buttonTimer.reset();
        buttonTimer2.reset();
        buttonTimer3.reset();


        while (!isStopRequested()) {
            switch (liftState) {
                case LIFT_START: {
                    if (gamepad1.y) {
                        lift.setTargetHeight(HIGH_JUNCTION);
                        claw.closeClaw();
                        angler.setAngle(HORIZ_ANGLE);
                        liftState = LiftState.LIFT_EXTEND;
                        buttonTimer3.reset();
                    }
                    else if (gamepad1.x) {
                        lift.setTargetHeight(MID_JUNCTION);
                        claw.closeClaw();
                        angler.setAngle(HORIZ_ANGLE);
                        liftState = LiftState.LIFT_EXTEND;
                        buttonTimer3.reset();

                    }
                    else if (gamepad1.b) {
                        lift.setTargetHeight(LOW_JUNCTION);
                        claw.closeClaw();
                        angler.setAngle(HORIZ_ANGLE);
                        liftState = LiftState.LIFT_EXTEND;
                        buttonTimer3.reset();

                    }
                    else if (gamepad1.a) {
                        lift.setTargetHeight(MINIMUM_HEIGHT);
                        claw.closeClaw();
                        angler.setAngle(GROUND_ANGLE);
                        liftState = LiftState.LIFT_EXTEND;
                        buttonTimer3.reset();

                    }
                    else if (gamepad1.dpad_down) {
                        lift.setTargetHeight(MINIMUM_HEIGHT);
                        liftState = LiftState.LIFT_COLLECT;
                        buttonTimer3.reset();
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
                    if (buttonTimer3.seconds() > BUTTON_PAUSE && (gamepad1.y || gamepad1.x || gamepad1.b || gamepad1.a || gamepad1.dpad_down)) {
                        liftState = LiftState.LIFT_START;
                        buttonTimer3.reset();
                    }
                    break;
                }
                case LIFT_DUMP: {
                    if (gamepad1.right_bumper) {
                        claw.dropCone();
                        liftTimer.reset();
                        liftState = LiftState.LIFT_RETRACT;
                        firstState = true;
                    }
                    if (buttonTimer3.seconds() > BUTTON_PAUSE && (gamepad2.y || gamepad2.x || gamepad2.b || gamepad2.a || gamepad2.dpad_down)) {
                        liftState = LiftState.LIFT_START;
                        buttonTimer3.reset();
                    }
                    break;
                }
                case LIFT_RETRACT: {
                    if (firstState && liftTimer.seconds() >= DROP_PAUSE) {
                        claw.closeClaw();
                        angler.setAngle(GROUND_ANGLE);
                        liftTimer.reset();
                        firstState = false;
                    }
                    else if (liftTimer.seconds() >= DROP_PAUSE) {
                        lift.setTargetHeight(MINIMUM_HEIGHT);
                    }
                    if (lift.getHeight() <= MID_JUNCTION) {
                        claw.collect();
                        clawOpen = false;
                    }
                    if (lift.getHeight() >= (MINIMUM_HEIGHT - TICK_ERROR) && lift.getHeight() <= (MINIMUM_HEIGHT + TICK_ERROR)) {
                        liftState = LiftState.LIFT_COLLECT;
                        liftTimer.reset();
                        firstState = true;
                    }
                    break;
                }
                case LIFT_COLLECT: {
                    if (gamepad1.dpad_up) {
                        claw.closeClaw();
                        angler.setAngle(SAFE_ANGLE);
                        liftState = LiftState.LIFT_START;
                        break;
                    }
                    if (gamepad1.left_bumper && buttonTimer.seconds() >= BUTTON_PAUSE) {
                        clawAngle++;
                        buttonTimer.reset();
                    }
                    if (gamepad1.dpad_down && buttonTimer.seconds() >= BUTTON_PAUSE) {
                        clawAngle = 0;
                        buttonTimer.reset();
                    }
                    switch (clawAngle) {
                        case 0: {
                            angler.setAngle(GROUND_ANGLE);
                            break;
                        }
                        case 1: {
                            angler.setAngle(STACK_2);
                            break;
                        }
                        case 2: {
                            angler.setAngle(STACK_3);
                            break;
                        }
                        case 3: {
                            angler.setAngle(STACK_4);
                            break;
                        }
                        case 4: {
                            angler.setAngle(STACK_5);
                            break;
                        }
                        case 5: {
                            clawAngle = 0;
                            break;
                        }
                    }
                    if (firstState) {
                        angler.setAngle(GROUND_ANGLE);
                        liftTimer.reset();
                        firstState = false;
                    }
                    if (!clawOpen) {
                        claw.collect();
                        firstState = false;
                        clawOpen = true;
                    }
                    break;
                }
            }

            if (gamepad1.left_trigger > .5) {
                drivePower = DRIVE_POWER/3;
            }
            else {
                drivePower = DRIVE_POWER;
            }
            drive(drive, gamepad1);

            lift.update();
            drive.update();

            if (gamepad1.dpad_down) {
                lift.setTargetHeight(lift.getHeight()-40);
            }
        }

        angler.setAngle(GROUND_ANGLE);
    }
}
