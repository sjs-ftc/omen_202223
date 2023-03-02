package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Angler;
import org.firstinspires.ftc.teamcode.drive.Claw;
import org.firstinspires.ftc.teamcode.drive.Distances;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.atomic.AtomicReference;

@Disabled
@Autonomous(name = "Auto: Red Right No Camera", group = "Game No Camera")
public class FSMRedRightAuto extends LinearOpMode {

    public enum LiftState {
        LIFT_START,
        LIFT_HIGH1,
        LIFT_DROP1,
        LIFT_COLLECTWAIT1,
        LIFT_COLLECT1,
        LIFT_HIGH2,
        LIFT_DROP2,
        LIFT_COLLECTWAIT2,
        LIFT_COLLECT2,
        LIFT_HIGH3,
        LIFT_DROP3,
        LIFT_COLLECT3,
        LIFT_COLLECTWAIT3,
        LIFT_HIGH4,
        LIFT_DROP4,
        LIFT_COLLECT4,
        LIFT_COLLECTWAIT4,
        LIFT_HIGH5,
        LIFT_DROP5,
        LIFT_COLLECT5,
        LIFT_COLLECTWAIT5,
        LIFT_HIGH6,
        LIFT_DROP6,
        LIFT_END
    }

    Pose2d startPose = RRstartPose;
    Pose2d stackPose = RRstackPose;
    Pose2d dropPose = RRdropPose;
    Pose2d readyPose = RRreadyPose;
    Pose2d parkingPose1 = RRparkingPose1;
    Pose2d parkingPose2 = RRparkingPose2;
    Pose2d parkingPose3 = RRparkingPose3;
    Pose2d endPose = new Pose2d(10,-12,0);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Claw claw = new Claw(this, hardwareMap);
        Angler angler = new Angler(this, hardwareMap);
        Lift lift = new Lift(this, hardwareMap, telemetry);
        Distances distances = new Distances(this,hardwareMap);

        AtomicReference<LiftState> liftState = new AtomicReference<>(LiftState.LIFT_START);
        liftState.set(LiftState.LIFT_START);
        ElapsedTime liftTimer = new ElapsedTime();


        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(readyPose,
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(() -> {
                    liftState.set(LiftState.LIFT_HIGH1);
                })
                .lineToLinearHeading(dropPose)
                .build();

        angler.setAngle(SAFE_ANGLE);
        claw.closeClaw();

        waitForStart();

        drive.setPoseEstimate(startPose);

        boolean firstState = true;

        while (!isStopRequested()) {
            switch(liftState.get()){
                case LIFT_START: {
                    telemetry.addData("State","Start State");
                    telemetry.update();
                    if (firstState) {
                        drive.followTrajectorySequenceAsync(trajectorySequence);
                        firstState = false;
                    }
                    break;
                }
                case LIFT_HIGH1: {
                    angler.setAngle(HORIZ_ANGLE);
                    lift.setTargetHeight(HIGH_JUNCTION);
                    if (lift.getHeight() >= (HIGH_JUNCTION - TICK_ERROR) && lift.getHeight() <= (HIGH_JUNCTION + TICK_ERROR)) {
                        claw.dropCone();
                        liftState.set(LiftState.LIFT_DROP1);
                        liftTimer.reset();
                        firstState = true;
                    }
                    break;
                }
                case LIFT_DROP1: {
                    angler.setAngle(STACK_5);
                    if (liftTimer.seconds() >= DROP_PAUSE) {
                        claw.closeClaw();
                        lift.setTargetHeight(MINIMUM_HEIGHT);
                        liftState.set(LiftState.LIFT_COLLECTWAIT1);
                    }
                    break;
                }
                case LIFT_COLLECTWAIT1: {
                    if (lift.getHeight() < MID_JUNCTION && firstState) {
                        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(stackPose,
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .waitSeconds(COLLECT_PAUSE)
                                .build();

                        drive.followTrajectorySequenceAsync(trajectorySequence);
                        firstState = false;
                    }
                    else if (lift.getHeight() < MID_JUNCTION && lift.getHeight() > LOW_JUNCTION) {
                        claw.collect();
                    }
                    if (lift.getHeight() < MINIMUM_HEIGHT + TICK_ERROR) {
                        liftState.set(LiftState.LIFT_COLLECT1);
                        liftTimer.reset();
                        firstState = true;
                    }
                    if (distances.getDistances()[0] <= 8 || distances.getDistances()[1] <= 8) {
                        drive.setPoseEstimate(stackPose);
                    }
                    break;
                }
                case LIFT_COLLECT1: {
                    claw.closeClaw();
                    if (firstState && liftTimer.seconds() >= DROP_PAUSE) {
                            trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .waitSeconds(.3)
                                    .addTemporalMarker(() -> {
                                        liftState.set(LiftState.LIFT_HIGH2);
                                    })
                                    .lineToLinearHeading(dropPose,
                                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                    .build();

                            drive.followTrajectorySequenceAsync(trajectorySequence);
                            firstState = false;
                        }
                    break;
                }
                case LIFT_HIGH2: {
                    angler.setAngle(HORIZ_ANGLE);
                    lift.setTargetHeight(HIGH_JUNCTION);
                    if (lift.getHeight() >= (HIGH_JUNCTION - TICK_ERROR) && lift.getHeight() <= (HIGH_JUNCTION + TICK_ERROR)) {
                        claw.dropCone();
                        liftState.set(LiftState.LIFT_DROP2);
                        liftTimer.reset();
                        firstState = true;
                    }
                    break;
                }
                case LIFT_DROP2: {
                    angler.setAngle(STACK_4);
                    if (liftTimer.seconds() >= DROP_PAUSE) {
                        claw.closeClaw();
                        lift.setTargetHeight(MINIMUM_HEIGHT);
                        liftState.set(LiftState.LIFT_COLLECTWAIT2);
                    }
                    break;
                }
                case LIFT_COLLECTWAIT2: {
                    if (lift.getHeight() < MID_JUNCTION && firstState) {
                        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(stackPose,
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .waitSeconds(COLLECT_PAUSE)
                                .build();

                        drive.followTrajectorySequenceAsync(trajectorySequence);
                        firstState = false;
                    }
                    else if (lift.getHeight() < MID_JUNCTION && lift.getHeight() > LOW_JUNCTION) {
                        claw.collect();
                    }
                    if (lift.getHeight() < MINIMUM_HEIGHT + TICK_ERROR) {
                        liftState.set(LiftState.LIFT_COLLECT2);
                        liftTimer.reset();
                        firstState = true;
                    }
                    if (distances.getDistances()[0] <= 8 || distances.getDistances()[1] <= 8) {
                        drive.setPoseEstimate(stackPose);
                    }
                    break;
                }
                case LIFT_COLLECT2: {
                    if (firstState && liftTimer.seconds() >= DROP_PAUSE) {
                        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .waitSeconds(.3)
                                .addTemporalMarker(() -> {
                                    liftState.set(LiftState.LIFT_HIGH3);
                                })
                                .lineToLinearHeading(dropPose,
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();

                        claw.closeClaw();
                        drive.followTrajectorySequenceAsync(trajectorySequence);
                        firstState = false;
                    }
                    break;
                }
                case LIFT_HIGH3: {
                    angler.setAngle(HORIZ_ANGLE);
                    lift.setTargetHeight(HIGH_JUNCTION);
                    if (lift.getHeight() >= (HIGH_JUNCTION - TICK_ERROR) && lift.getHeight() <= (HIGH_JUNCTION + TICK_ERROR)) {
                        claw.dropCone();
                        liftState.set(LiftState.LIFT_DROP3);
                        liftTimer.reset();
                        firstState = true;
                    }
                    break;
                }
                case LIFT_DROP3: {
                    angler.setAngle(STACK_3);
                    if (liftTimer.seconds() >= DROP_PAUSE) {
                        claw.closeClaw();
                        lift.setTargetHeight(MINIMUM_HEIGHT);
                        liftState.set(LiftState.LIFT_COLLECTWAIT3);
                    }
                    break;
                }
                case LIFT_COLLECTWAIT3: {
                    if (lift.getHeight() < MID_JUNCTION && firstState) {
                        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(stackPose,
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .waitSeconds(COLLECT_PAUSE)
                                .build();

                        drive.followTrajectorySequenceAsync(trajectorySequence);
                        firstState = false;
                    }
                    else if (lift.getHeight() < MID_JUNCTION && lift.getHeight() > LOW_JUNCTION) {
                        claw.collect();
                    }
                    if (lift.getHeight() < MINIMUM_HEIGHT + TICK_ERROR) {
                        liftState.set(LiftState.LIFT_COLLECT3);
                        liftTimer.reset();
                        firstState = true;
                    }
                    if (distances.getDistances()[0] <= 8 || distances.getDistances()[1] <= 8) {
                        drive.setPoseEstimate(stackPose);
                    }
                    break;
                }
                case LIFT_COLLECT3: {
                    if (firstState && liftTimer.seconds() >= DROP_PAUSE) {
                        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .waitSeconds(.3)
                                .addTemporalMarker(() -> {
                                    liftState.set(LiftState.LIFT_HIGH4);
                                })
                                .lineToLinearHeading(dropPose,
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();

                        claw.closeClaw();
                        drive.followTrajectorySequenceAsync(trajectorySequence);
                        firstState = false;
                    }
                    break;
                }
                case LIFT_HIGH4: {
                    angler.setAngle(HORIZ_ANGLE);
                    lift.setTargetHeight(HIGH_JUNCTION);
                    if (lift.getHeight() >= (HIGH_JUNCTION - TICK_ERROR) && lift.getHeight() <= (HIGH_JUNCTION + TICK_ERROR)) {
                        claw.dropCone();
                        liftState.set(LiftState.LIFT_DROP4);
                        liftTimer.reset();
                        firstState = true;
                    }
                    break;
                }
                case LIFT_DROP4: {
                    angler.setAngle(STACK_2);
                    if (liftTimer.seconds() >= DROP_PAUSE) {
                        claw.closeClaw();
                        lift.setTargetHeight(MINIMUM_HEIGHT);
                        liftState.set(LiftState.LIFT_END);
                    }
                    break;
                }
                /*
                case LIFT_COLLECTWAIT4: {
                    if (lift.getHeight() < MID_JUNCTION && firstState) {
                        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(stackPose,
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .waitSeconds(COLLECT_PAUSE)
                                .build();

                        drive.followTrajectorySequenceAsync(trajectorySequence);
                        firstState = false;
                    }
                    else if (lift.getHeight() < MID_JUNCTION && lift.getHeight() > LOW_JUNCTION) {
                        claw.collect();
                    }
                    if (lift.getHeight() < MINIMUM_HEIGHT + TICK_ERROR) {
                        liftState.set(LiftState.LIFT_COLLECT4);
                        liftTimer.reset();
                        firstState = true;
                    }
                    if (distances.getDistances()[0] <= 8 || distances.getDistances()[1] <= 8) {
                        drive.setPoseEstimate(stackPose);
                    }
                    break;
                }
                case LIFT_COLLECT4: {
                    if (firstState && liftTimer.seconds() >= DROP_PAUSE) {
                        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .waitSeconds(.3)
                                .addTemporalMarker(() -> {
                                    liftState.set(LiftState.LIFT_HIGH5);
                                })
                                .lineToLinearHeading(dropPose,
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();

                        claw.closeClaw();
                        drive.followTrajectorySequenceAsync(trajectorySequence);
                        firstState = false;
                    }
                    break;
                }
                case LIFT_HIGH5: {
                    angler.setAngle(HORIZ_ANGLE);
                    lift.setTargetHeight(HIGH_JUNCTION);
                    if (lift.getHeight() >= (HIGH_JUNCTION - TICK_ERROR) && lift.getHeight() <= (HIGH_JUNCTION + TICK_ERROR)) {
                        claw.dropCone();
                        liftState.set(LiftState.LIFT_DROP5);
                        liftTimer.reset();
                        firstState = true;
                    }
                    break;
                }
                case LIFT_DROP5: {
                    angler.setAngle(GROUND_ANGLE);
                    if (liftTimer.seconds() >= DROP_PAUSE) {
                        claw.closeClaw();
                        lift.setTargetHeight(MINIMUM_HEIGHT);
                        liftState.set(LiftState.LIFT_COLLECTWAIT5);
                    }
                    break;
                }
                case LIFT_COLLECTWAIT5: {
                    if (lift.getHeight() < MID_JUNCTION && firstState) {
                        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(stackPose,
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .waitSeconds(COLLECT_PAUSE)
                                .build();

                        drive.followTrajectorySequenceAsync(trajectorySequence);
                        firstState = false;
                    }
                    else if (lift.getHeight() < MID_JUNCTION && lift.getHeight() > LOW_JUNCTION) {
                        claw.collect();
                    }
                    if (lift.getHeight() < MINIMUM_HEIGHT + TICK_ERROR) {
                        liftState.set(LiftState.LIFT_COLLECT5);
                        liftTimer.reset();
                        firstState = true;
                    }
                    if (distances.getDistances()[0] <= 8 || distances.getDistances()[1] <= 8) {
                        drive.setPoseEstimate(stackPose);
                    }
                    break;
                }
                case LIFT_COLLECT5: {
                    if (firstState && liftTimer.seconds() >= DROP_PAUSE) {
                        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .waitSeconds(.3)
                                .addTemporalMarker(() -> {
                                    liftState.set(LiftState.LIFT_HIGH6);
                                })
                                .lineToLinearHeading(dropPose,
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();

                        claw.closeClaw();
                        drive.followTrajectorySequenceAsync(trajectorySequence);
                        firstState = false;
                    }
                    break;
                }
                case LIFT_HIGH6: {
                    angler.setAngle(HORIZ_ANGLE);
                    lift.setTargetHeight(HIGH_JUNCTION);
                    if (lift.getHeight() >= (HIGH_JUNCTION - TICK_ERROR) && lift.getHeight() <= (HIGH_JUNCTION + TICK_ERROR)) {
                        claw.dropCone();
                        liftState.set(LiftState.LIFT_DROP6);
                        liftTimer.reset();
                        firstState = true;
                    }
                    break;
                }
                case LIFT_DROP6: {
                    angler.setAngle(GROUND_ANGLE);
                    if (liftTimer.seconds() >= DROP_PAUSE) {
                        claw.closeClaw();
                        lift.setTargetHeight(MINIMUM_HEIGHT);
                        liftState.set(LiftState.LIFT_END);
                    }
                    break;
                }
                 */
                case LIFT_END: {
                    trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(endPose,
                                    SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build();

                    angler.setAngle(GROUND_ANGLE);
                    claw.closeClaw();
                    drive.followTrajectorySequenceAsync(trajectorySequence);
                    break;
                }
            }
            lift.update();
            drive.update();
        }
    }
}
