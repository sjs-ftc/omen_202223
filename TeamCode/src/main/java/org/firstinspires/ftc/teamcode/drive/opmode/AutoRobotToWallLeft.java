package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Angler;
import org.firstinspires.ftc.teamcode.drive.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.Claw;
import org.firstinspires.ftc.teamcode.drive.ClawSwitch;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name = "High Left", group = "a")
public class AutoRobotToWallLeft extends LinearOpMode {

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

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = TAG_SIZE;

    int left = 7;
    int middle = 8;
    int right = 9;

    AprilTagDetection tagOfInterest = null;


    Pose2d startPose = HLstartPose;
    Pose2d stackReadyPose = HLstackReadyPose;
    Pose2d dropPose = HLdropPose;
    Pose2d cone5Pose = HLcone5Pose;
    Pose2d readyPose = HLreadyPose;
    Pose2d parkingPose1 = HLparkingPose1;
    Pose2d parkingPose2 = HLparkingPose2;
    Pose2d parkingPose3 = HLparkingPose3;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1920 ,1080, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });
        telemetry.setMsTransmissionInterval(50);

        telemetry.addData("Init","Hello");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Claw claw = new Claw(this, hardwareMap);
        Angler angler = new Angler(this, hardwareMap);
        Lift lift = new Lift(this, hardwareMap, telemetry);
        ClawSwitch clawSwitch = new ClawSwitch(this,hardwareMap);


        AtomicBoolean firstState = new AtomicBoolean(true);
        AtomicReference<LiftState> liftState = new AtomicReference<>(LiftState.LIFT_START);
        liftState.set(LiftState.LIFT_START);
        ElapsedTime liftTimer = new ElapsedTime();





        int signalNumber = 3;
        telemetry.addData("Init","Before Loop");
        telemetry.update();

        claw.closeClaw();
        while (!isStarted() && !isStopRequested()) {
            angler.setAngle(START_ANGLE);

            telemetry.addData("Command","April Tags Running");
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                telemetry.addData("number of objects ",currentDetections.size());
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == left || tag.id == middle || tag.id == right)
                    {
                        tagOfInterest = tag;
                        if (tagOfInterest.id == left) {
                            telemetry.addData("Tag: ","1");
                            signalNumber = 1;
                        }
                        else if (tagOfInterest.id == middle) {
                            signalNumber = 2;
                            telemetry.addData("Tag: ","2");
                        }
                        else if (tagOfInterest.id == right) {
                            signalNumber = 3;
                            telemetry.addData("Tag: ","3");
                        }
                        tagFound = true;
                        break;
                    }
                }
            }
            telemetry.update();
            sleep(20);
        }
        if (tagOfInterest == null) {
            signalNumber = 3;
        }

        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    angler.setAngle(SAFE_ANGLE);
                })
                .lineToLinearHeading(readyPose,
                        SampleMecanumDrive.getVelocityConstraint(TRAJECTORY_VELOCITY, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(() -> {
                    liftState.set(LiftState.LIFT_HIGH1);
                    firstState.set(true);
                })
                .lineToLinearHeading(dropPose)
                .build();

        drive.setPoseEstimate(startPose);


        while (!isStopRequested()) {
            switch(liftState.get()){
                case LIFT_START: {
                    telemetry.addData("State","Start State");
                    telemetry.update();
                    if (firstState.get()) {
                        drive.followTrajectorySequenceAsync(trajectorySequence);
                        firstState.set(false);
                    }
                    break;
                }
                case LIFT_HIGH1: {
                    angler.setAngle(HORIZ_ANGLE);
                    lift.setTargetHeight(HIGH_JUNCTION);
                    if (lift.stalling && firstState.get()) {
                        liftTimer.reset();
                        firstState.set(false);
                    }
                    else if (lift.stalling && liftTimer.seconds() > DROP_PAUSE) {
                        claw.dropCone();
                        liftState.set(LiftState.LIFT_DROP1);
                        liftTimer.reset();
                        firstState.set(true);
                    }
                    break;
                }
                case LIFT_DROP1: {
                    angler.setAngle(STACK_5);
                    if (liftTimer.seconds() >= DROP_PAUSE) {
                        claw.closeClaw();
                        lift.setTargetHeight(MINIMUM_HEIGHT);
                        liftState.set(LiftState.LIFT_COLLECTWAIT1);
                        liftTimer.reset();
                    }
                    break;
                }
                case LIFT_COLLECTWAIT1: {
                    if (lift.getHeight() < MID_JUNCTION && firstState.get()) {
                        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(HLstackPose1,
                                        SampleMecanumDrive.getVelocityConstraint(TRAJECTORY_VELOCITY, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addTemporalMarker(() -> {
                                    liftTimer.reset();
                                    liftState.set(LiftState.LIFT_COLLECT1);
                                    firstState.set(true);
                                })
                                .build();

                        drive.followTrajectorySequenceAsync(trajectorySequence);
                        liftTimer.reset();
                        firstState.set(false);
                        claw.autoCollect();
                    }
                    if (clawSwitch.isPressed()) {
                        liftTimer.reset();
                        liftState.set(LiftState.LIFT_COLLECT1);
                        firstState.set(true);
                        drive.setPoseEstimate(HLstackPose1);
                    }
                    break;
                }
                case LIFT_COLLECT1: {
                    angler.setAngle(HORIZ_ANGLE);
                    claw.closeClaw();
                    if (firstState.get() && liftTimer.seconds() >= COLLECT_PAUSE) {
                        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addTemporalMarker(() -> {
                                    lift.setTargetHeight(HIGH_JUNCTION);
                                    liftState.set(LiftState.LIFT_HIGH2);
                                    firstState.set(true);
                                    liftTimer.reset();
                                })
                                .lineToLinearHeading(dropPose,
                                        SampleMecanumDrive.getVelocityConstraint(TRAJECTORY_VELOCITY, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();


                        drive.followTrajectorySequenceAsync(trajectorySequence);
                        firstState.set(false);
                    }
                    break;
                }
                case LIFT_HIGH2: {
                    angler.setAngle(HORIZ_ANGLE);
                    lift.setTargetHeight(HIGH_JUNCTION);
                    if (lift.stalling && firstState.get()) {
                        liftTimer.reset();
                        firstState.set(false);
                    }
                    else if (lift.stalling && liftTimer.seconds() > PAUSE_TIME) {
                        claw.dropCone();
                        liftState.set(LiftState.LIFT_DROP2);
                        liftTimer.reset();
                        firstState.set(true);
                    }
                    break;
                }
                case LIFT_DROP2: {
                    angler.setAngle(STACK_4);
                    if (liftTimer.seconds() >= DROP_PAUSE) {
                        claw.closeClaw();
                        lift.setTargetHeight(MINIMUM_HEIGHT);
                        liftState.set(LiftState.LIFT_COLLECTWAIT2);
                        liftTimer.reset();
                    }
                    break;
                }
                case LIFT_COLLECTWAIT2: {
                    if (lift.getHeight() < MID_JUNCTION && firstState.get()) {
                        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(HLstackPose2,
                                        SampleMecanumDrive.getVelocityConstraint(TRAJECTORY_VELOCITY, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addTemporalMarker(() -> {
                                    liftTimer.reset();
                                    liftState.set(LiftState.LIFT_COLLECT2);
                                    firstState.set(true);
                                })
                                .build();

                        drive.followTrajectorySequenceAsync(trajectorySequence);
                        liftTimer.reset();
                        firstState.set(false);
                        claw.autoCollect();
                    }
                    if (clawSwitch.isPressed()) {
                        liftTimer.reset();
                        liftState.set(LiftState.LIFT_COLLECT2);
                        firstState.set(true);
                        drive.setPoseEstimate(HLstackPose2);

                    }
                    break;
                }
                case LIFT_COLLECT2: {
                    claw.closeClaw();
                    angler.setAngle(HORIZ_ANGLE);
                    if (firstState.get() && liftTimer.seconds() >= COLLECT_PAUSE) {
                        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addTemporalMarker(() -> {
                                    lift.setTargetHeight(HIGH_JUNCTION);
                                    liftState.set(LiftState.LIFT_HIGH3);
                                    firstState.set(true);
                                    liftTimer.reset();
                                })
                                .lineToLinearHeading(dropPose,
                                        SampleMecanumDrive.getVelocityConstraint(TRAJECTORY_VELOCITY, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();


                        drive.followTrajectorySequenceAsync(trajectorySequence);
                        firstState.set(false);
                    }
                    break;
                }
                case LIFT_HIGH3: {
                    angler.setAngle(HORIZ_ANGLE);
                    lift.setTargetHeight(HIGH_JUNCTION);
                    if (lift.stalling && firstState.get()) {
                        liftTimer.reset();
                        firstState.set(false);
                    }
                    else if (lift.stalling && liftTimer.seconds() > PAUSE_TIME) {
                        claw.dropCone();
                        liftState.set(LiftState.LIFT_DROP3);
                        liftTimer.reset();
                        firstState.set(true);
                    }
                    break;
                }
                case LIFT_DROP3: {
                    angler.setAngle(STACK_3);
                    if (liftTimer.seconds() >= DROP_PAUSE) {
                        claw.closeClaw();
                        lift.setTargetHeight(MINIMUM_HEIGHT);
                        liftState.set(LiftState.LIFT_COLLECTWAIT3);
                        liftTimer.reset();
                    }
                    break;
                }
                case LIFT_COLLECTWAIT3: {
                    if (lift.getHeight() < MID_JUNCTION && firstState.get()) {
                        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(HLstackPose3,
                                        SampleMecanumDrive.getVelocityConstraint(TRAJECTORY_VELOCITY, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addTemporalMarker(() -> {
                                    liftTimer.reset();
                                    liftState.set(LiftState.LIFT_COLLECT3);
                                    firstState.set(true);
                                })
                                .build();

                        drive.followTrajectorySequenceAsync(trajectorySequence);
                        liftTimer.reset();
                        firstState.set(false);
                        claw.autoCollect();
                    }
                    if (clawSwitch.isPressed()) {
                        liftTimer.reset();
                        liftState.set(LiftState.LIFT_COLLECT3);
                        firstState.set(true);
                        drive.setPoseEstimate(HLstackPose3);

                    }
                    break;
                }
                case LIFT_COLLECT3: {
                    claw.closeClaw();
                    angler.setAngle(HORIZ_ANGLE);
                    if (firstState.get() && liftTimer.seconds() >= COLLECT_PAUSE) {
                        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addTemporalMarker(() -> {
                                    lift.setTargetHeight(HIGH_JUNCTION);
                                    liftState.set(LiftState.LIFT_HIGH4);
                                    firstState.set(true);
                                    liftTimer.reset();
                                })
                                .lineToLinearHeading(dropPose,
                                        SampleMecanumDrive.getVelocityConstraint(TRAJECTORY_VELOCITY, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();


                        drive.followTrajectorySequenceAsync(trajectorySequence);
                        firstState.set(false);
                    }
                    break;
                }
                case LIFT_HIGH4: {
                    angler.setAngle(HORIZ_ANGLE);
                    lift.setTargetHeight(HIGH_JUNCTION);
                    if (lift.stalling && firstState.get()) {
                        liftTimer.reset();
                        firstState.set(false);
                    }
                    else if (lift.stalling && liftTimer.seconds() > PAUSE_TIME) {
                        claw.dropCone();
                        liftState.set(LiftState.LIFT_DROP4);
                        liftTimer.reset();
                        firstState.set(true);
                    }
                    break;
                }
                case LIFT_DROP4: {
                    angler.setAngle(STACK_2);
                    if (liftTimer.seconds() >= DROP_PAUSE) {
                        claw.closeClaw();
                        lift.setTargetHeight(MINIMUM_HEIGHT);
                        liftState.set(LiftState.LIFT_COLLECTWAIT4);
                        liftTimer.reset();
                    }
                    break;
                }
                case LIFT_COLLECTWAIT4: {
                    if (lift.getHeight() < MID_JUNCTION && firstState.get()) {
                        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(HLstackPose4,
                                        SampleMecanumDrive.getVelocityConstraint(TRAJECTORY_VELOCITY, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addTemporalMarker(() -> {
                                    liftTimer.reset();
                                    liftState.set(LiftState.LIFT_COLLECT4);
                                    firstState.set(true);
                                })
                                .build();

                        drive.followTrajectorySequenceAsync(trajectorySequence);
                        liftTimer.reset();
                        firstState.set(false);
                        claw.autoCollect();
                    }
                    if (clawSwitch.isPressed()) {
                        liftTimer.reset();
                        liftState.set(LiftState.LIFT_COLLECT4);
                        firstState.set(true);
                    }
                    break;
                }
                case LIFT_COLLECT4: {
                    claw.closeClaw();
                    angler.setAngle(HORIZ_ANGLE);

                    if (firstState.get() && liftTimer.seconds() >= COLLECT_PAUSE) {
                        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addTemporalMarker(() -> {
                                    lift.setTargetHeight(HIGH_JUNCTION);
                                    liftState.set(LiftState.LIFT_HIGH5);
                                    firstState.set(true);
                                    liftTimer.reset();
                                })
                                .lineToLinearHeading(dropPose,
                                        SampleMecanumDrive.getVelocityConstraint(TRAJECTORY_VELOCITY, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();


                        drive.followTrajectorySequenceAsync(trajectorySequence);
                        firstState.set(false);
                    }
                    break;
                }
                case LIFT_HIGH5: {
                    angler.setAngle(HORIZ_ANGLE);
                    lift.setTargetHeight(HIGH_JUNCTION);
                    if (lift.stalling && firstState.get()) {
                        liftTimer.reset();
                        firstState.set(false);
                    }
                    else if (lift.stalling && liftTimer.seconds() > PAUSE_TIME) {
                        claw.dropCone();
                        liftState.set(LiftState.LIFT_DROP5);
                        liftTimer.reset();
                        firstState.set(true);
                    }
                    break;
                }
                case LIFT_DROP5: {
                    angler.setAngle(GROUND_ANGLE);
                    if (liftTimer.seconds() >= DROP_PAUSE) {
                        claw.closeClaw();
                        lift.setTargetHeight(MINIMUM_HEIGHT);
                        liftState.set(LiftState.LIFT_COLLECTWAIT5);
                        liftTimer.reset();
                    }
                    break;
                }
                case LIFT_COLLECTWAIT5: {
                    if (lift.getHeight() < MID_JUNCTION && firstState.get()) {
                        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(HLstackPose5,
                                        SampleMecanumDrive.getVelocityConstraint(TRAJECTORY_VELOCITY, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addTemporalMarker(() -> {
                                    liftTimer.reset();
                                    liftState.set(LiftState.LIFT_COLLECT5);
                                    firstState.set(true);
                                })
                                .build();

                        drive.followTrajectorySequenceAsync(trajectorySequence);
                        liftTimer.reset();
                        firstState.set(false);
                        claw.autoCollect();
                    }
                    if (clawSwitch.isPressed()) {
                        liftTimer.reset();
                        liftState.set(LiftState.LIFT_COLLECT5);
                        firstState.set(true);
                    }
                    break;
                }
                case LIFT_COLLECT5: {
                    claw.closeClaw();
                    angler.setAngle(HORIZ_ANGLE);

                    if (firstState.get() && liftTimer.seconds() >= COLLECT_PAUSE) {
                        trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addTemporalMarker(() -> {
                                    lift.setTargetHeight(HIGH_JUNCTION);
                                    liftState.set(LiftState.LIFT_HIGH6);
                                    firstState.set(true);
                                })
                                .lineToLinearHeading(dropPose,
                                        SampleMecanumDrive.getVelocityConstraint(TRAJECTORY_VELOCITY, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();

                        drive.followTrajectorySequenceAsync(trajectorySequence);
                        firstState.set(false);
                    }
                    break;
                }
                case LIFT_HIGH6: {
                    angler.setAngle(HORIZ_ANGLE);
                    lift.setTargetHeight(HIGH_JUNCTION);
                    if (lift.stalling && firstState.get()) {
                        liftTimer.reset();
                        firstState.set(false);
                    }
                    else if (lift.stalling && liftTimer.seconds() > PAUSE_TIME) {
                        claw.dropCone();
                        liftState.set(LiftState.LIFT_DROP6);
                        liftTimer.reset();
                        firstState.set(true);
                    }
                    break;
                }
                case LIFT_DROP6: {
                    angler.setAngle(GROUND_ANGLE);
                    if (liftTimer.seconds() >= DROP_PAUSE) {
                        claw.closeClaw();
                        lift.setTargetHeight(MINIMUM_HEIGHT);
                        liftState.set(LiftState.LIFT_END);
                        liftTimer.reset();
                    }
                    break;
                }
                case LIFT_END: {
                    switch (signalNumber) {
                        case 0: {
                            trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(parkingPose2,
                                            SampleMecanumDrive.getVelocityConstraint(TRAJECTORY_VELOCITY, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                    .build();
                            break;
                        }
                        case 1: {
                            trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(parkingPose1,
                                            SampleMecanumDrive.getVelocityConstraint(TRAJECTORY_VELOCITY, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                    .build();
                            break;
                        }
                        case 2: {
                            trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(parkingPose2,
                                            SampleMecanumDrive.getVelocityConstraint(TRAJECTORY_VELOCITY, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                    .build();
                            break;
                        }
                        case 3: {
                            trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(parkingPose3,
                                            SampleMecanumDrive.getVelocityConstraint(TRAJECTORY_VELOCITY, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                    .build();
                            break;
                        }

                    }

                    if (firstState.get()) {
                        drive.followTrajectorySequenceAsync(trajectorySequence);
                        firstState.set(false);
                    }
                    angler.setAngle(COLLECT_ANGLE);
                    claw.closeClaw();

                    break;
                }
            }
            lift.update();
            drive.update();
        }
    }

}
