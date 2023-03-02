package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.Angler;
import org.firstinspires.ftc.teamcode.drive.Claw;
import org.firstinspires.ftc.teamcode.drive.Distances;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Disabled
@Autonomous(name = "Auto: Blue Left Camera", group = "Game")
public class CameraBlueLeftAuto extends LinearOpMode {

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

    Pose2d startPose = BLstartPose;
    Pose2d stackPose = BLstackPose;
    Pose2d dropPose = BLdropPose;
    Pose2d readyPose = BLreadyPose;
    Pose2d parkingPose1 = BLparkingPose1;
    Pose2d parkingPose2 = BLparkingPose2;
    Pose2d parkingPose3 = BLparkingPose3;

    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    private static final String VUFORIA_KEY =
            "AfdxXSz/////AAABmVzIu0NamEtKgZ//wS//1OUuq+bvE4nNlADPoxwJZN10SAVnsW0DzeR22RlyNXvW3ll3ZE3I2pS/vEdo/6h2pKMrEFbdPANhJyGKh8r4Tz2KNx6oH9BcREvgMFGUozEsne7JI3g9CG1BibmsjbOnGRWCn/QuD/iZJAEErz2weomPGMeWa1hC8zVsXzI22jAPYDXCEeFAVnG+FUF2lM34oVa7imVkVVxK9vWVilJ06qWNLTM7SoEbtysgBsV9LY0pMmN3bWnAtYQP+RhrqNdrLJVuV0JAo7JNPXvX5QSIa2goRlzEYVRhcNC6e5whK4KKJ1CBbkzlsDygWboYmywTg1nKTS/IaUEG9MhQ6/nfYXUR";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    @Override
    public void runOpMode() throws InterruptedException {
        initVuforia();
        initTfod();
        ElapsedTime timer = new ElapsedTime();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }



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

        int signalNumber = 2;

        waitForStart();
        timer.reset();

        if (opModeIsActive()) {
            while (timer.seconds() < IMAGE_TIME) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            i++;

                            // check label to see if the camera now sees a Duck         ** ADDED **
                            if (recognition.getLabel().equals("1 Bolt")) {
                                signalNumber = 1;
                                telemetry.addData("Object Detected", "Parking 1");      //  ** ADDED **
                                break;
                            }
                            else if (recognition.getLabel().equals("2 Bulb")) {
                                signalNumber = 2;
                                telemetry.addData("Object Detected", "Parking 2");      //  ** ADDED **
                                break;
                            }
                            else if (recognition.getLabel().equals("3 Panel")) {
                                signalNumber = 3;
                                telemetry.addData("Object Detected", "Parking 3");      //  ** ADDED **
                                break;
                            }
                            //  ** ADDED **
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

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
                        firstState = true;
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
                    switch (signalNumber) {
                        case 0: {
                            trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(parkingPose2,
                                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                    .build();
                            break;
                        }
                        case 1: {
                            trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(parkingPose1,
                                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                    .build();
                            break;
                        }
                        case 2: {
                            trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(parkingPose2,
                                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                    .build();
                            break;
                        }
                        case 3: {
                            trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(parkingPose3,
                                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                    .build();
                            break;
                        }

                    }

                    if (firstState) {
                        drive.followTrajectorySequenceAsync(trajectorySequence);
                        firstState = false;
                    }
                    angler.setAngle(GROUND_ANGLE);
                    claw.closeClaw();

                    break;
                }
            }
            lift.update();
            drive.update();
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = MIN_CONFIDENCE;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
