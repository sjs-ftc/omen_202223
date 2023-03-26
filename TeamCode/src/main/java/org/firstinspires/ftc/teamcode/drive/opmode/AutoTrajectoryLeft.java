package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Angler;
import org.firstinspires.ftc.teamcode.drive.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.Claw;
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

@Disabled
@Autonomous(name = "Trajectory Auto Left", group = "Game")
public class AutoTrajectoryLeft extends LinearOpMode {


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
    Pose2d stackPose = HLstackPose1;
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
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
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

        ElapsedTime liftTimer = new ElapsedTime();

        int signalNumber = 3;
        telemetry.addData("Init","Before Loop");
        telemetry.update();

        claw.closeClaw();
        lift.setTargetHeight(SAFE_HEIGHT);
        while (!isStarted() && !isStopRequested()) {
            lift.update();
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

        drive.setPoseEstimate(startPose);

        AtomicBoolean firstState = new AtomicBoolean(true);

        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    angler.setAngle(SAFE_ANGLE);
                })
                .lineToLinearHeading(readyPose,
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(dropPose)
                .build();

        while (!isStopRequested()) {
            lift.update();
            drive.update();
        }
    }

}
