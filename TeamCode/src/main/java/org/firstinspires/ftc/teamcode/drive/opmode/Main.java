package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.Claw;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive")
public class Main extends LinearOpMode {

    private final int START = 75;


        public int moveArm(DcMotorEx rightSlide,DcMotorEx leftSlide, int currentHeight) {
            int height = 0;

            if (gamepad1.y) {
                height = 850;
                currentHeight = goToHeight(rightSlide,leftSlide,height,currentHeight);
                return currentHeight;
            }
            if (gamepad1.x) {
                height = 340;
                currentHeight = goToHeight(rightSlide,leftSlide,height,currentHeight);
                return currentHeight;
            }
            if (gamepad1.b) {
                height = 600;
                currentHeight = goToHeight(rightSlide,leftSlide,height,currentHeight);
                return currentHeight;
            }
            if (gamepad1.a) {
                height = START;
                currentHeight = goToHeight(rightSlide,leftSlide,height,currentHeight);
                return currentHeight;
            }
            return currentHeight;
        }

        /**
        Set the bottom of the last linear slide to be at the input height relative to the surface the robot drives on
         */
        public int goToHeight(DcMotorEx rightSlide,DcMotorEx leftSlide, int height, int currentHeight) {

        double power = 1.0;
        height = height - currentHeight;

        double ENCODER_CPR = 537.7;
        double GEAR_RATIO = 10.0/14.0; //10/14 is the sprocket teeth ratio
        double CIRCUMFERENCE = 112;

        double ROTATIONS = height / CIRCUMFERENCE;
        double COUNTS = Math.round(ENCODER_CPR * ROTATIONS * GEAR_RATIO);

        int length = (int) COUNTS;

        telemetry.addData("RPosI", rightSlide.getCurrentPosition());
        telemetry.addData("Counts", COUNTS);
        telemetry.addData("length",length);

        if (length < 0) {
            power = .5;
        }

            telemetry.addData("RPosI", rightSlide.getCurrentPosition());
            telemetry.addData("LPosI", leftSlide.getCurrentPosition());
            rightSlide.setTargetPosition(rightSlide.getCurrentPosition() - length);
            leftSlide.setTargetPosition(leftSlide.getCurrentPosition() + length);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setPower(power);
            leftSlide.setPower(power);
            while(opModeIsActive() && leftSlide.isBusy() && rightSlide.isBusy()) {}
            telemetry.addData("finished", "True");
            telemetry.addData("RPosF", rightSlide.getCurrentPosition());
            telemetry.addData("LPosF", leftSlide.getCurrentPosition());
            telemetry.update();
            rightSlide.setPower(0);
            leftSlide.setPower(0);

            return currentHeight + height;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Claw claw = new Claw();
        Servo rightClaw, leftClaw;
        DcMotorEx rightSlide, leftSlide;
        rightClaw = hardwareMap.servo.get("rightClaw");
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //537.7
        int currentHeight = START;
        int height = 0;
        final double ENCODER_CPR = 537.7;
        final double GEAR_RATIO = 10.0/14.0; //10/14 is the sprocket teeth ratio
        final double CIRCUMFERENCE = 112;
        int DISTANCE = 850;

        final double ROTATIONS = DISTANCE / CIRCUMFERENCE ;
        final double DoubleCOUNTS = Math.round(ENCODER_CPR * ROTATIONS * GEAR_RATIO);
        final int COUNTS = (int) DoubleCOUNTS;

        telemetry.addData("COUNTS", COUNTS);
        boolean clawPos = false;

        waitForStart();

        while (!isStopRequested()) {
            if (gamepad1.left_stick_button) {
                Log.d("Claw State", "A pressed: ");
                if (clawPos) {
                   claw.closeClaw(rightClaw,leftClaw);
                   clawPos = false;
                }
                else {
                    claw.openClaw(rightClaw,leftClaw);
                    clawPos = true;
                }
            }

            //if (gamepad1.a) {
            //    height = 850;
            //    currentHeight = goToHeight(rightSlide,leftSlide,height,currentHeight);
            //}

            currentHeight = moveArm(rightSlide,leftSlide,currentHeight);
            telemetry.addData("currentHeight",currentHeight);
            telemetry.update();

            rightSlide.setVelocity(0.0);
            leftSlide.setVelocity(0.0);
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();
        }
    }
}
