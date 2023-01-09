package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;


public class Lift {

    DcMotorEx leftSlide, rightSlide;
    Telemetry telemetry;
    LinearOpMode opMode;
    int targetHeight;
    public boolean stalling;

    public Lift( LinearOpMode opmode, HardwareMap hardwareMap, Telemetry telemetry) {
        this.opMode = opmode;
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.telemetry = telemetry;
        targetHeight = MINIMUM_HEIGHT;
    }

    public void update() {
        if (!(getHeight() >= (targetHeight - TICK_ERROR) && getHeight() <= (targetHeight + TICK_ERROR))) {
            rightSlide.setPower(POWER);
            leftSlide.setPower(POWER);
            stalling = false;
        }
        else {
            stall();
            stalling = true;
        }
    }

    public void setTargetHeight(int height) {
        targetHeight = height;
        height = height - MINIMUM_HEIGHT;

        int currentPos = (int) Math.round((leftSlide.getCurrentPosition()*CIRCUMFERENCE)/(ENCODER_CPR*GEAR_RATIO));

        int distance = (int) (height * 1/Math.sin(Math.toRadians(LIFT_ANGLE)));
        distance = distance - currentPos;

        double ROTATIONS = distance / CIRCUMFERENCE;
        double COUNTS = Math.round(ENCODER_CPR * ROTATIONS * GEAR_RATIO);
        int length = (int) COUNTS;

        rightSlide.setTargetPosition(rightSlide.getCurrentPosition() - length);
        leftSlide.setTargetPosition(leftSlide.getCurrentPosition() + length);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     Set the bottom of the last linear slide to be at the input height relative to the surface the robot drives on
     */
    public void goToHeight(int height) {
        height = height - MINIMUM_HEIGHT;

        int currentPos = (int) Math.round((leftSlide.getCurrentPosition()*CIRCUMFERENCE)/(ENCODER_CPR*GEAR_RATIO));

        int distance = (int) (height * 1/Math.sin(Math.toRadians(LIFT_ANGLE)));
        distance = distance - currentPos;

        double ROTATIONS = distance / CIRCUMFERENCE;
        double COUNTS = Math.round(ENCODER_CPR * ROTATIONS * GEAR_RATIO);
        int length = (int) COUNTS;

        rightSlide.setTargetPosition(rightSlide.getCurrentPosition() - length);
        leftSlide.setTargetPosition(leftSlide.getCurrentPosition() + length);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightSlide.setPower(POWER);
        leftSlide.setPower(POWER);
        while(opMode.opModeIsActive() && leftSlide.isBusy() && rightSlide.isBusy()) {}

        rightSlide.setPower(STATIC_POWER);
        leftSlide.setPower(STATIC_POWER);
    }

    public void stall() {
        rightSlide.setPower(STATIC_POWER);
        leftSlide.setPower(STATIC_POWER);
    }

    public int getHeight() {
        int currentPos = (int) Math.round(((leftSlide.getCurrentPosition()*CIRCUMFERENCE)/(ENCODER_CPR*GEAR_RATIO)) * Math.sin(Math.toRadians(LIFT_ANGLE))) + MINIMUM_HEIGHT;
        telemetry.addData("LiftHeight",currentPos);
        return currentPos;
    }

}
