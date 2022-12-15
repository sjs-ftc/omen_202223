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

    public Lift( LinearOpMode opmode, HardwareMap hardwareMap, Telemetry telemetry) {
        this.opMode = opmode;
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.telemetry = telemetry;
    }

    /**
     Set the bottom of the last linear slide to be at the input height relative to the surface the robot drives on
     */
    public void goToHeight(int height) {
        height = height - MINIMUM_HEIGHT;

        int currentPos = (int) Math.round((leftSlide.getCurrentPosition()*CIRCUMFERENCE)/(ENCODER_CPR*GEAR_RATIO));
        telemetry.addData("currentHeight",currentPos*Math.sin(Math.toRadians(LIFT_ANGLE)) + MINIMUM_HEIGHT);

        int distance = (int) (height * 1/Math.sin(Math.toRadians(LIFT_ANGLE)));
        distance = distance - currentPos;

        telemetry.addData("DISTANCE", distance);

        double ROTATIONS = distance / CIRCUMFERENCE;
        double COUNTS = Math.round(ENCODER_CPR * ROTATIONS * GEAR_RATIO);
        int length = (int) COUNTS;

        telemetry.addData("LPosI", leftSlide.getCurrentPosition());
        telemetry.addData("Counts", COUNTS);
        telemetry.addData("length",length);

        telemetry.addData("RPosI", rightSlide.getCurrentPosition());
        telemetry.addData("LPosI", leftSlide.getCurrentPosition());
        rightSlide.setTargetPosition(rightSlide.getCurrentPosition() - length);
        leftSlide.setTargetPosition(leftSlide.getCurrentPosition() + length);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setPower(POWER);
        leftSlide.setPower(POWER);
        while(opMode.opModeIsActive() && leftSlide.isBusy() && rightSlide.isBusy()) {}
        telemetry.addData("finished", "True");
        telemetry.addData("RPosF", rightSlide.getCurrentPosition());
        telemetry.addData("LPosF", leftSlide.getCurrentPosition());
        telemetry.update();
        rightSlide.setPower(STATIC_POWER);
        leftSlide.setPower(STATIC_POWER);
    }

    public int getHeight() {
        int currentPos = (int) Math.round(((leftSlide.getCurrentPosition()*CIRCUMFERENCE)/(ENCODER_CPR*GEAR_RATIO)) * Math.sin(Math.toRadians(LIFT_ANGLE))) + MINIMUM_HEIGHT;
        telemetry.addData("LiftHeight",currentPos);
        return currentPos;
    }

    public void depositCone(int height, Angler angler) {

        goToHeight(height);
        angler.slowAngle(HORIZ_ANGLE);
        double startWait = opMode.getRuntime();
        while (opMode.getRuntime() < startWait + PAUSE_TIME) {
        }
        angler.slowAngle(SAFE_ANGLE);
        goToHeight(MINIMUM_HEIGHT);
    }
}
