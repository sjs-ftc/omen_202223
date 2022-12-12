package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;


public class Lift {

    DcMotorEx leftSlide, rightSlide;
    Telemetry telemetry;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
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
        telemetry.addData("currentHeight",currentPos);

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
        while(leftSlide.isBusy() && rightSlide.isBusy()) {}
        telemetry.addData("finished", "True");
        telemetry.addData("RPosF", rightSlide.getCurrentPosition());
        telemetry.addData("LPosF", leftSlide.getCurrentPosition());
        telemetry.update();
        rightSlide.setPower(STATIC_POWER);
        leftSlide.setPower(STATIC_POWER);
    }
}
