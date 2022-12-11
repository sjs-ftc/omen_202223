package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
        height = height - 150;

        double power = 1.0;
        double ENCODER_CPR = 537.7;
        double GEAR_RATIO = 1;
        double CIRCUMFERENCE = 112;

        int currentPos = (int) Math.round((leftSlide.getCurrentPosition()*CIRCUMFERENCE)/(ENCODER_CPR*GEAR_RATIO));
        telemetry.addData("currentHeight",currentPos);

        int distance = (int) (height * 1/Math.sin(Math.toRadians(45)));
        distance = distance - currentPos;

        telemetry.addData("DISTANCE", distance);

        double ROTATIONS = distance / CIRCUMFERENCE;
        double COUNTS = Math.round(ENCODER_CPR * ROTATIONS * GEAR_RATIO);
        int length = (int) COUNTS;

        telemetry.addData("RPosI", leftSlide.getCurrentPosition());
        telemetry.addData("Counts", COUNTS);
        telemetry.addData("length",length);

        //if (length < 0) {
        //    power = .5;
        //}

        telemetry.addData("RPosI", rightSlide.getCurrentPosition());
        telemetry.addData("LPosI", leftSlide.getCurrentPosition());
        rightSlide.setTargetPosition(rightSlide.getCurrentPosition() - length);
        leftSlide.setTargetPosition(leftSlide.getCurrentPosition() + length);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setPower(power);
        leftSlide.setPower(power);
        while(leftSlide.isBusy() && rightSlide.isBusy()) {}
        telemetry.addData("finished", "True");
        telemetry.addData("RPosF", rightSlide.getCurrentPosition());
        telemetry.addData("LPosF", leftSlide.getCurrentPosition());
        telemetry.update();
        rightSlide.setPower(.1);
        leftSlide.setPower(.1);
    }
}
