package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.DRIVE_POWER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {

    private SampleMecanumDrive drive;

    public DriveTrain(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void drive(Gamepad gamepad1) {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -Math.pow(gamepad1.left_stick_y,3) * DRIVE_POWER,
                        -Math.pow(gamepad1.left_stick_x,3) * DRIVE_POWER,
                        -Math.pow(gamepad1.right_stick_x,3) * DRIVE_POWER
                )
        );
    }

    public void update() {
        drive.update();
    }
}
