package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Claw {

    public static double rightServoPosition = 0.0;
    public static double leftServoPosition = 1.0;

    public Claw() {
    }

    public void openClaw(Servo right, Servo left) throws InterruptedException {
        rightServoPosition = 0.0;
        leftServoPosition = 1.0;
        right.setPosition(rightServoPosition);
        left.setPosition(leftServoPosition);
    }

    public void closeClaw(Servo right, Servo left) throws InterruptedException {
        rightServoPosition = 0.69;
        leftServoPosition = 0.25;
        right.setPosition(rightServoPosition);
        left.setPosition(leftServoPosition);
    }
}
