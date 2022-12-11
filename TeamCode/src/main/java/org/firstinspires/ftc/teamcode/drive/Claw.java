package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Claw {

    Servo clawServo;
    public static double servoPosition = 0.0;

    public Claw(HardwareMap hardwareMap) {
        clawServo = hardwareMap.servo.get("claw");
    }

    public void openClaw() throws InterruptedException {
        servoPosition = -.75;
        clawServo.setPosition(servoPosition);
    }

    public void closeClaw() throws InterruptedException {
        servoPosition = 0.9;
        clawServo.setPosition(servoPosition);
    }

    public double getPos() {
        return clawServo.getPosition();
    }
}
