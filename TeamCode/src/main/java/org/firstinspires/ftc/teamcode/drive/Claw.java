package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;


public class Claw {

    Servo clawServo;
    public static double servoPosition = CLOSED_POS;

    public Claw(HardwareMap hardwareMap) {
        clawServo = hardwareMap.servo.get("claw");
    }

    public void openClaw() throws InterruptedException {
        servoPosition = OPEN_POS;
        clawServo.setPosition(servoPosition);
    }

    public void closeClaw() throws InterruptedException {
        servoPosition = CLOSED_POS;
        clawServo.setPosition(servoPosition);
    }

    public double getPos() {
        return clawServo.getPosition();
    }
}
