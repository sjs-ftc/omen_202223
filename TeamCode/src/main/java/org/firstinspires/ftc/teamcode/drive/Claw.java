package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;


public class Claw {

    Servo clawServo;
    public static double servoPosition = CLOSED_POS;
    LinearOpMode opMode;

    public Claw( LinearOpMode opMode, HardwareMap hardwareMap){
        this.opMode = opMode;
        clawServo = hardwareMap.servo.get("claw");
        closeClaw();
    }

    public void openClaw() {
        servoPosition = OPEN_POS;
        clawServo.setPosition(servoPosition);
    }

    public void collect() {
        servoPosition = COLLECT;
        clawServo.setPosition(servoPosition);
    }

    public void autoCollect() {
        servoPosition = AUTO_COLLECT;
        clawServo.setPosition(servoPosition);
    }

    public void dropCone() {
        servoPosition = DROP_OPEN;
        clawServo.setPosition(servoPosition);
    }

    public void closeClaw() {
        servoPosition = CLOSED_POS;
        clawServo.setPosition(servoPosition);
    }

    public double getPos() {
        return clawServo.getPosition();
    }
}
