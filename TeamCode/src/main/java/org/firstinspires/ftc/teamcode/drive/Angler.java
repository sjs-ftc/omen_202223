package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Angler {

    Servo leftAngle, rightAngle;
    double leftPos, rightPos;
    int maxAngle;

    public Angler(HardwareMap hardwareMap) {
        leftAngle = hardwareMap.servo.get("leftAngle");
        rightAngle = hardwareMap.servo.get("rightAngle");
    }

    /**Set angle from .2 to 1,
     * 1 is parrallel to the slides,
     * .2 is horizontal
     */
    public void setAngle(double angle) {
        if (angle < HORIZ_ANGLE) {
            angle = HORIZ_ANGLE;
        }
        leftAngle.setPosition(MAX_ANGLE - angle);
        rightAngle.setPosition(angle);
    }

    public double getAngle() {
        return rightAngle.getPosition();
    }

}
