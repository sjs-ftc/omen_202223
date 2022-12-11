package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Angler {

    Servo leftAngle, rightAngle;
    int leftPos, rightPos;
    int maxAngle;

    public Angler(HardwareMap hardwareMap) {
        maxAngle = 180;
        leftAngle = hardwareMap.servo.get("leftAngle");
        rightAngle = hardwareMap.servo.get("rightAngle");
        //leftAngle.scaleRange(0,180);
        //rightAngle.scaleRange(0,180);
    }

    /**Set angle from .2 to 1,
     * 1 is parrallel to the slides,
     * .2 is horizontal
     */
    public void setAngle(double angle) {
        leftAngle.setPosition(1 - angle);
        rightAngle.setPosition(angle);
    }

}
