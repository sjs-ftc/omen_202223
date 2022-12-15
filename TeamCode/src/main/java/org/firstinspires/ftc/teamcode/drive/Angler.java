package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Angler {

    Servo leftAngle, rightAngle;
    double leftPos, rightPos;
    int maxAngle;
    LinearOpMode opMode;

    public Angler(LinearOpMode opMode, HardwareMap hardwareMap) {
        this.opMode = opMode;
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

    public void slowAngle(double angle) {
        double currentAngle = getAngle();
        double tick = (angle - currentAngle)/10.0;

        for (int u = 0; u < 10; u++) {
            setAngle(currentAngle + tick);
            currentAngle = getAngle();
            double startWait = opMode.getRuntime();
            while (opMode.getRuntime() < startWait + .1) {
            }
        }
    }

    public double getAngle() {
        return rightAngle.getPosition();
    }

}
