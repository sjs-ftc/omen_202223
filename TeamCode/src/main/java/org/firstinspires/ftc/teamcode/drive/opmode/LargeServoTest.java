package org.firstinspires.ftc.teamcode.drive.opmode;


import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "drive")
public class LargeServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo leftAngle = hardwareMap.servo.get("leftAngle");
        Servo rightAngle = hardwareMap.servo.get("rightAngle");
        double pos = .4;


        waitForStart();

        while (!isStopRequested()) {
            rightAngle.setPosition(pos);
            leftAngle.setPosition(pos);
            telemetry.addData("Position", pos);
            telemetry.update();
        }
    }
}