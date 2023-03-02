package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.GROUND_ANGLE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Angler;

@Disabled
@Autonomous
public class ServoCalibration extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Angler angler = new Angler(this, hardwareMap);

        waitForStart();
        angler.setAngle(GROUND_ANGLE);
        while (!isStopRequested()) {

        }
    }
}
