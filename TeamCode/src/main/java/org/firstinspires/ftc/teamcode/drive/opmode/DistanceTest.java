package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.ClawDistance;
import org.firstinspires.ftc.teamcode.drive.Distances;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@TeleOp
public class DistanceTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        ClawDistance distance = new ClawDistance(this,hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("distance",distance.getDistance());
            telemetry.addData("Cone Detected",distance.coneDetected());
            telemetry.update();

        }
    }
}
