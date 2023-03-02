package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Angler;
import org.firstinspires.ftc.teamcode.drive.Lift;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;

@Disabled
@Autonomous()
public class LiftTest extends LinearOpMode {



    private void slowAngle(double angle, Angler angler) {
        double currentAngle = angler.getAngle();
        double tick = (angle - currentAngle)/10.0;

        for (int u = 0; u < 10; u++) {
            angler.setAngle(currentAngle + tick);
            currentAngle = angler.getAngle();
            double startWait = getRuntime();
            while (getRuntime() < startWait + .1) {
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Angler angler = new Angler(this, hardwareMap);
        Lift lift = new Lift(this, hardwareMap, telemetry);
        angler.setAngle(.75);
        waitForStart();

        //lift.depositCone(HIGH_JUNCTION, angler);
        slowAngle(HORIZ_ANGLE, angler);

        double startWait = getRuntime();
        while (getRuntime() < startWait + 3) {
        }
        lift.goToHeight(MINIMUM_HEIGHT);
        angler.setAngle(.95);
        while (!isStopRequested()) {
            //angler.setAngle(.65);
        }
    }
}