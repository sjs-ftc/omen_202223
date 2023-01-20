package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Angler;
import org.firstinspires.ftc.teamcode.drive.Claw;
import org.firstinspires.ftc.teamcode.drive.Lift;

@Autonomous()
public class AngleTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Angler angler = new Angler(this, hardwareMap);
        Claw claw = new Claw(this, hardwareMap);
        Lift lift = new Lift(this, hardwareMap, telemetry);

        //claw.collect();
        angler.setAngle(GROUND_ANGLE);
        claw.collect();
        waitForStart();

        int i = 0;
        while (!isStopRequested()) {
            lift.setTargetHeight(AUTO_COLLECT_HEIGHT_1);
            angler.setAngle(AUTO_STACK_3);

            lift.update();

        }
    }
}