package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Angler;
import org.firstinspires.ftc.teamcode.drive.Claw;
import org.firstinspires.ftc.teamcode.drive.Lift;

@TeleOp(name = "Claw Test")

public class ClawTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(this,hardwareMap);
        Lift lift = new Lift(this,hardwareMap,telemetry);
        Angler angler = new Angler(this,hardwareMap);

        claw.collect();
        angler.setAngle(1);

        waitForStart();

        while(!isStopRequested()) {
            lift.setTargetHeight(240);

            lift.update();
        }

    }


}
