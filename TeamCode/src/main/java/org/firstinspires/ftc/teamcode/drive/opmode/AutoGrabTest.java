package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Angler;
import org.firstinspires.ftc.teamcode.drive.Claw;
import org.firstinspires.ftc.teamcode.drive.Lift;
@Disabled
@TeleOp (name = "Auto Grab Test", group = "Test")
public class AutoGrabTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Angler angler = new Angler(this, hardwareMap);
        Claw claw = new Claw(this, hardwareMap);
        Lift lift = new Lift(this, hardwareMap, telemetry);

        claw.collect();
        waitForStart();

        while (!isStopRequested()) {
            if (gamepad2.y) {
                lift.setTargetHeight(AUTO_COLLECT_HEIGHT_5);
                angler.setAngle(AUTO_STACK_5);
            }
            if (gamepad2.x) {
                lift.setTargetHeight(AUTO_COLLECT_HEIGHT_4);
                angler.setAngle(AUTO_STACK_4);
            }
            if (gamepad2.b) {
                lift.setTargetHeight(AUTO_COLLECT_HEIGHT_3);
                angler.setAngle(AUTO_STACK_3);
            }
            if (gamepad2.a) {
                lift.setTargetHeight(AUTO_COLLECT_HEIGHT_2);
                angler.setAngle(AUTO_STACK_2);
            }
            if (gamepad2.right_bumper) {
                lift.setTargetHeight(AUTO_COLLECT_HEIGHT_1);
                angler.setAngle(AUTO_STACK_1);
            }
            lift.update();
        }
    }
}
