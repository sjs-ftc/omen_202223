package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.Angler;
import org.firstinspires.ftc.teamcode.drive.Claw;
import org.firstinspires.ftc.teamcode.drive.DriveTrain;
import org.firstinspires.ftc.teamcode.drive.Lift;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;


@TeleOp(group = "drive")
public class Main extends LinearOpMode {


    private Gamepad lastGamepad1 = gamepad1;
    private Gamepad lastGamepad2 = gamepad2;
    private boolean clawPos = false;

        public void moveArm(Lift lift, Angler angler) {

            int height;
            if (gamepad1.y && !lastGamepad1.y) {
                height = HIGH_POLE;
                lift.goToHeight(height);
                angler.setAngle(HORIZ_ANGLE);
            }
            else if (gamepad1.x && !lastGamepad1.x) {
                height = LOW_POLE;
                lift.goToHeight(height);
                angler.setAngle(HORIZ_ANGLE);
            }
            else if (gamepad1.b && !lastGamepad1.b) {
                height = MID_POLE;
                lift.goToHeight(height);
                angler.setAngle(HORIZ_ANGLE);
            }
            else if (gamepad1.a && !lastGamepad1.a) {
                height = MINIMUM_HEIGHT;
                lift.goToHeight(height);
                angler.setAngle(HORIZ_ANGLE);
            }
        }

        public void manageClaw(Claw claw) throws InterruptedException {
            if (gamepad1.options && !lastGamepad1.options) {
                if (clawPos) {
                    claw.closeClaw();
                    clawPos = false;
                }
                else {
                    claw.openClaw();
                    clawPos = true;
                }
                Log.d("Claw State", "A pressed: ");
                claw.closeClaw();
                clawPos = false;
            }
        }

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain drive = new DriveTrain(hardwareMap);

        Claw claw = new Claw(hardwareMap);
        Angler angler = new Angler(hardwareMap);
        Lift lift = new Lift(hardwareMap, telemetry);

        waitForStart();

        while (!isStopRequested()) {
            manageClaw(claw);
            moveArm(lift, angler);
            drive.drive(gamepad1);

            lastGamepad1 = gamepad1;
            drive.update();
        }
    }
}
