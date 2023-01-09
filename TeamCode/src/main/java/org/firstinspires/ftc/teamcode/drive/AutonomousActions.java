package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.HIGH_JUNCTION;
import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.HORIZ_ANGLE;
import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.LOW_JUNCTION;
import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.MID_JUNCTION;
import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.MINIMUM_HEIGHT;
import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.PAUSE_TIME;
import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.SAFE_ANGLE;
import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.TICK_ERROR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutonomousActions {



    private LinearOpMode opmode;

    public void dropCone(Lift lift, Angler angler, Claw claw) {
        claw.dropCone();
        double startWait = this.opmode.getRuntime();
        while (this.opmode.getRuntime() < startWait + PAUSE_TIME) {
        }
        claw.closeClaw();
        angler.setAngle(SAFE_ANGLE);
        lift.goToHeight(MINIMUM_HEIGHT);
        while (this.opmode.opModeIsActive() && !(lift.getHeight() >= (MINIMUM_HEIGHT - TICK_ERROR) && lift.getHeight() <= (MINIMUM_HEIGHT + TICK_ERROR))) {
            lift.goToHeight(MINIMUM_HEIGHT);
        }
        claw.closeClaw();
    }

    public AutonomousActions(LinearOpMode opmode) {
        this.opmode = opmode;
    }

    public void depositLow(Lift lift, Angler angler, Claw claw){
        claw.closeClaw();
        angler.slowAngle(SAFE_ANGLE);
        lift.goToHeight(LOW_JUNCTION);
        while (this.opmode.opModeIsActive() && !(lift.getHeight() >= (HIGH_JUNCTION - TICK_ERROR) && lift.getHeight() <= (HIGH_JUNCTION + TICK_ERROR))) {
            lift.goToHeight(HIGH_JUNCTION);
        }
        angler.slowAngle(HORIZ_ANGLE);
        double startWait = this.opmode.getRuntime();
        while (this.opmode.getRuntime() < startWait + PAUSE_TIME) {
        }
        dropCone(lift,angler,claw);

    }

    public void depositMid(Lift lift, Angler angler, Claw claw){
        claw.closeClaw();
        angler.slowAngle(SAFE_ANGLE);
        lift.goToHeight(MID_JUNCTION);
        while (this.opmode.opModeIsActive() && !(lift.getHeight() >= (HIGH_JUNCTION - TICK_ERROR) && lift.getHeight() <= (HIGH_JUNCTION + TICK_ERROR))) {
            lift.goToHeight(HIGH_JUNCTION);
        }
        angler.slowAngle(HORIZ_ANGLE);

        dropCone(lift,angler,claw);
    }

    public void depositHigh(Lift lift, Angler angler, Claw claw) {
        claw.closeClaw();
        angler.setAngle(HORIZ_ANGLE);
        lift.goToHeight(HIGH_JUNCTION);
        while (this.opmode.opModeIsActive() && !(lift.getHeight() >= (HIGH_JUNCTION - TICK_ERROR) && lift.getHeight() <= (HIGH_JUNCTION + TICK_ERROR))) {
            lift.goToHeight(HIGH_JUNCTION);
        }
        dropCone(lift,angler,claw);
    }


}
