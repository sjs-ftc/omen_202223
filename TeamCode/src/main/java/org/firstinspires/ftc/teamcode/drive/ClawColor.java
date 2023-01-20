package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ClawColor {

    RevColorSensorV3 color;
    LinearOpMode opMode;

    public ClawColor(LinearOpMode opMode, HardwareMap hardwareMap) {
        this.opMode = opMode;
        color = hardwareMap.get(RevColorSensorV3.class, "clawColor");
    }

    public boolean coneDetected() {
        //if (color.getDistance(DistanceUnit.CM) < 4 && color.red() > RED_VALUE) {

        //}
        return true;
    }
}
