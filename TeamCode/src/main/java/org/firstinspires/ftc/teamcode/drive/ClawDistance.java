package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ClawDistance {

    Rev2mDistanceSensor distance;
    LinearOpMode opMode;

    public ClawDistance(LinearOpMode opMode, HardwareMap hardwareMap) {
        this.opMode = opMode;
        distance = hardwareMap.get(Rev2mDistanceSensor.class, "clawDistance");
    }

    public double getDistance(){
        return distance.getDistance(DistanceUnit.CM);
    }

    public boolean coneDetected() {
        if (distance.getDistance(DistanceUnit.CM) < CONE_DIST) {
            return true;
        }
        return false;
    }
}
