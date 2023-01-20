package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.GeneralConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Distances {

    DistanceSensor rightDistance, leftDistance;
    LinearOpMode opMode;

    public Distances(LinearOpMode opMode, HardwareMap hardwareMap) {
        this.opMode = opMode;
        rightDistance = hardwareMap.get(DistanceSensor.class,"rightDistance");
        leftDistance = hardwareMap.get(DistanceSensor.class,"leftDistance");
    }

    public double[] getDistances() {
        double[] distances = new double[2];
        distances[0] = rightDistance.getDistance(DistanceUnit.CM);
        distances[1] = leftDistance.getDistance(DistanceUnit.CM);
        return distances;
    }
}
