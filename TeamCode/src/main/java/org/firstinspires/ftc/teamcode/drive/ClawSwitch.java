package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ClawSwitch {

    DigitalChannel clawSwitch;
    LinearOpMode opMode;

    public ClawSwitch(LinearOpMode opMode, HardwareMap hardwareMap) {
        clawSwitch = hardwareMap.get(DigitalChannel.class,"clawSwitch");
    }

    public boolean isPressed() {
        return !clawSwitch.getState();
    }
}
