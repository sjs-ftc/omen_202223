package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
public final class GeneralConstants {

    //Main
    public static double DRIVE_POWER = 5.0/10.0;
    public static int LOW_POLE = 340;
    public static int MID_POLE = 600;
    public static int HIGH_POLE = 750;



    //Lift
    public static int MINIMUM_HEIGHT = 210; // also in Main
    public static int LIFT_ANGLE = 45;
    public static double ENCODER_CPR = 537.7;
    public static double POWER = 1.0;
    public static double DOWN_POWER = 1.0;
    public static double GEAR_RATIO = 1.0;
    public static double CIRCUMFERENCE = 112;
    public static double STATIC_POWER = 0;

    //Angler
    public static double HORIZ_ANGLE = .22;
    public static double MAX_ANGLE = 1;



    //Claw
    public static double CLOSED_POS = .9;
    public static double OPEN_POS = 0;

}
