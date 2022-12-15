package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class GeneralConstants {

    //Main
    public static double DRIVE_POWER = 5.0/10.0;
    public static int LOW_JUNCTION = 400;
    public static int MID_JUNCTION = 650;
    public static int HIGH_JUNCTION = 920;
    public static int GROUND_JUNCTION = 250;
    public static double PAUSE_TIME = 1.0;
    public static double COLLECT_PAUSE = 0.6;
    public static double TICK_ERROR = 20;




    //Lift
    public static int MINIMUM_HEIGHT = 210; // also in Main
    public static double ENCODER_CPR = 537.7;
    public static double POWER = 1.0;
    public static double GEAR_RATIO = 1.0;
    public static double CIRCUMFERENCE = 112;
    public static double STATIC_POWER = .15;

    /**
     * Angle of the lift in degrees
     * call Math.toRadians() before doing trig with this
     */
    public static int LIFT_ANGLE = 55;


    //Angler
    public static double HORIZ_ANGLE = .26;
    public static double MAX_ANGLE = 1;
    public static double SAFE_ANGLE = .75;
    public static double GROUND_ANGLE = .915;

        //Autonomous
        public static double STACK_2 = .89;
        public static double STACK_3 = .87;
        public static double STACK_4 = .85;
        public static double STACK_5 = .83;


    //Claw
    public static double CLOSED_POS = 0.0;
    public static double COLLECT = 0.15;
    public static double DROP_OPEN = 0.12;
    public static double OPEN_POS = .4;

}
