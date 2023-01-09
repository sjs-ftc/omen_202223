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
    public static double DROP_PAUSE = 0.6;
    public static double TICK_ERROR = 5;




    //Lift
    public static int MINIMUM_HEIGHT = 210; // also in Main
    public static double ENCODER_CPR = 384.5;
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
    public static double HORIZ_ANGLE = .6;
    public static double MAX_ANGLE = .8;
    public static double MIN_ANGLE = .2;
    public static double SAFE_ANGLE = .02;
    public static double GROUND_ANGLE = .0;


    //Autonomous
        public static double STACK_2 = .01;
        public static double STACK_3 = .02;
        public static double STACK_4 = .03;
        public static double STACK_5 = .04;


    //Claw
    public static double CLOSED_POS = 0.0;
    public static double COLLECT = 0.15;
    public static double DROP_OPEN = 0.12;
    public static double OPEN_POS = .4;


}
