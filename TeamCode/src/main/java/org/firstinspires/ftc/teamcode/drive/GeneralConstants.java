package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public final class GeneralConstants {

    //Main
    public static double DRIVE_POWER = 5.0/10.0;
    public static int LOW_JUNCTION = 400;
    public static int MID_JUNCTION = 650;
    public static int HIGH_JUNCTION = 930;
    public static int AUTO_COLLECT_HEIGHT_1 = 235;
    public static int AUTO_COLLECT_HEIGHT_2 = 255;
    public static int AUTO_COLLECT_HEIGHT_3 = 280;
    public static int AUTO_COLLECT_HEIGHT_4 = 285;
    public static int AUTO_COLLECT_HEIGHT_5 = 295;
    public static int GROUND_JUNCTION = 250;
    public static double PAUSE_TIME = 1.0;
    public static double COLLECT_PAUSE = 0.6;
    public static double DROP_PAUSE = 0.6;
    public static double TICK_ERROR = 5;
    public static double ANGLE_PAUSE = .75;
    public static double BUTTON_PAUSE = .5;


    //Lift
    public static int MINIMUM_HEIGHT = 210; // also in Main
    public static double ENCODER_CPR = 384.5;
    public static double POWER = 1.0;
    public static double GEAR_RATIO = 1.0;
    public static double CIRCUMFERENCE = 112;
    public static double STATIC_POWER = .35;

    /**
     * Angle of the lift in degrees
     * call Math.toRadians() before doing trig with this
     */
    public static int LIFT_ANGLE = 55;


    //Angler
    public static double HORIZ_ANGLE = .075;
    public static double MAX_ANGLE = 1;
    public static double MIN_ANGLE = .075;
    public static double SAFE_ANGLE = .53;
    public static double GROUND_ANGLE = .96;

    public static double STACK_2 = .91;
    public static double STACK_3 = .86;
    public static double STACK_4 = .81;
    public static double STACK_5 = .77;

    //Autonomous
        public static double AUTO_GROUND = 1;
        public static double AUTO_STACK_2 = .99;
        public static double AUTO_STACK_3 = .95;
        public static double AUTO_STACK_4 = .89;
        public static double AUTO_STACK_5 = .88;



    //Claw
    public static double CLOSED_POS = 0.0;
    public static double COLLECT = 0.145;
    public static double DROP_OPEN = 0.12;
    public static double OPEN_POS = .145;


    //Distances
    public static double CONE_DIST = 8.5;

    //Color
    public static double RED_VALUE = .75;

    //Auto
        //Red Right
            public static Pose2d RRstartPose = new Pose2d(36, -64, 3*Math.PI/2);
            public static Pose2d RRstackPose = new Pose2d(60,-12,0);
            public static Pose2d RRdropPose = new Pose2d(50,-12,Math.toRadians(-24));
            public static Pose2d RRreadyPose = new Pose2d(36,-14,0);
            public static Pose2d RRparkingPose1 = new Pose2d(12,-12,0);
            public static Pose2d RRparkingPose2 = new Pose2d(35,-12,0);
            public static Pose2d RRparkingPose3 = new Pose2d(58,-12,0);

        //Red Left
            public static Pose2d RLstartPose = new Pose2d(-36, -64, 3*Math.PI/2);
            public static Pose2d RLstackPose = new Pose2d(-60,-12,Math.PI);
            public static Pose2d RLdropPose = new Pose2d(-50,-12,Math.toRadians(204));
            public static Pose2d RLreadyPose = new Pose2d(-36,-14,Math.PI);
            public static Pose2d RLparkingPose1 = new Pose2d(-12,-12,Math.PI);
            public static Pose2d RLparkingPose2 = new Pose2d(-35,-12,Math.PI);
            public static Pose2d RLparkingPose3 = new Pose2d(-58,-12,Math.PI);

        //Blue Right
            public static Pose2d BRstartPose = new Pose2d(36, -64, 3*Math.PI/2);
            public static Pose2d BRstackPose = new Pose2d(62,-12,0);
            public static Pose2d BRdropPose = new Pose2d(52,-14,Math.toRadians(-26));
            public static Pose2d BRreadyPose = new Pose2d(36,-14,0);
            public static Pose2d BRparkingPose1 = new Pose2d(12,-12,0);
            public static Pose2d BRparkingPose2 = new Pose2d(37,-12,0);
            public static Pose2d BRparkingPose3 = new Pose2d(58,-12,0);

        //Blue Left
            public static Pose2d BLstartPose = new Pose2d(-36, -64, 3*Math.PI/2);
            public static Pose2d BLstackPose = new Pose2d(-61,-12,Math.PI);
            public static Pose2d BLdropPose = new Pose2d(-49,-12,Math.toRadians(206));
            public static Pose2d BLreadyPose = new Pose2d(-36,-14,Math.PI);
            public static Pose2d BLparkingPose1 = new Pose2d(-12,-12,Math.PI);
            public static Pose2d BLparkingPose2 = new Pose2d(-35,-12,Math.PI);
            public static Pose2d BLparkingPose3 = new Pose2d(-58,-12,Math.PI);

    //Vuforia
        public static float MIN_CONFIDENCE = .65f;
        public static double IMAGE_TIME = 1;
}
