package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public final class GeneralConstants {

    //RR
    public static double PID_REST = .2;
    //Main
    public static double DRIVE_POWER = 10.0/10.0;
    public static int LOW_JUNCTION = 370;
    public static int MID_JUNCTION = 610;
    public static int HIGH_JUNCTION = 890;
    public static int SAFE_HEIGHT = 320;
    public static int AUTO_COLLECT_HEIGHT_1 = 260;
    public static int AUTO_COLLECT_HEIGHT_2 = 290;
    public static int AUTO_COLLECT_HEIGHT_3 = 300;
    public static int AUTO_COLLECT_HEIGHT_4 = 320;
    public static int AUTO_COLLECT_HEIGHT_5 = 340;
    public static int GROUND_JUNCTION = 250;
    public static double PAUSE_TIME = 1.3;
    public static double COLLECT_PAUSE = 0.2;
    public static double DROP_PAUSE = .5;
    public static double TICK_ERROR = 3;
    public static double BUTTON_PAUSE = .2;


    //Lift
    public static int MINIMUM_HEIGHT = 240; // also in Main
    public static double ENCODER_CPR = 384.5;
    public static double POWER = 1.0;
    public static double GEAR_RATIO = 1.0;
    public static double CIRCUMFERENCE = 112;
    public static double STATIC_POWER = .45;

    /**
     * Angle of the lift in degrees
     * call Math.toRadians() before doing trig with this
     */
    public static double LIFT_ANGLE = 62;


    //Angler
    public static double HORIZ_ANGLE = .0;
    public static double MAX_ANGLE = 1;
    public static double MIN_ANGLE = .0;
    public static double SAFE_ANGLE = .53;
    public static double GROUND_ANGLE = .94;
    public static double START_ANGLE = .15;
    public static double CALIBRATION_ANGLE = 1.0;


    public static double STACK_2 = .9;
    public static double STACK_3 = .86;
    public static double STACK_4 = .8;
    public static double STACK_5 = .75;

    //Autonomous
        public static double AUTO_STACK_1 = 1;
        public static double AUTO_STACK_2 = 1;
        public static double AUTO_STACK_3 = .97;
        public static double AUTO_STACK_4 = .96;
        public static double AUTO_STACK_5 = .93;



    //Claw
    public static double CLOSED_POS = 0.28;
    public static double COLLECT = 0.15;
    public static double DROP_OPEN = 0.165;
    public static double OPEN_POS = .185;


    //Distances
    public static double CONE_DIST = 7;

    //Color
    public static double RED_VALUE = .75;

    //Auto

    public static double TRAJECTORY_VELOCITY = 50;
        //Red Right
            public static Pose2d RRstartPose = new Pose2d(34, -64, 3*Math.PI/2);
            public static Pose2d RRstackPose = new Pose2d(60,-12,0);
            public static Pose2d RRdropPose = new Pose2d(47,-10,Math.toRadians(-26));
            public static Pose2d RRreadyPose = new Pose2d(36,-14,0);
            public static Pose2d RRparkingPose1 = new Pose2d(12,-12,0);
            public static Pose2d RRparkingPose2 = new Pose2d(37,-12,0);
            public static Pose2d RRparkingPose3 = new Pose2d(58,-12,0);

        //Left
            public static Pose2d LstartPose = new Pose2d(-34, -63, 3*Math.PI/2);
            public static Pose2d LstackPose = new Pose2d(-58.5,-12.5,Math.PI);
            public static Pose2d Lcone5Pose = new Pose2d(-59,-12.5,Math.PI);
            public static Pose2d LdropPose = new Pose2d(-42,-10.5,Math.toRadians(210));
            public static Pose2d LreadyPose = new Pose2d(-36,-14,Math.PI);
            public static Pose2d LparkingPose1 = new Pose2d(-56,-12,Math.PI);
            public static Pose2d LparkingPose2 = new Pose2d(-35,-12,Math.PI);
            public static Pose2d LparkingPose3 = new Pose2d(-10,-12,Math.PI);


    //Blue Right
            public static Pose2d BRstartPose = new Pose2d(34, -64, 3*Math.PI/2);
            public static Pose2d BRstackPose = new Pose2d(64,-13.5,0);
            public static Pose2d BRcone5Pose = new Pose2d(62,-12.5,0);
            public static Pose2d BRdropPose = new Pose2d(38,-12,Math.toRadians(-37));
            public static Pose2d BRreadyPose = new Pose2d(36,-14,0);
            public static Pose2d BRparkingPose1 = new Pose2d(12,-12,0);
            public static Pose2d BRparkingPose2 = new Pose2d(40,-12,0);
            public static Pose2d BRparkingPose3 = new Pose2d(60,-12,0);

        //Blue Left
            public static double BLstackAdjust = 1.0;
            public static Pose2d BLstartPose = new Pose2d(-34, -64, 3*Math.PI/2);
            public static Pose2d BLstackPose = new Pose2d(-67,-13.5,Math.PI);
            public static Pose2d BLcone5Pose = new Pose2d(-65,-12.5,Math.PI);
            public static Pose2d BLdropPose = new Pose2d(-38.5,-12.5,Math.toRadians(222));
            public static Pose2d BLdropreadyPose = new Pose2d(-45,-12.5,Math.toRadians(200));
            public static Pose2d BLreadyPose = new Pose2d(-36,-14,Math.PI);
            public static Pose2d BLparkingPose1 = new Pose2d(-60,-12,Math.PI);
            public static Pose2d BLparkingPose2 = new Pose2d(-42,-12,Math.PI);
            public static Pose2d BLparkingPose3 = new Pose2d(-15,-12,Math.PI);

    //Vuforia
        public static float MIN_CONFIDENCE = .65f;
        public static double IMAGE_TIME = 1;

    //April Tags
        public static double TAG_SIZE = .05;
}
