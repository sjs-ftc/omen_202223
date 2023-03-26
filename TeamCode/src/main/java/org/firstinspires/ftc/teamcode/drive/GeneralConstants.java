package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public final class GeneralConstants {

    //RR
    public static double PID_REST = .2;
    //Main
    public static double DRIVE_POWER = 1.0;
    public static int LOW_JUNCTION = 370;
    public static int MID_JUNCTION = 660;
    public static int HIGH_JUNCTION = 880;
    public static int SAFE_HEIGHT = 320;
    public static int AUTO_COLLECT_HEIGHT_1 = 260;
    public static int AUTO_COLLECT_HEIGHT_2 = 290;
    public static int AUTO_COLLECT_HEIGHT_3 = 300;
    public static int AUTO_COLLECT_HEIGHT_4 = 320;
    public static int AUTO_COLLECT_HEIGHT_5 = 340;
    public static int GROUND_JUNCTION = 250;
    public static double PAUSE_TIME = 1.3;
    public static double COLLECT_PAUSE = 0.7;
    public static double DROP_PAUSE = .5;
    public static double TICK_ERROR = 5;
    public static double BUTTON_PAUSE = .2;


    //Lift
    public static int MINIMUM_HEIGHT = 240; // also in Main
    public static double ENCODER_CPR = 384.5;
    public static double POWER = 1.0;
    public static double GEAR_RATIO = 1.0;
    public static double CIRCUMFERENCE = 112;
    public static double STATIC_POWER = .5;

    /**
     * Angle of the lift in degrees
     * call Math.toRadians() before doing trig with this
     */
    public static double LIFT_ANGLE = 58;


    //Angler
    public static double HORIZ_ANGLE = .15;
    public static double MAX_ANGLE = 1;
    public static double MIN_ANGLE = .0;
    public static double SAFE_ANGLE = .15;
    public static double GROUND_ANGLE = .99;
    public static double START_ANGLE = .17;
    public static double CALIBRATION_ANGLE = 1.0;
    public static double COLLECT_ANGLE = .3;


    public static double STACK_2 = .92;
    public static double STACK_3 = .86;
    public static double STACK_4 = .83;
    public static double STACK_5 = .79;

    //Autonomous
        public static double AUTO_STACK_1 = 1;
        public static double AUTO_STACK_2 = 1;
        public static double AUTO_STACK_3 = .97;
        public static double AUTO_STACK_4 = .96;
        public static double AUTO_STACK_5 = .93;


    //Claw
    public static double CLOSED_POS = 0.7;
    public static double COLLECT = 0.6;
    public static double AUTO_COLLECT = 0.575;
    public static double DROP_OPEN = 0.6;
    public static double OPEN_POS = 0.6;


    //Distances
    public static double CONE_DIST = 7;

    //Color
    public static double RED_VALUE = .75;

    //Auto

    public static double TRAJECTORY_VELOCITY = 40;
        //Mid Right
            public static double MRstackAdjust = 1.0;
            public static Pose2d MRstartPose = new Pose2d(34, -64, 3*Math.PI/2);
            public static Pose2d MRstackPose = new Pose2d(57,-14.5,0);
            public static Pose2d MRcone5Pose = new Pose2d(58.5,-14,0);
            public static Pose2d MRdropPose = new Pose2d(37,-13,Math.toRadians(38));
            public static Pose2d MRdropreadyPose = new Pose2d(45,-12.5,Math.toRadians(-20));
            public static Pose2d MRreadyPose = new Pose2d(36,-14,3*Math.PI/2);
            public static Pose2d MRparkingPose1 = new Pose2d(60,-12,0);
            public static Pose2d MRparkingPose2 = new Pose2d(42,-12,0);
            public static Pose2d MRparkingPose3 = new Pose2d(15,-12,0);

        //Mid Left
            public static double MLstackAdjust = 1.0;
            public static Pose2d MLstartPose = new Pose2d(-34, -64, 3*Math.PI/2);
            public static Pose2d MLstackPose = new Pose2d(-57,-13,Math.PI);
            public static Pose2d MLstackReadyPose = new Pose2d(-53,-13,Math.PI);
            public static Pose2d MLcone5Pose = new Pose2d(-58.5,-13,Math.PI);
            public static Pose2d MLdropPose = new Pose2d(-33,-15,Math.toRadians(142));
            public static Pose2d MLdropreadyPose = new Pose2d(-45,-12.5,Math.toRadians(200));
            public static Pose2d MLreadyPose = new Pose2d(-36,-14,3*Math.PI/2);
            public static Pose2d MLparkingPose1 = new Pose2d(-60,-12,Math.PI);
            public static Pose2d MLparkingPose2 = new Pose2d(-42,-12,Math.PI);
            public static Pose2d MLparkingPose3 = new Pose2d(-15,-12,Math.PI);

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
            public static double HRstackAdjust = 1.0;
            public static Pose2d HRstartPose = new Pose2d(34, -64, 3*Math.PI/2);
            public static Pose2d HRstackPose = new Pose2d(57,-14.5,0);
            public static Pose2d HRcone5Pose = new Pose2d(58.5,-14,0);
            public static Pose2d HRdropPose = new Pose2d(39,-13,Math.toRadians(-38));
            public static Pose2d HRdropreadyPose = new Pose2d(45,-12.5,Math.toRadians(-20));
            public static Pose2d HRreadyPose = new Pose2d(36,-14,3*Math.PI/2);
            public static Pose2d HRparkingPose1 = new Pose2d(60,-12,0);
            public static Pose2d HRparkingPose2 = new Pose2d(42,-12,0);
            public static Pose2d HRparkingPose3 = new Pose2d(15,-12,0);

        //High Left
            public static double HLstackAdjust = 1.0;
            public static Pose2d HLstartPose = new Pose2d(-34, -63, 3*Math.PI/2);
            public static Pose2d HLstackPose1 = new Pose2d(-57,-14,Math.PI);
            public static Pose2d HLstackPose2 = new Pose2d(-57,-13,Math.PI);
            public static Pose2d HLstackPose3 = new Pose2d(-57,-12,Math.PI);
            public static Pose2d HLstackPose4 = new Pose2d(-57,-11,Math.PI);
            public static Pose2d HLstackPose5 = new Pose2d(-58,-10,Math.PI);
            public static Pose2d HLstackReadyPose = new Pose2d(-53,-13,Math.PI);
            public static Pose2d HLcone5Pose = new Pose2d(-57.5,-11,Math.PI);
            public static Pose2d HLdropPose = new Pose2d(-37,-12,Math.toRadians(213));
            public static Pose2d HLdropreadyPose = new Pose2d(-45,-12.5,Math.toRadians(200));
            public static Pose2d HLreadyPose = new Pose2d(-36,-14,3*Math.PI/2);
            public static Pose2d HLparkingPose1 = new Pose2d(-63,-12,Math.PI);
            public static Pose2d HLparkingPose2 = new Pose2d(-39,-12,Math.PI);
            public static Pose2d HLparkingPose3 = new Pose2d(-12,-12,Math.PI);

    //Vuforia
        public static float MIN_CONFIDENCE = .65f;
        public static double IMAGE_TIME = 1;

    //April Tags
        public static double TAG_SIZE = .05;
}
