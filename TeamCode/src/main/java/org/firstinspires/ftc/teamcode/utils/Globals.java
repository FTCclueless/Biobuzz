package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;

@Config
public class Globals {
    public static long LOOP_START = System.nanoTime();
    public static double LOOP_TIME = 0.0;
    public static RunMode RUNMODE = RunMode.TESTER;
    public static boolean TESTING_DISABLE_CONTROL = true;
    public static boolean isRed = true;
    public static long autoStartTime = -1;
    public static boolean fullField = false;

    public static boolean DRIVETRAIN_ENABLED = true;
    public static double TRACK_WIDTH = 11.27;
    public static double ROBOT_WIDTH = 16.04;
    public static double ROBOT_LENGTH = 18.0;
    public static double ROBOT_BACK_LENGTH = 6.2;
    public static double ROBOT_FORWARD_LENGTH = 7.4;
    public static Pose2d ROBOT_POSITION = new Pose2d(0,0,0);
    public static Pose2d ROBOT_VELOCITY = new Pose2d(0,0,0);
    public static Pose2d ROBOT_GLOBAL_VELOCITY = new Pose2d(0,0,0);
    public static Pose2d ROBOT_GLOBAL_ACCELERATION = new Pose2d(0,0,0);
    public static Pose2d AUTO_ENDING_POSE = new Pose2d(0,0,0);

    public static void START_LOOP() {
        LOOP_START = System.nanoTime();
    }

    public static double GET_LOOP_TIME() {
        LOOP_TIME = (System.nanoTime() - LOOP_START) / 1.0e9;
        return LOOP_TIME;
    }
}
