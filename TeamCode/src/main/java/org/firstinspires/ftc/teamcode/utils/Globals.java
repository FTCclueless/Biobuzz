package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import java.util.ArrayList;

@Config
public class Globals {
    // general
    public static long LOOP_START = System.nanoTime();
    public static double LOOP_TIME = 0.0;
    public static boolean isRed = true;
    public static RunMode RUNMODE = RunMode.TESTER;
    public static boolean TESTING_DISABLE_CONTROL = true;



    // drivetrain
    public static boolean DRIVETRAIN_ENABLED = true;
    public static double TRACK_WIDTH = 11.27;
    public static double ROBOT_WIDTH = 16.04;
    public static double ROBOT_LENGTH = 18.0;
    public static double ROBOT_BACK_LENGTH = 6.2;
    public static double ROBOT_FORWARD_LENGTH = 7.4;
    public static Pose ROBOT_POSITION = new Pose(0,0,0);
    public static Pose ROBOT_VELOCITY = new Pose(0,0,0);
    public static Pose ROBOT_GLOBAL_VELOCITY = new Pose(0,0,0);
    public static Pose ROBOT_GLOBAL_ACCELERATION = new Pose(0,0,0);
    public static Pose AUTO_ENDING_POSE = new Pose(0,0,0);

    // loop time methods
    public static void START_LOOP() {
        LOOP_START = System.nanoTime();
    }

    public static double GET_LOOP_TIME() {
        LOOP_TIME = (System.nanoTime() - LOOP_START) / 1.0e9; // converts from nano secs to secs
        return LOOP_TIME;
    }
}