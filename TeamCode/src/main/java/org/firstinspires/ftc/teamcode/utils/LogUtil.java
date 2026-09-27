package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

@Config
public class LogUtil {
    private static Datalogger datalogger = null;

    public static class StateField extends Datalogger.LoggableField {
        private String value = "";

        public StateField(String name) { super(name); }

        @Override
        public void writeToBuffer(StringBuilder out) { out.append(value); }

        public void set(String newValue) {
            if (!newValue.equals(value)) {
                //TelemetryUtil.packet.put("LogUtil : stateTransition", name + "," + value);
                LogUtil.stateTransition = true;
            }
            value = newValue;
        }

        @NonNull
        @Override
        public String toString() { return value; }
    }

    public static class EventField extends Datalogger.LoggableField {
        private String value = "";

        public EventField(String name) { super(name); }

        @Override
        public void writeToBuffer(StringBuilder out) { out.append(value); }

        public void set(String newValue) {
            value = newValue;
        }

        public void add(String newValue) {
            if (value.isEmpty()) value = newValue;
            else value += " " + newValue;
        }

        @NonNull
        @Override
        public String toString() { return value; }
    }

    // These are all of the fields that we want in the datalog.
    // Note that order here is NOT important. The order is important in the setFields() call below
    //public static Datalogger.GenericField loopTime = new Datalogger.GenericField("loopTime");
    public static Datalogger.GenericField driveState = new Datalogger.GenericField("driveState");
    public static Datalogger.GenericField driveCurrentX = new Datalogger.GenericField("driveCurrentX");
    public static Datalogger.GenericField driveCurrentY = new Datalogger.GenericField("driveCurrentY");
    public static Datalogger.GenericField driveCurrentAngle = new Datalogger.GenericField("driveCurrentAngle");
    //public static Datalogger.GenericField driveTargetX = new Datalogger.GenericField("driveTargetX");
    //public static Datalogger.GenericField driveTargetY = new Datalogger.GenericField("driveTargetY");
    //public static Datalogger.GenericField driveTargetAngle = new Datalogger.GenericField("driveTargetAngle");
    public static Datalogger.GenericField drivePath = new Datalogger.GenericField("drivePath");
    public static EventField event = new EventField("event");

    private static long timeAtNextWrite;

    public static boolean DISABLED = true;
    public static boolean stateTransition = false;
    public static boolean drivePositionReset = false;

    public static void reset() {
        if (datalogger != null) {
            datalogger = null;
        }
    }

    public static void init() {
        timeAtNextWrite = 0;

        long timeNow = System.currentTimeMillis();
        String fileName = "Log_" + timeNow + "_"
            + new SimpleDateFormat("yyyy-MM-dd_HH.mm.ss", Locale.US).format(new Date(timeNow))
            + "_Offseason_" + Globals.RUNMODE.toString();
        //TelemetryUtil.packet.put("LogUtil : filename", fileName);

        if (datalogger != null) throw new IllegalStateException("LogUtil was already initialized");
        if (DISABLED) return;

        datalogger = new Datalogger.Builder()
            // Pass through the filename
            .setFilename(fileName)
            // Request an automatic timestamp field
            .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)
            // Tell it about the fields we care to log.
            // Note that order *IS* important here! The order in which we list
            // the fields is the order in which they will appear in the log.
            .setFields(
                driveCurrentX,
                driveCurrentY,
                driveCurrentAngle,
                drivePath,
                driveState,
                event
            )
            .build();
    }

    public static void send() {
        if (datalogger == null) return;
        long timeNow = System.nanoTime();
        if (timeNow >= timeAtNextWrite || stateTransition || drivePositionReset) {
            if (drivePositionReset) driveState.set("[reset]");
            datalogger.writeLine();
            timeAtNextWrite = timeNow + 200_000_000;
            stateTransition = false;
            drivePositionReset = false;
            event.set("");
            TelemetryUtil.packet.put("LogUtil : stateTransition", "[ none ]");
        }
    }
}
