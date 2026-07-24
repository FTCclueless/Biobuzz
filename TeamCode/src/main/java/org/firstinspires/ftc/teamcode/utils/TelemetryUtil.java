package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class TelemetryUtil {
    public static FtcDashboard dashboard;
    public static TelemetryPacket packet = new TelemetryPacket();

    public static void setup() {
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(100);
        //DashboardUtil.drawField();
    }

    public static TelemetryPacket getPacket() {
        return packet;
    }

    public static void sendTelemetry() {
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
        //DashboardUtil.drawField();
    }
}