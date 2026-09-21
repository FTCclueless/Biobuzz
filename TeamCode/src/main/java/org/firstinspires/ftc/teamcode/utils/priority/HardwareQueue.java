package org.firstinspires.ftc.teamcode.utils.priority;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;

@Config
public class HardwareQueue {
    public ArrayList<PriorityDevice> devices = new ArrayList<>();

    public static double targetLoopLength = 0.016;

    public static int lastCriticalWrites = 0;
    public static int lastRankedWrites = 0;
    public static int lastDeferred = 0;

    public PriorityDevice getDevice(String name){
        for (PriorityDevice device : devices){
            if (device.name.equals(name)){
                return device;
            }
        }
        return null;
    }

    public void addDevice(PriorityDevice device) {
        devices.add(device);
    }

    public void addDevices(PriorityDevice... devices) {
        for (PriorityDevice device : devices) {
            this.addDevice(device);
        }
    }

    public void update() {
        int n = devices.size();

        for (int i = 0; i < n; i++) {
            PriorityDevice d = devices.get(i);
            d.resetUpdateBoolean();
            d.advanceModel();
        }

        int criticalWrites = 0;
        for (int i = 0; i < n; i++) {
            PriorityDevice d = devices.get(i);
            if (d.isCritical() && d.needsCriticalWrite()) {
                d.update();
                criticalWrites++;
            }
        }

        int rankedWrites = 0;
        double bestPriority;
        double loopTime = GET_LOOP_TIME();
        do {
            int bestIndex = -1;
            bestPriority = 0;
            double remaining = targetLoopLength - loopTime;

            for (int i = 0; i < n; i++) {
                PriorityDevice d = devices.get(i);
                if (d.isCritical()) continue;
                double p = d.getPriority(remaining);
                if (p > bestPriority) {
                    bestPriority = p;
                    bestIndex = i;
                }
            }

            if (bestIndex >= 0) {
                devices.get(bestIndex).update();
                rankedWrites++;
            }
            loopTime = GET_LOOP_TIME();
        } while (bestPriority > 0 && loopTime <= targetLoopLength);

        int deferred = 0;
        for (int i = 0; i < n; i++) {
            PriorityDevice d = devices.get(i);
            if (!d.isCritical() && d.hasPendingWrite()) deferred++;
        }

        lastCriticalWrites = criticalWrites;
        lastRankedWrites = rankedWrites;
        lastDeferred = deferred;
    }
}
