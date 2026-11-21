package org.firstinspires.ftc.teamcode.utils.priority;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.photon.ReflectionUtils;

import java.lang.reflect.Field;
import java.util.IdentityHashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Sends motor and servo writes for the RS485 hubs (Expansion Hub, Servo Hub) from one background
 * thread, so the loop doesn't wait for each ack. On the wire nothing changes: each write takes the
 * SDK's bus lock and waits for its ack, one at a time, exactly like a stock write. Only the most
 * recent value per device is kept; an older unsent one is replaced, never queued behind.
 *
 * <p>Stop safety comes from the SDK: LynxModule.sendCommand drops motor/servo commands once the
 * OpMode is stopping, under the same bus lock the stop failsafe takes, so a write from here can't
 * land after the failsafe. Control Hub devices are written inline (Photon already makes those fast).</p>
 */
public final class AsyncWriter {
    private static final LinkedHashMap<Object, Runnable> pending = new LinkedHashMap<>();
    private static final Map<Object, Boolean> onRs485 = new IdentityHashMap<>();
    private static volatile boolean enabled = false;
    private static Thread thread;

    /** Average time one RS485 write takes on the writer thread (ms). */
    public static volatile double writeMillis = 0;

    private AsyncWriter() {}

    /** Robot calls this once per OpMode: drops writes left over from the last one. */
    public static void reset(boolean enable) {
        synchronized (pending) {
            pending.clear();
            enabled = enable;
        }
    }

    public static int pendingCount() {
        synchronized (pending) { return pending.size(); }
    }

    public static void setPower(DcMotor motor, double power) { submit(motor, () -> motor.setPower(power)); }

    public static void setPower(CRServo servo, double power) { submit(servo, () -> servo.setPower(power)); }

    public static void setPosition(Servo servo, double position) { submit(servo, () -> servo.setPosition(position)); }

    /** Every write to {@code device} must come through here, or an older queued value could land after it. */
    public static void submit(Object device, Runnable write) {
        synchronized (pending) {
            if (enabled && isOnRs485(device)) {
                pending.put(device, write);
                pending.notifyAll();
                startThread();
                return;
            }
            pending.remove(device); // keep order if a device ever switches path
        }
        write.run();
    }

    /** Replaced by tests, which have no hubs. */
    static java.util.function.Predicate<Object> rs485Detector = AsyncWriter::detect;

    private static boolean isOnRs485(Object device) {
        Boolean cached = onRs485.get(device);
        if (cached == null) {
            cached = rs485Detector.test(device);
            onRs485.put(device, cached);
        }
        return cached;
    }

    /** A child module is one reached over RS485. Anything unrecognised stays inline. */
    private static boolean detect(Object device) {
        Object controller = device instanceof DcMotor ? ((DcMotor) device).getController()
                : device instanceof Servo ? ((Servo) device).getController()
                : device instanceof CRServo ? ((CRServo) device).getController()
                : null;
        if (controller == null) return false;
        Field f = ReflectionUtils.getField(controller.getClass(), LynxModule.class);
        if (f == null) return false;
        try {
            LynxModule module = (LynxModule) f.get(controller);
            return module != null && !module.isParent();
        } catch (IllegalAccessException e) {
            return false;
        }
    }

    private static void startThread() {
        if (thread != null) return;
        thread = new Thread(AsyncWriter::run, "AsyncWriter");
        thread.setDaemon(true);
        thread.start();
    }

    private static void run() {
        while (true) {
            Runnable write;
            try {
                synchronized (pending) {
                    while (pending.isEmpty()) pending.wait();
                    Iterator<Runnable> it = pending.values().iterator();
                    write = it.next();
                    it.remove();
                }
            } catch (InterruptedException e) {
                continue; // nothing else owns this thread; keep serving
            }
            if (OpModeManagerImpl.shouldPreventDangerousHardwareAccess()) continue;
            long start = System.nanoTime();
            try {
                write.run();
            } catch (RuntimeException e) {
                RobotLog.ee("AsyncWriter", e, "write failed");
            }
            writeMillis = writeMillis * 0.9 + (System.nanoTime() - start) / 1e6 * 0.1;
        }
    }
}
