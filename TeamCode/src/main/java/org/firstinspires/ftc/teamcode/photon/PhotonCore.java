package org.firstinspires.ftc.teamcode.photon;

import android.content.Context;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.LynxUsbDeviceImpl;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.LynxDatagram;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.LynxRespondable;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetServoPulseWidthCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.robotcore.internal.usb.exception.RobotUsbException;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Cuts loop time by writing motor-power and servo-pulse commands straight to the hub's USB device
 * instead of blocking the caller until each one is acknowledged.
 *
 * <h2>How it attaches</h2>
 * <p>You do <b>not</b> create any Photon object yourself, and there is nothing to create per motor.
 * {@link #attachEventLoop} is invoked by the SDK via {@code @OnCreateEventLoop}, which registers
 * this singleton as an op-mode listener. On every op-mode init, {@link #onOpModePreInit} walks the
 * hardware map, replaces each {@link LynxModule} with a {@link PhotonLynxModule}, and re-points
 * every motor, servo, and I2C device at the replacement. Your {@code hardwareMap.get(...)} calls are
 * unchanged.</p>
 *
 * <p>The only thing your op-mode does is switch it on:</p>
 * <pre>{@code
 * @TeleOp
 * public class MyTeleOp extends LinearOpMode {
 *     @Override public void runOpMode() {
 *         PhotonCore.enable();
 *         DcMotorEx left = hardwareMap.get(DcMotorEx.class, "left");   // ordinary lookup
 *         waitForStart();
 *         while (opModeIsActive()) {
 *             left.setPower(...);
 *         }
 *     }
 * }
 * }</pre>
 *
 * <h2>What it deliberately keeps from the SDK</h2>
 * <p>The fast path is not a free-for-all. It still holds the same monitor
 * {@code LynxUsbDeviceImpl.transmit} holds around its write, it still registers the command in
 * {@code unfinishedCommands} so the real ack from the hub resolves normally, and it still honours
 * {@link OpModeManagerImpl#shouldPreventDangerousHardwareAccess()} so motors and servos cannot be
 * driven while the SDK is tearing an op-mode down.</p>
 */
public class PhotonCore implements Runnable, OpModeManagerNotifier.Notifications {
    private static final String TAG = "PhotonCore";

    protected static final PhotonCore instance = new PhotonCore();
    protected AtomicBoolean enabled, threadEnabled;

    private OpModeManagerImpl opModeManager;

    /** Set by {@link #onOpModePreInit}; the hubs Photon is driving. */
    public static LynxModule CONTROL_HUB, EXPANSION_HUB;

    /**
     * Whether servo pulse-width commands take the fast path too. Motor power always does.
     */
    public static boolean PARALLELIZE_SERVOS = true;

    /**
     * Bulk-read caching mode applied to each hub in {@link #enable()} (only when the hub is
     * otherwise {@code OFF}, so an explicit choice you made earlier is never overridden).
     *
     * <p>{@code AUTO} (the default) is safe and needs no per-loop management, but issues a second
     * bulk read if the same input is read twice within a loop. {@code MANUAL} guarantees exactly one
     * bulk read per hub per loop for the most deterministic loop time — but <b>requires</b> calling
     * {@link #clearBulkCache()} once at the top of every loop, otherwise reads return stale, frozen
     * values.</p>
     */
    public static LynxModule.BulkCachingMode BULK_CACHING_MODE = LynxModule.BulkCachingMode.AUTO;

    public static class ExperimentalParameters {
        /**
         * @deprecated Inert. This used to select a second code path that, before writing, scanned
         * every outstanding command for one with the same destination and command number and
         * <em>busy-spun</em> until none matched. That scan could not tell two motors apart — all
         * four motor-power commands on a hub share one command number — so it stalled on unrelated
         * writes, and it burned a core doing it. The write path is now the same either way.
         */
        @Deprecated
        private final AtomicBoolean singlethreadedOptimized = new AtomicBoolean(true);

        /**
         * Ceiling on how many commands may be outstanding on a hub before Photon stops taking the
         * fast path and lets the SDK's blocking path handle the write.
         *
         * <p>This is a safety valve, not a throttle. A hub acknowledges within about a millisecond,
         * so a robot writing fourteen actuators at 100 Hz sits at one to three outstanding commands
         * — nowhere near the limit. It only trips when acks genuinely stop coming back, which is
         * exactly when you want the SDK's retransmit-and-nack machinery instead of Photon.</p>
         *
         * <p>It also bounds a real hazard: {@code LynxModule.getNewMessageNumber()} loops until it
         * finds a message number not already in {@code unfinishedCommands}. Message numbers are a
         * single byte, so letting outstanding commands approach 255 turns that loop into a hang.
         * The cap is clamped below that.</p>
         */
        private final AtomicInteger maximumParallelCommands = new AtomicInteger(64);

        /** Hard ceiling — {@code getNewMessageNumber()} livelocks as the byte-wide space fills. */
        private static final int MAX_OUTSTANDING = 200;

        /** @deprecated No longer has any effect. Kept so existing op-modes still compile. */
        @Deprecated
        public void setSinglethreadedOptimized(boolean state) {
            this.singlethreadedOptimized.set(state);
        }

        /** @deprecated No longer has any effect. */
        @Deprecated
        public boolean getSinglethreadedOptimized() {
            return this.singlethreadedOptimized.get();
        }

        public boolean setMaximumParallelCommands(int maximumParallelCommands) {
            if (maximumParallelCommands <= 0) {
                return false;
            }
            this.maximumParallelCommands.set(Math.min(maximumParallelCommands, MAX_OUTSTANDING));
            return true;
        }

        public int getMaximumParallelCommands() {
            return this.maximumParallelCommands.get();
        }
    }

    public static ExperimentalParameters experimental = new ExperimentalParameters();

    public PhotonCore() {
        CONTROL_HUB = null;
        EXPANSION_HUB = null;
        enabled = new AtomicBoolean(false);
        threadEnabled = new AtomicBoolean(false);
    }

    public static void enable() {
        instance.enabled.set(true);
        applyBulkCachingMode(CONTROL_HUB);
        applyBulkCachingMode(EXPANSION_HUB);
    }

    public static void disable() {
        instance.enabled.set(false);
    }

    public static AtomicBoolean isEnabled() {
        return instance.enabled;
    }

    private static void applyBulkCachingMode(LynxModule hub) {
        if (hub != null && hub.getBulkCachingMode() == LynxModule.BulkCachingMode.OFF) {
            hub.setBulkCachingMode(BULK_CACHING_MODE);
        }
    }

    /**
     * Clears the bulk-read cache on both hubs so the next read of each hub issues a fresh bulk read.
     *
     * <p>Call this once at the very top of your loop when using {@link #BULK_CACHING_MODE} =
     * {@code MANUAL}, to get exactly one bulk read per hub per loop. Harmless in {@code AUTO} mode.
     * No-op for any hub that isn't present.</p>
     */
    public static void clearBulkCache() {
        if (CONTROL_HUB != null) {
            CONTROL_HUB.clearBulkCache();
        }
        if (EXPANSION_HUB != null) {
            EXPANSION_HUB.clearBulkCache();
        }
    }

    @OnCreateEventLoop
    public static void attachEventLoop(Context context, FtcEventLoop eventLoop) {
        eventLoop.getOpModeManager().registerListener(instance);
        instance.opModeManager = eventLoop.getOpModeManager();
    }

    // ---------------------------------------------------------------------------------------
    // Hot path
    // ---------------------------------------------------------------------------------------

    /**
     * Single gate for "should this message bypass the SDK's blocking send?", used by both
     * {@link PhotonLynxModule#sendCommand} and
     * {@link PhotonLynxModule#acquireNetworkTransmissionLock} so the two can never disagree.
     *
     * <p>The {@code shouldPreventDangerousHardwareAccess} term matters for more than safety. That
     * flag is what the SDK sets when it force-stops a runaway op-mode; the mechanism it relies on is
     * {@code MessageKeyedLock.acquire} throwing {@code ForceStopException} to unwind the op-mode
     * thread. Skipping the lock unconditionally — as this used to — makes an op-mode whose loop only
     * writes motor powers immune to that unwind. Deferring to the real lock while the flag is set
     * restores it, and costs one plain volatile read when it isn't.</p>
     */
    static boolean isPhotonPath(LynxMessage message) {
        return instance.enabled.get()
                && message instanceof LynxCommand
                && shouldParallelize((LynxCommand<?>) message)
                && !OpModeManagerImpl.shouldPreventDangerousHardwareAccess();
    }

    protected static boolean shouldParallelize(LynxCommand<?> command) {
        return command instanceof LynxSetMotorConstantPowerCommand
                || (PARALLELIZE_SERVOS && command instanceof LynxSetServoPulseWidthCommand);
    }

    /**
     * Every command that reaches {@link #registerSend} is one whose ack carries no data we need, so
     * the ack is synthesized locally the moment the bytes are on the wire. Kept as a named predicate
     * because it documents the invariant that {@link #shouldParallelize} is a subset of it.
     */
    protected static boolean shouldAckImmediately(LynxCommand<?> command) {
        return command instanceof LynxSetMotorConstantPowerCommand
                || command instanceof LynxSetServoPulseWidthCommand;
    }

    /**
     * Serializes {@code command} and writes it directly to the hub's USB device, then synthesizes
     * its ack so the caller never blocks.
     *
     * @return {@code true} if the command was written; {@code false} if the caller should fall back
     * to {@code super.sendCommand}, which happens when the module has no transport bound, when too
     * many commands are already outstanding, or when the USB write failed.
     * @throws LynxUnsupportedCommandException if the datagram cannot be built. Propagated rather
     *                                         than swallowed so {@code LynxRespondable.send()} turns
     *                                         it into a nack immediately — swallowing it left the
     *                                         command with no ack and no nack, stalling the caller
     *                                         until the SDK's multi-second retransmit timeout fired.
     */
    protected static boolean registerSend(LynxCommand<?> command)
            throws LynxUnsupportedCommandException, InterruptedException {

        PhotonLynxModule module = (PhotonLynxModule) command.getModule();

        // Plain field reads on the module itself. This used to be a HashMap lookup keyed by the
        // module, on every write, from a map that was also being mutated during init.
        final RobotUsbDevice usbDevice = module.getPhotonUsbDevice();
        final Object syncLock = module.getPhotonSyncLock();
        if (usbDevice == null || syncLock == null) {
            return false;
        }

        final ConcurrentHashMap<Integer, LynxRespondable> unfinished = module.getUnfinishedCommands();
        if (unfinished.size() >= experimental.getMaximumParallelCommands()) {
            // Acks have stopped coming back. Let the SDK's blocking path deal with it rather than
            // piling on — and never let the message-number space fill (see MAX_OUTSTANDING).
            return false;
        }

        // Note: identical repeated motor/servo values are already suppressed upstream by the SDK
        // controllers (LynxDcMotorController.lastKnownPower / LynxServoController
        // .lastKnownCommandedPosition), so no payload-level write cache is kept here.

        // One monitor, not two. syncLock is the same object LynxUsbDeviceImpl.transmit synchronizes
        // on, so holding it for the whole critical section gives mutual exclusion with the SDK's
        // writes *and* keeps Photon's own writes ordered — which the previous separate messageSync
        // was there to do. Message-number assignment lives inside it so a number cannot be handed
        // out and then written out of order.
        boolean written = false;
        synchronized (syncLock) {
            command.setMessageNumber(module.getNewMessageNumber());
            final Integer key = command.getMessageNumber();

            // Registered before the write, and before anything can fail, so the hub's real ack
            // always finds the command and LynxModule.finishedWithMessage can retire it. Photon's
            // synthesized ack only releases the caller; it does not remove the entry.
            final boolean tracked = command.isAckable() || command.isResponseExpected();
            if (tracked) {
                unfinished.put(key, (LynxRespondable) command);
            }

            try {
                LynxDatagram datagram = new LynxDatagram(command);
                command.setSerialization(datagram);
                usbDevice.write(datagram.toByteArray());
                command.noteHasBeenTransmitted();
                written = true;
            } catch (LynxUnsupportedCommandException | InterruptedException e) {
                // Nothing will ever ack this command, so retire the entry here. Leaving it behind
                // would hold a message number out of a 255-wide space until op-mode teardown.
                if (tracked) {
                    unfinished.remove(key);
                }
                throw e;
            } catch (RobotUsbException e) {
                if (tracked) {
                    unfinished.remove(key);
                }
                command.forgetSerialization();
                RobotLog.ee(TAG, e, "USB write failed; falling back to the SDK send path");
            }
        }

        if (!written) {
            return false;
        }

        // Outside the lock on purpose: onAckReceived reaches into LynxModule.setAttentionRequired,
        // which takes the module's status monitor. No reason to hold the bus lock across that.
        //
        // Not calling resetModulePingTimer() here, unlike LynxUsbDeviceImpl.transmit: it cancels and
        // reschedules a Future on the module's executor, which is real allocation and real work on
        // every single actuator write. Bulk reads go through the normal path every loop and reset
        // the timer there, so keep-alives stay suppressed anyway.
        command.onAckReceived(module.getImmediateAck());
        return true;
    }

    // ---------------------------------------------------------------------------------------
    // Op-mode lifecycle
    // ---------------------------------------------------------------------------------------

    @Override
    public void onOpModePreInit(OpMode opMode) {
        if (opModeManager == null
                || OpModeManager.DEFAULT_OP_MODE_NAME.equals(opModeManager.getActiveOpModeName())) {
            return;
        }

        HardwareMap map = opMode.hardwareMap;

        boolean replacedPrev = false;
        boolean hasChub = false;
        for (LynxModule module : map.getAll(LynxModule.class)) {
            if (module instanceof PhotonLynxModule) {
                replacedPrev = true;
            }
            if (LynxConstants.isEmbeddedSerialNumber(module.getSerialNumber())) {
                hasChub = true;
            }
        }

        if (replacedPrev) {
            // A previous op-mode already swapped these in; drop any stock modules the SDK re-added.
            HashMap<String, HardwareDevice> toRemove = new HashMap<>();
            for (LynxModule module : map.getAll(LynxModule.class)) {
                if (!(module instanceof PhotonLynxModule)) {
                    toRemove.put(nameOf(map, module), module);
                }
            }
            for (Map.Entry<String, HardwareDevice> e : toRemove.entrySet()) {
                map.remove(e.getKey(), e.getValue());
            }
        } else {
            CONTROL_HUB = null;
            EXPANSION_HUB = null;
        }

        List<String> moduleNames = new ArrayList<>();
        for (LynxModule module : map.getAll(LynxModule.class)) {
            moduleNames.add(nameOf(map, module));
        }

        HashMap<LynxModule, PhotonLynxModule> replacements = new HashMap<>();
        for (String name : moduleNames) {
            LynxModule module = map.get(LynxModule.class, name);
            if (module instanceof PhotonLynxModule) {
                continue;
            }
            try {
                Field usbField = ReflectionUtils.getField(module.getClass(), "lynxUsbDevice");
                if (usbField == null) {
                    RobotLog.ee(TAG, "no lynxUsbDevice field on %s; leaving '%s' unaccelerated",
                            module.getClass().getName(), name);
                    continue;
                }

                PhotonLynxModule photon = new PhotonLynxModule(
                        (LynxUsbDevice) usbField.get(module),
                        module.getModuleAddress(),
                        module.isParent(),
                        module.isUserModule());

                RobotLog.vv(TAG, "replacing LynxModule '%s' (addr %d)", name, module.getModuleAddress());
                ReflectionUtils.deepCopy(module, photon);
                map.remove(name, module);
                map.put(name, photon);
                replacements.put(module, photon);

                // Bind the transport for *every* module, not just parents. An Expansion Hub daisy-
                // chained off a Control Hub is a child on the same LynxUsbDevice: previously it
                // never got an entry, so registerSend declined it and every expansion-hub motor and
                // servo silently fell back to the stock blocking path.
                attachTransport(photon);

                if (module.isParent() && hasChub && CONTROL_HUB == null
                        && LynxConstants.isEmbeddedSerialNumber(module.getSerialNumber())) {
                    CONTROL_HUB = photon;
                } else {
                    EXPANSION_HUB = photon;
                }
            } catch (IllegalAccessException e) {
                RobotLog.ee(TAG, e, "could not replace LynxModule '%s'", name);
            }
        }

        // Re-register the replacements under the USB device that owns them, so the incoming-datagram
        // poller resolves acks against the Photon module rather than the discarded original.
        for (Map.Entry<LynxModule, PhotonLynxModule> entry : replacements.entrySet()) {
            PhotonLynxModule photon = entry.getValue();
            LynxUsbDevice owner = photon.getLynxUsbDevice();
            if (owner == null) {
                continue;
            }
            LynxUsbDeviceImpl impl = owner.getDelegationTarget();
            if (impl == null) {
                continue;
            }
            impl.removeConfiguredModule(entry.getKey());
            try {
                Field knownModulesField = ReflectionUtils.getField(impl.getClass(), "knownModules");
                if (knownModulesField == null) {
                    RobotLog.ee(TAG, "no knownModules field on %s", impl.getClass().getName());
                    continue;
                }
                @SuppressWarnings("unchecked")
                ConcurrentHashMap<Integer, LynxModule> knownModules =
                        (ConcurrentHashMap<Integer, LynxModule>) knownModulesField.get(impl);
                synchronized (knownModules) {
                    knownModules.put(photon.getModuleAddress(), photon);
                }
                RobotLog.vv(TAG, "addConfiguredModule() name=%s", photon.getDeviceName());
            } catch (IllegalAccessException e) {
                RobotLog.ee(TAG, e, "could not re-register module %d", photon.getModuleAddress());
            }
        }

        // Every device caches the LynxModule it talks to. Re-point those caches at the replacements.
        for (HardwareDevice device : map.getAll(HardwareDevice.class)) {
            if (device instanceof LynxModule) {
                continue;
            }
            if (device instanceof I2cDeviceSynchDevice || device instanceof I2cDeviceSynchSimple) {
                try {
                    Field clientField = ReflectionUtils.getField(device.getClass(), "deviceClient");
                    if (clientField == null) {
                        setLynxObject(device, replacements);
                        continue;
                    }
                    I2cDeviceSynchSimple client = (I2cDeviceSynchSimple) clientField.get(device);
                    if (client == null) {
                        continue;
                    }
                    if (!(client instanceof LynxI2cDeviceSynch)) {
                        Field innerField = ReflectionUtils.getField(client.getClass(), "i2cDeviceSynchSimple");
                        if (innerField != null) {
                            Object inner = innerField.get(client);
                            if (inner instanceof I2cDeviceSynchSimple) {
                                client = (I2cDeviceSynchSimple) inner;
                            }
                        }
                    }
                    setLynxObject(client, replacements);
                } catch (IllegalAccessException | ClassCastException e) {
                    RobotLog.vv(TAG, "could not re-point I2C device %s: %s",
                            device.getClass().getSimpleName(), e.getMessage());
                }
            } else {
                setLynxObject(device, replacements);
            }
        }
    }

    /**
     * Resolves the FTDI device and bus monitor for {@code photon} and binds them to it.
     *
     * <p>{@code getDelegationTarget()} and {@code getRobotUsbDevice()} are both public on the
     * {@link LynxUsbDevice} interface, so the only reflection left is {@code engageLock} — the
     * monitor {@code LynxUsbDeviceImpl.transmit} holds around its write. If that lookup ever fails,
     * the module stays unbound and everything falls back to the stock path rather than writing
     * without mutual exclusion and shredding the datagram stream.</p>
     */
    private static void attachTransport(PhotonLynxModule photon) {
        LynxUsbDevice usb = photon.getLynxUsbDevice();
        if (usb == null) {
            return;
        }
        LynxUsbDeviceImpl impl = usb.getDelegationTarget();
        if (impl == null) {
            return;
        }
        RobotUsbDevice robotUsbDevice = impl.getRobotUsbDevice();
        if (robotUsbDevice == null) {
            RobotLog.vv(TAG, "module %d has no RobotUsbDevice yet; not accelerating",
                    photon.getModuleAddress());
            return;
        }
        Field engageLockField = ReflectionUtils.getField(impl.getClass(), "engageLock");
        if (engageLockField == null) {
            RobotLog.ee(TAG, "no engageLock on %s; not accelerating module %d",
                    impl.getClass().getName(), photon.getModuleAddress());
            return;
        }
        try {
            Object engageLock = engageLockField.get(impl);
            if (engageLock == null) {
                return;
            }
            photon.attachPhotonTransport(robotUsbDevice, engageLock);
        } catch (IllegalAccessException e) {
            RobotLog.ee(TAG, e, "could not read engageLock; not accelerating module %d",
                    photon.getModuleAddress());
        }
    }

    private static String nameOf(HardwareMap map, HardwareDevice device) {
        return map.getNamesOf(device).iterator().next();
    }

    private void setLynxObject(Object device, HashMap<LynxModule, PhotonLynxModule> replacements) {
        Field f = ReflectionUtils.getField(device.getClass(), LynxModule.class);
        if (f == null) {
            return;
        }
        try {
            LynxModule module = (LynxModule) f.get(device);
            if (module == null) {
                return;
            }
            PhotonLynxModule replacement = replacements.get(module);
            if (replacement != null) {
                f.set(device, replacement);
            }
        } catch (IllegalAccessException e) {
            RobotLog.ee(TAG, e, "could not re-point %s at its Photon module",
                    device.getClass().getSimpleName());
        }
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {
    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        enabled.set(false);
        threadEnabled.set(false);
    }

    /**
     * Retained only because this class has always been {@link Runnable} and something downstream may
     * still reference it. Nothing starts it.
     *
     * <p>The background thread this used to run woke every 5 ms to check a flag and do nothing else.
     * On a Control Hub that is pure contention against the op-mode loop — two hundred pointless
     * wakeups a second on a device where the loop is already fighting for CPU.</p>
     */
    @Override
    public void run() {
    }
}
