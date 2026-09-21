package org.firstinspires.ftc.teamcode.photon;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.LynxMessage;
import com.qualcomm.hardware.lynx.commands.LynxRespondable;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;

import java.util.Collections;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Drop-in replacement for {@link LynxModule} that lets {@link PhotonCore} write motor and servo
 * commands straight to the USB device instead of waiting for each one to be acknowledged.
 *
 * <p>You never construct this yourself. {@link PhotonCore#onOpModePreInit} builds one per hub,
 * transplants the original module's state onto it, and swaps it into the hardware map before your
 * op-mode's {@code init()} runs.</p>
 */
public class PhotonLynxModule extends LynxModule {
    /**
     * Reusable ACK for parallelized, fire-and-forget writes.
     *
     * <p>{@code LynxRespondable.onAckReceived} only reads {@link LynxAck#isAttentionRequired()}
     * (always {@code false} here) and never retains the object, so one shared immutable instance per
     * module is safe. It saves an allocation on every single motor/servo write — at eight motors and
     * six servos per loop that is well over a thousand short-lived objects per second, which is
     * exactly the kind of garbage that shows up as periodic loop-time spikes on a Control Hub.</p>
     */
    private final LynxAck immediateAck = new LynxAck(this, false);

    /**
     * Messages whose network-transmission lock we intentionally skipped, so the matching release
     * skips too.
     *
     * <p>Backed by a {@link ConcurrentHashMap} because acquire and release can be invoked from
     * different threads, and keyed by object identity — O(1) contains/remove instead of the previous
     * {@code ArrayList}'s O(n) scan, and thread-safe where the {@code ArrayList} was not.</p>
     *
     * <p>Tracking the set rather than re-evaluating the predicate in
     * {@link #releaseNetworkTransmissionLock} is deliberate: {@link PhotonCore#isEnabled()} can flip
     * between the two calls, and releasing a lock we never acquired makes {@code MessageKeyedLock}
     * log an error and leaves its owner bookkeeping wrong.</p>
     */
    private final Set<LynxMessage> skippedAcquire = Collections.newSetFromMap(new ConcurrentHashMap<>());

    /**
     * The FTDI device this hub's datagrams go out on, resolved once at init.
     *
     * <p>Held per-module so {@link PhotonCore#registerSend} is a plain field read instead of a hash
     * lookup in a shared map on every write.</p>
     */
    private volatile RobotUsbDevice photonUsbDevice;

    /**
     * The monitor {@code LynxUsbDeviceImpl.transmit} holds around its {@code robotUsbDevice.write}
     * (its {@code engageLock}). Photon must hold the same one, or a Photon write can interleave its
     * bytes with an SDK write and corrupt the datagram stream.
     */
    private volatile Object photonSyncLock;

    public PhotonLynxModule(LynxUsbDevice lynxUsbDevice, int moduleAddress, boolean isParent, boolean isUserModule) {
        super(lynxUsbDevice, moduleAddress, isParent, isUserModule);
    }

    /**
     * Exposes the inherited protected {@code lynxUsbDevice} to {@link PhotonCore}, which lives in
     * this package but is not a subclass. Saves a reflective field lookup at init and, more
     * importantly, keeps working if the SDK renames the field.
     */
    LynxUsbDevice getLynxUsbDevice() {
        return this.lynxUsbDevice;
    }

    /**
     * Binds this module to the transport {@link PhotonCore} will write through. Until this is
     * called, {@link PhotonCore#registerSend} declines the module and everything falls through to
     * the stock SDK path.
     */
    void attachPhotonTransport(RobotUsbDevice usbDevice, Object syncLock) {
        this.photonUsbDevice = usbDevice;
        this.photonSyncLock = syncLock;
    }

    RobotUsbDevice getPhotonUsbDevice() {
        return photonUsbDevice;
    }

    Object getPhotonSyncLock() {
        return photonSyncLock;
    }

    public ConcurrentHashMap<Integer, LynxRespondable> getUnfinishedCommands() {
        return this.unfinishedCommands;
    }

    public LynxAck getImmediateAck() {
        return immediateAck;
    }

    /**
     * Re-declared purely for visibility. {@code LynxModule.getNewMessageNumber} is {@code protected}
     * in {@code com.qualcomm.hardware.lynx}; re-declaring it here makes it package-accessible to
     * {@link PhotonCore}, which is not a subclass. Do not delete — {@code PhotonCore} will not
     * compile without it.
     */
    @Override
    protected byte getNewMessageNumber() {
        return super.getNewMessageNumber();
    }

    @Override
    public void sendCommand(LynxMessage command) throws InterruptedException, LynxUnsupportedCommandException {
        if (PhotonCore.isPhotonPath(command)) {
            if (PhotonCore.registerSend((LynxCommand<?>) command)) {
                return;
            }
            // registerSend declined (module not bound, queue saturated, dangerous access blocked, or
            // a USB write error). Hand the command back to the SDK, which knows how to nack it,
            // mark the device as having shut down abnormally, and raise the global warning.
        }
        super.sendCommand(command);
    }

    @Override
    public void acquireNetworkTransmissionLock(LynxMessage message) throws InterruptedException {
        if (PhotonCore.isPhotonPath(message)) {
            skippedAcquire.add(message);
            return;
        }
        super.acquireNetworkTransmissionLock(message);
    }

    @Override
    public void releaseNetworkTransmissionLock(LynxMessage message) throws InterruptedException {
        // Keyed off what we actually did at acquire time, not off the predicate, so a mid-message
        // enable()/disable() cannot desynchronize the pair.
        if (skippedAcquire.remove(message)) {
            return;
        }
        super.releaseNetworkTransmissionLock(message);
    }
}
