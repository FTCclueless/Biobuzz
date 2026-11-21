package org.firstinspires.ftc.teamcode.utils.priority;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Test;

import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

public class AsyncWriterTest {
    @After
    public void off() { AsyncWriter.reset(false); }

    @Test
    public void onlyTheNewestValuePerDeviceIsSent() throws Exception {
        Object busy = new Object(), shooter = new Object();
        AsyncWriter.rs485Detector = d -> true;
        AsyncWriter.reset(true);

        CountDownLatch release = new CountDownLatch(1), done = new CountDownLatch(1);
        List<Integer> sent = new CopyOnWriteArrayList<>();
        AsyncWriter.submit(busy, () -> { try { release.await(); } catch (InterruptedException ignored) {} });
        Thread.sleep(50); // writer is now stuck on "busy", like a slow ack
        for (int v = 1; v <= 3; v++) {
            int value = v;
            AsyncWriter.submit(shooter, () -> { sent.add(value); done.countDown(); });
        }
        release.countDown();

        assertTrue(done.await(1, TimeUnit.SECONDS));
        Thread.sleep(50);
        assertEquals(List.of(3), sent);
    }

    @Test
    public void inlineWriteCancelsAnOlderQueuedOne() throws Exception {
        Object busy = new Object(), servo = new Object();
        AsyncWriter.rs485Detector = d -> d == busy;
        AsyncWriter.reset(true);

        CountDownLatch release = new CountDownLatch(1);
        List<String> sent = new CopyOnWriteArrayList<>();
        AsyncWriter.submit(busy, () -> { try { release.await(); } catch (InterruptedException ignored) {} });
        Thread.sleep(50);
        // Detection caches per device; "servo" is inline, so a stale queued value must not exist for it.
        AsyncWriter.submit(servo, () -> sent.add("inline"));
        release.countDown();
        Thread.sleep(100);
        assertEquals(List.of("inline"), sent);
        assertEquals(0, AsyncWriter.pendingCount());
    }

    @Test
    public void disabledWritesRunOnTheCallerThread() {
        AsyncWriter.rs485Detector = d -> true;
        AsyncWriter.reset(false);
        Thread[] ranOn = new Thread[1];
        AsyncWriter.submit(new Object(), () -> ranOn[0] = Thread.currentThread());
        assertEquals(Thread.currentThread(), ranOn[0]);
    }
}
