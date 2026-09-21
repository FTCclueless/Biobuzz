package org.firstinspires.ftc.teamcode.photon;

import com.qualcomm.robotcore.util.RobotLog;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

/**
 * Small reflection helpers used to reach into SDK internals during op-mode init.
 *
 * <p>Nothing in here runs on the control loop's hot path — every call site is in
 * {@link PhotonCore#onOpModePreInit}. Correctness and robustness across SDK versions therefore
 * matter far more than speed, which is why the lookups walk the whole class hierarchy and return
 * {@code null} rather than throwing.</p>
 */
public final class ReflectionUtils {
    private static final String TAG = "PhotonReflection";

    private ReflectionUtils() {
    }

    /**
     * Finds a field by name on {@code clazz} or any of its superclasses.
     *
     * <p>Walks the hierarchy explicitly instead of recursing on a caught {@code
     * NoSuchFieldException}: filling in a throwable's stack trace is by far the most expensive part
     * of a failed lookup, and a miss on a deep class such as {@code LynxModule} used to build one
     * per level.</p>
     *
     * @return the accessible field, or {@code null} if no class in the hierarchy declares it
     */
    public static Field getField(Class<?> clazz, String fieldName) {
        for (Class<?> c = clazz; c != null; c = c.getSuperclass()) {
            for (Field f : c.getDeclaredFields()) {
                if (f.getName().equals(fieldName)) {
                    f.setAccessible(true);
                    return f;
                }
            }
        }
        return null;
    }

    /**
     * Finds the first field whose declared type is exactly {@code target}, searching {@code clazz}
     * then its superclasses.
     *
     * @return the accessible field, or {@code null} if no class in the hierarchy declares one
     */
    public static Field getField(Class<?> clazz, Class<?> target) {
        for (Class<?> c = clazz; c != null; c = c.getSuperclass()) {
            for (Field f : c.getDeclaredFields()) {
                if (f.getType() == target) {
                    f.setAccessible(true);
                    return f;
                }
            }
        }
        return null;
    }

    /**
     * Copies every instance field of {@code org} onto the identically-named field of {@code target}.
     *
     * <p>Used to graft a live {@code LynxModule}'s state onto its {@link PhotonLynxModule}
     * replacement. References are copied as-is (shallow), which is deliberate: the replacement must
     * share the original's {@code unfinishedCommands} map, monitors, and executor so that the SDK's
     * incoming-datagram poller keeps resolving acks against the same state.</p>
     *
     * <p>Two things this skips on purpose:</p>
     * <ul>
     *   <li><b>Static fields.</b> The previous version wrote every static it walked past — including
     *       {@code LynxModule.standardMessages} and {@code responseClasses} — back onto itself.
     *       Harmless in practice but pure waste, and a real hazard the moment a static's value
     *       differs between the two objects.</li>
     *   <li><b>Synthetic fields</b> (outer-class links, assertion flags) which are compiler
     *       bookkeeping and never valid to transplant.</li>
     * </ul>
     *
     * <p>It also walks {@code org}'s superclasses, which the previous version did not — it only
     * looked at {@code getDeclaredFields()} on the exact class, silently dropping anything inherited
     * (e.g. {@code LynxCommExceptionHandler.tag}).</p>
     */
    public static void deepCopy(Object org, Object target) {
        Class<?> targetClass = target.getClass();
        for (Class<?> c = org.getClass(); c != null && c != Object.class; c = c.getSuperclass()) {
            for (Field src : c.getDeclaredFields()) {
                int mods = src.getModifiers();
                if (Modifier.isStatic(mods) || src.isSynthetic()) {
                    continue;
                }
                Field dst = getField(targetClass, src.getName());
                if (dst == null || Modifier.isStatic(dst.getModifiers())) {
                    continue;
                }
                src.setAccessible(true);
                try {
                    dst.set(target, src.get(org));
                } catch (IllegalAccessException | IllegalArgumentException e) {
                    // A field we cannot transplant is not fatal — the replacement simply keeps the
                    // value its constructor gave it. Log it so an SDK change that starts rejecting
                    // a load-bearing field is visible instead of silently degrading behaviour.
                    RobotLog.ww(TAG, "deepCopy skipped %s.%s: %s",
                            c.getSimpleName(), src.getName(), e.getMessage());
                }
            }
        }
    }
}
