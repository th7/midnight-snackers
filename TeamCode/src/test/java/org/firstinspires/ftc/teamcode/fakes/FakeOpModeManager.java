package org.firstinspires.ftc.teamcode.fakes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

/**
 * Records what a registrar registers, the way the robot controller's manager would keep it.
 */
public final class FakeOpModeManager implements OpModeManager {
    public static final class Registration {
        public final OpModeMeta meta;
        /** The registered instance, or null when a class was registered. */
        public final OpMode instance;
        /** The registered class, or null when an instance was registered. */
        public final Class<? extends OpMode> type;

        Registration(OpModeMeta meta, OpMode instance, Class<? extends OpMode> type) {
            this.meta = meta;
            this.instance = instance;
            this.type = type;
        }

        /**
         * The op mode as the robot controller runs it: the registered instance every time, or a
         * new instance of the registered class each time.
         */
        public OpMode opMode() {
            if (instance != null) {
                return instance;
            }
            try {
                return type.getDeclaredConstructor().newInstance();
            } catch (ReflectiveOperationException e) {
                throw new IllegalStateException("could not construct " + type.getName(), e);
            }
        }
    }

    public final List<Registration> registrations = new ArrayList<>();

    public Optional<Registration> find(String name) {
        return registrations.stream().filter(r -> r.meta.name.equals(name)).findFirst();
    }

    @Override
    public void register(String name, Class<? extends OpMode> opModeClass) {
        register(new OpModeMeta.Builder().setName(name).build(), opModeClass);
    }

    @Override
    public void register(OpModeMeta meta, Class<? extends OpMode> opModeClass) {
        registrations.add(new Registration(meta, null, opModeClass));
    }

    @Override
    public void register(String name, OpMode opModeInstance) {
        register(new OpModeMeta.Builder().setName(name).build(), opModeInstance);
    }

    @Override
    public void register(OpModeMeta meta, OpMode opModeInstance) {
        registrations.add(new Registration(meta, opModeInstance, null));
    }
}
