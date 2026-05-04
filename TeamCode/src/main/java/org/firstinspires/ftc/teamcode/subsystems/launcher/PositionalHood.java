package org.firstinspires.ftc.teamcode.subsystems.launcher;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

@Configurable
public class PositionalHood implements Subsystem {

    public static final PositionalHood INSTANCE = new PositionalHood();
    private PositionalHood() {}

    private static Servo hoodServo1n;
    private static Servo hoodServo2n;

    // Lazy suppliers so commands work after initialize()
    private static ServoEx hoodServo1 = new ServoEx(() -> hoodServo1n);
    private static ServoEx hoodServo2 = new ServoEx(() -> hoodServo2n);

    public static ParallelGroup hoodPowerZero = new ParallelGroup(
            new SetPosition(hoodServo1, 0),
            new SetPosition(hoodServo2, 0)
    );

    public static ParallelGroup hoodRunDown = new ParallelGroup(
            new SetPosition(hoodServo1, 1),
            new SetPosition(hoodServo2, -1)
    );

    /** Positions hood at the given servo runtime value. Shared by DriveTrain and auto opmodes. */
    public static double hoodToPos(double runtime) {
        if (!Double.isNaN(runtime)) {
            ActiveOpMode.telemetry().addData("runtime", runtime);
            ParallelGroup moveHood = new ParallelGroup(
                    new SetPosition(hoodServo1, runtime),
                    new SetPosition(hoodServo2, -1 * runtime)
            );
            moveHood.schedule();
            return runtime;
        }
        ActiveOpMode.telemetry().addLine("NaN");
        return 0;
    }

    @Override
    public void initialize() {
        hoodServo1n = ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo1");
        hoodServo2n = ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo2");
    }

    @Override
    public void periodic() {}
}
