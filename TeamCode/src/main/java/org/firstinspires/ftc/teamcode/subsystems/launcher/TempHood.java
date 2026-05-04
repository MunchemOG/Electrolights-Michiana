package org.firstinspires.ftc.teamcode.subsystems.launcher;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.powerable.SetPower;

@Configurable
public class TempHood implements Subsystem {

    public static final TempHood INSTANCE = new TempHood();
    private TempHood() {}

    private static CRServo hoodServo1n;
    private static CRServo hoodServo2n;

    // Lazy suppliers so commands work after initialize()
    private static CRServoEx hoodServo1 = new CRServoEx(() -> hoodServo1n);
    private static CRServoEx hoodServo2 = new CRServoEx(() -> hoodServo2n);

    public static ParallelGroup hoodRunUp = new ParallelGroup(
            new SetPower(hoodServo1, -1),
            new SetPower(hoodServo2, 1)
    );

    public static ParallelGroup hoodPowerZero = new ParallelGroup(
            new SetPower(hoodServo1, 0),
            new SetPower(hoodServo2, 0)
    );

    public static SequentialGroup hoodUp = new SequentialGroup(
            hoodRunUp,
            new Delay(0.18),
            hoodPowerZero
    );

    public static ParallelGroup hoodRunDown = new ParallelGroup(
            new SetPower(hoodServo1, 1),
            new SetPower(hoodServo2, -1)
    );

    public static SequentialGroup hoodDown = new SequentialGroup(
            hoodRunDown,
            new Delay(0.17),
            hoodPowerZero
    );

    /** Moves hood to target position relative to current state. */
    public static double hoodUp(double targetRuntime, double currentState) {
        if (!Double.isNaN(targetRuntime)) {
            ActiveOpMode.telemetry().addData("runtime", targetRuntime);
            ActiveOpMode.telemetry().addData("currentstate", currentState);

            SequentialGroup runUp = new SequentialGroup(
                    hoodRunUp, hoodRunUp, hoodRunUp, hoodRunUp,
                    new Delay(targetRuntime - currentState),
                    hoodPowerZero
            );
            SequentialGroup runDown = new SequentialGroup(
                    hoodRunDown,
                    new Delay(currentState - targetRuntime),
                    hoodPowerZero
            );

            if (targetRuntime > currentState + 0.007) {
                runUp.schedule();
                ActiveOpMode.telemetry().addLine("runUp");
                return targetRuntime;
            }
            if (targetRuntime < currentState - 0.007) {
                runDown.schedule();
                ActiveOpMode.telemetry().addLine("runDown");
                return targetRuntime;
            }
            ActiveOpMode.telemetry().addLine("returning0");
            return 0;
        }
        ActiveOpMode.telemetry().addLine("NaN");
        return 0;
    }

    @Override
    public void initialize() {
        // Uncomment to enable CR servo hood control:
        // hoodServo1n = ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo1");
        // hoodServo2n = ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo2");
    }

    @Override
    public void periodic() {}
}
