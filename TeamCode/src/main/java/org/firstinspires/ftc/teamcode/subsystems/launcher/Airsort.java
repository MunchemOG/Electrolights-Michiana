package org.firstinspires.ftc.teamcode.subsystems.launcher;

import com.qualcomm.robotcore.hardware.CRServo;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.powerable.SetPower;

/**
 * Airsort manages hood servo movement for ball sorting.
 * Transfer motor logic lives in subsystems/transfer/Transfer.java.
 */
public class Airsort implements Subsystem {

    public static final Airsort INSTANCE = new Airsort();
    private Airsort() {}

    private CRServo hoodServo1n;
    private CRServo hoodServo2n;

    // Lazy suppliers so commands work after initialize()
    private CRServoEx hoodServo1 = new CRServoEx(() -> hoodServo1n);
    private CRServoEx hoodServo2 = new CRServoEx(() -> hoodServo2n);

    private ParallelGroup hoodRunUp = new ParallelGroup(
            new SetPower(hoodServo1, 1),
            new SetPower(hoodServo2, -1)
    );

    public ParallelGroup hoodPowerZero = new ParallelGroup(
            new SetPower(hoodServo1, 0),
            new SetPower(hoodServo2, 0)
    );

    public SequentialGroup hoodDown = new SequentialGroup(
            hoodRunUp,
            new Delay(0.18),
            hoodPowerZero
    );

    private ParallelGroup hoodRunDown = new ParallelGroup(
            new SetPower(hoodServo1, -1),
            new SetPower(hoodServo2, 1)
    );

    public SequentialGroup hoodUp = new SequentialGroup(
            hoodRunDown,
            new Delay(0.17),
            hoodPowerZero
    );

    // Ball sorting sequences - require Transfer.INSTANCE to be initialized first.
    // These are called externally after both Airsort and Transfer subsystems are initialized.
    public SequentialGroup buildPpgToPgp() {
        return new SequentialGroup(
                TempHood.hoodUp, new Delay(0.18),
                TempHood.hoodUp,
                TempHood.hoodUp,
                TempHood.hoodDown
        );
    }

    public SequentialGroup buildGppToPgp() {
        return new SequentialGroup(
                TempHood.hoodUp,
                TempHood.hoodUp,
                TempHood.hoodDown
        );
    }

    public SequentialGroup buildPgpToPpg() {
        return new SequentialGroup(
                TempHood.hoodUp, new Delay(0.18),
                TempHood.hoodUp,
                TempHood.hoodUp,
                TempHood.hoodDown
        );
    }

    @Override
    public void initialize() {
        hoodServo1n = ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo1");
        hoodServo2n = ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo2");
    }

    @Override
    public void periodic() {}
}
