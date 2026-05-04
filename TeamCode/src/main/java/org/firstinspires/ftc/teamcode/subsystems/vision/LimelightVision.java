package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * Wraps Limelight3A initialization and pipeline selection.
 * Extend or call setPipeline() before use.
 */
public class LimelightVision implements Subsystem {

    public static final LimelightVision INSTANCE = new LimelightVision();
    private LimelightVision() {}

    private Limelight3A limelight;

    private int pipeline = 0;

    /** Set desired pipeline before initialize() runs (e.g. in opmode constructor). */
    public void setPipeline(int pipeline) {
        this.pipeline = pipeline;
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    @Override
    public void initialize() {
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    @Override
    public void periodic() {}
}
