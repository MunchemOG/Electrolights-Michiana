package org.firstinspires.ftc.teamcode.subsystems.launcher;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class Flywheel implements Subsystem {

    public static final Flywheel INSTANCE = new Flywheel();
    private Flywheel() {}

    public static double flywheelVelocity;
    public static double flywheelVelocity2;

    public static MotorEx flywheel = new MotorEx("launchingmotor");
    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");

    public static PIDCoefficients pidCoeff = new PIDCoefficients(0.015, 0.005, 0.00);
    public static BasicFeedforwardParameters feedforward = new BasicFeedforwardParameters(0.00045, 0, 0.0);

    public static double configVelocity = 1400; // far zone ~1500, near zone ~1200-1300

    public static void velocityControl1(KineticState currentState, float targetTps) {
        ControlSystem controller = ControlSystem.builder()
                .velPid(pidCoeff)
                .basicFF(feedforward)
                .build();
        controller.setGoal(new KineticState(0.0, targetTps, 0.0));
        double power = controller.calculate(currentState);
        flywheel.setPower(power);
    }

    public static void velocityControl2(KineticState currentState, float targetTps) {
        ControlSystem controller = ControlSystem.builder()
                .velPid(pidCoeff)
                .basicFF(feedforward)
                .build();
        controller.setGoal(new KineticState(0.0, targetTps, 0.0));
        double power = controller.calculate(currentState);
        flywheel2.setPower(-1 * power);
    }

    public static void shooter(float tps) {
        BindingManager.update();
        flywheelVelocity = flywheel.getVelocity();
        flywheelVelocity2 = flywheel2.getVelocity();
        KineticState state1 = new KineticState(0, flywheelVelocity, 0.0);
        KineticState state2 = new KineticState(0, -1 * flywheelVelocity2, 0.0);
        velocityControl1(state1, tps);
        velocityControl2(state2, tps);
    }

    @Override
    public void initialize() {}

    @Override
    public void periodic() {}
}
