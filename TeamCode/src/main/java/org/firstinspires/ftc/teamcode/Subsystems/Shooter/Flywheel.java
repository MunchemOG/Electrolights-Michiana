package org.firstinspires.ftc.teamcode.Subsystems.Shooter;


import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.bindings.*;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.ControlSystem;

import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
@Configurable
public class Flywheel implements Subsystem {
    public Flywheel() {

    }

    public static final Flywheel INSTANCE = new Flywheel();
    public static double flywheelvelocity;

    public static double flywheelvelocity2;
    private ControlSystem controller1;
    private ControlSystem controller2;

    public static MotorEx flywheel = new MotorEx("launchingmotor");

    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");

    public static PIDCoefficients myPidCoeff = new PIDCoefficients(0.015, 0.005, 0.00);
    public static BasicFeedforwardParameters myFF = new BasicFeedforwardParameters(0.00045, 0, 0.0);


    public static double configvelocity = 1400; //far zone - ~1500. near zone - ~1200-1300

    public static void velocityControlWithFeedforwardExample(KineticState currentstate, float configtps) {
        // Create a velocity controller with PID and feedforward
        ControlSystem controller1 = ControlSystem.builder()
                .velPid(myPidCoeff) // Velocity PID with kP=0.1, kI=0.01, kD=0.05
                .basicFF(myFF) // Basic feedforward with kV=0.02, kA=0.0, kS=0.01 //pid tuning
                .build();

        controller1.setGoal(new KineticState(0.0, configtps, 0.0));

        // In a loop (simulated here), you would:
        // Create a KineticState with current position and velocity

        double power = controller1.calculate(currentstate);
        flywheel.setPower(power);
    }
    public static void velocityControlWithFeedforwardExample2(KineticState currentstate, float configtps) {
        // Create a velocity controller with PID and feedforward
        ControlSystem controller2 = ControlSystem.builder()
                .velPid(myPidCoeff) // Velocity PID with kP=0.1, kI=0.01, kD=0.05
                .basicFF(myFF) // Basic feedforward with kV=0.02, kA=0.0, kS=0.01 //pid tuning
                .build();

        controller2.setGoal(new KineticState(0.0, configtps, 0.0));

        // In a loop (simulated here), you would:
        // Create a KineticState with current position and velocity

        double power = controller2.calculate(currentstate);
        flywheel2.setPower(-1*power);
    }
    public static void shooter(float tps) {
        BindingManager.update();
        flywheelvelocity = flywheel.getVelocity();
        flywheelvelocity2 = flywheel2.getVelocity();
        KineticState currentState = new KineticState(0, flywheelvelocity, 0.0);
        KineticState currentState2 = new KineticState(0, -1*flywheelvelocity2, 0.0);
        //if(tps-(-1*flywheelvelocity)<7 && tps-(-1*flywheelvelocity)>-7){
        velocityControlWithFeedforwardExample(currentState, tps);
        velocityControlWithFeedforwardExample2(currentState2, tps);
        double rpm = (flywheelvelocity / 28) * 60.0;

    }
    @Override public void initialize() {

    }

    @Override public void periodic() {

    }
}

