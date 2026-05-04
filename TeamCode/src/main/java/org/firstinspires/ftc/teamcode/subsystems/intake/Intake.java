package org.firstinspires.ftc.teamcode.subsystems.intake;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

/**
 * Manages the intake motor.
 * Positive power = intake, negative = reverse/eject.
 */
public class Intake implements Subsystem {

    public static final Intake INSTANCE = new Intake();
    private Intake() {}

    public MotorEx intakeMotor;

    public Command intakeOn = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(-1))
            .setIsDone(() -> true);

    public Command intakeOff = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(0))
            .setIsDone(() -> true);

    public Command intakeReverse = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(1))
            .setIsDone(() -> true);

    public Command intakeSlowReverse = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(0.5))
            .setIsDone(() -> true);

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    @Override
    public void initialize() {
        intakeMotor = new MotorEx("intake");
    }

    @Override
    public void periodic() {}
}
