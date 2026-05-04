package org.firstinspires.ftc.teamcode.subsystems.transfer;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

/**
 * Manages the transfer motor and servo gate.
 * openTransfer / closeTransfer control the servo gate.
 * transferOn / transferOff control the motor.
 */
public class Transfer implements Subsystem {

    public static final Transfer INSTANCE = new Transfer();
    private Transfer() {}

    public MotorEx transferMotor;
    public ServoEx transferServo;

    public static double transferPower = -1.0;

    // Open servo gate (ball can pass through)
    public Command openTransfer = new LambdaCommand()
            .setStart(() -> transferServo.setPosition(0.35))
            .setIsDone(() -> true);

    // Close servo gate
    public Command closeTransfer = new LambdaCommand()
            .setStart(() -> transferServo.setPosition(0.635))
            .setIsDone(() -> true);

    public Command transferOn = new LambdaCommand()
            .setStart(() -> transferMotor.setPower(transferPower))
            .setIsDone(() -> true);

    public Command transferOff = new LambdaCommand()
            .setStart(() -> transferMotor.setPower(0))
            .setIsDone(() -> true);

    public void setMotorPower(double power) {
        transferMotor.setPower(power);
    }

    public void setServoPosition(double pos) {
        transferServo.setPosition(pos);
    }

    @Override
    public void initialize() {
        transferMotor = new MotorEx("transfer");
        transferServo = new ServoEx("transferServo1");
    }

    @Override
    public void periodic() {}
}
