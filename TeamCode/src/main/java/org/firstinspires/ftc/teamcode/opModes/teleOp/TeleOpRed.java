package org.firstinspires.ftc.teamcode.opModes.teleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.launcher.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.launcher.PositionalHood;
import org.firstinspires.ftc.teamcode.subsystems.launcher.TempHood;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Transfer;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpRed")
public class TeleOpRed extends NextFTCOpMode {

    public TeleOpRed() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(
                        TempHood.INSTANCE,
                        PositionalHood.INSTANCE,
                        Flywheel.INSTANCE,
                        DriveTrain.INSTANCE,
                        Intake.INSTANCE,
                        Transfer.INSTANCE
                ),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    public static boolean red;
    public static boolean isRed() { return red; }

    private Command buildShoot() {
        return new SequentialGroup(
                Transfer.INSTANCE.openTransfer,
                new Delay(0.1),
                new LambdaCommand().setStart(() -> Intake.INSTANCE.setPower(-0.6)).setIsDone(() -> true),
                Transfer.INSTANCE.transferOn,
                new Delay(0.6),
                Transfer.INSTANCE.transferOff,
                new LambdaCommand().setStart(() -> Intake.INSTANCE.setPower(0)).setIsDone(() -> true),
                Transfer.INSTANCE.closeTransfer
        );
    }

    @Override
    public void onInit() {
        red = true;
        DriveTrain.INSTANCE.matchStarted = false;

        // Right trigger = shoot
        Gamepads.gamepad1().rightTrigger().greaterThan(0.3)
                .whenBecomesTrue(() -> buildShoot().schedule());

        // Left trigger = intake + transfer slowly (parallel)
        Gamepads.gamepad1().leftTrigger().greaterThan(0.3)
                .whenBecomesTrue(() -> new ParallelGroup(
                        new LambdaCommand().setStart(() -> Intake.INSTANCE.setPower(-0.6)).setIsDone(() -> true),
                        new LambdaCommand().setStart(() -> Transfer.INSTANCE.setMotorPower(-0.4)).setIsDone(() -> true)
                ).schedule())
                .whenBecomesFalse(() -> {
                    Intake.INSTANCE.setPower(0);
                    Transfer.INSTANCE.setMotorPower(0);
                });

        // Triangle = autolock toggle
        Gamepads.gamepad1().triangle()
                .whenBecomesTrue(() -> DriveTrain.INSTANCE.setAutoLockTrue())
                .whenBecomesFalse(() -> DriveTrain.INSTANCE.setAutoLockFalse());

        // X = localize
        Gamepads.gamepad1().cross()
                .whenBecomesTrue(() -> DriveTrain.INSTANCE.getLocalize().schedule());

        // Circle = intake reverse
        Gamepads.gamepad1().circle()
                .whenBecomesTrue(() -> Intake.INSTANCE.setPower(1))
                .whenBecomesFalse(() -> Intake.INSTANCE.setPower(0));
    }

    @Override
    public void onStartButtonPressed() {
        DriveTrain.INSTANCE.matchStarted = true;
    }

    @Override
    public void onUpdate() {}

    @Override
    public void onStop() {
        red = false;
        DriveTrain.INSTANCE.matchStarted = false;
    }
}