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
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpBlue")
public class TeleOpBlue extends NextFTCOpMode {

    public TeleOpBlue() {
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

    // Alliance flag read by DriveTrain
    public static boolean blue;
    public static boolean isBlue() {
        return blue;
    }

    // Vision / ball color state (used by auto sorting logic)
    public static int tagId;
    public static boolean findMotif = false;
    public static int ball1Color = 0; // 1 = green, 2 = purple
    public static int ball2Color = 0;
    public static int ball3Color = 0;

    public static int getBall1Color() { return ball1Color; }
    public static int getBall2Color() { return ball2Color; }
    public static int getBall3Color() { return ball3Color; }

    // Shoot sequence: open gate → run transfer → close gate
    private Command buildShoot() {
        return new SequentialGroup(
                Transfer.INSTANCE.openTransfer,
                new Delay(0.1),
                Transfer.INSTANCE.transferOn,
                new Delay(0.4),
                Transfer.INSTANCE.transferOff,
                Transfer.INSTANCE.closeTransfer
        );
    }

    @Override
    public void onInit() {
        blue = true;

        // === Gamepad 1 bindings ===

        // Intake on/off
        Gamepads.gamepad1().leftTrigger().greaterThan(0.3)
                .whenBecomesTrue(() -> Intake.INSTANCE.setPower(-1))
                .whenBecomesFalse(() -> Intake.INSTANCE.setPower(0));

        // Autolock (auto-aim) hold
        Gamepads.gamepad1().triangle()
                .whenBecomesTrue(() -> DriveTrain.INSTANCE.setAutoLockTrue())
                .whenBecomesFalse(() -> DriveTrain.INSTANCE.setAutoLockFalse());

        // Localize pose
        Gamepads.gamepad1().x()
                .whenBecomesTrue(() -> DriveTrain.INSTANCE.getLocalize().schedule());

        // Hood angle control
        Gamepads.gamepad1().cross()
                .whenBecomesTrue(() -> DriveTrain.INSTANCE.hoodControl());

        // Shoot (right trigger)
        Gamepads.gamepad1().rightTrigger().greaterThan(0.3)
                .whenBecomesTrue(() -> buildShoot().schedule());

        // === Gamepad 2 bindings ===

        // Intake reverse
        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(() -> Intake.INSTANCE.setPower(1))
                .whenBecomesFalse(() -> Intake.INSTANCE.setPower(0));

        // Transfer motor reverse
        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> Transfer.INSTANCE.setMotorPower(1))
                .whenBecomesFalse(() -> Transfer.INSTANCE.setMotorPower(0));
    }

    @Override
    public void onUpdate() {
        // Aiming and flywheel speed are managed in DriveTrain.periodic()
    }

    @Override
    public void onStartButtonPressed() {
        // Sequences can be scheduled here when match starts
    }

    @Override
    public void onStop() {
        blue = false;
    }
}
