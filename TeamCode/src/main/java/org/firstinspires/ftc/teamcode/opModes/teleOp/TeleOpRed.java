package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;

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

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

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

    // Alliance flag read by DriveTrain
    public static boolean red;
    public static boolean isRed() { return red; }

    // Vision / ball color state
    public static int tagId;
    public static boolean findMotif = false;
    public static int ball1Color = 0; // 1 = green, 2 = purple
    public static int ball2Color = 0;
    public static int ball3Color = 0;

    public static int getBall1Color() { return ball1Color; }
    public static int getBall2Color() { return ball2Color; }
    public static int getBall3Color() { return ball3Color; }

    /** Returns whether (x,y) is inside a launch zone triangle. */
    public boolean isInLaunchZone(double x, double y) {
        if (y >= 64 && y <= 144) {
            double halfWidth = (y - 64);
            if (x >= (72 - halfWidth) && x <= (72 + halfWidth)) return true;
        }
        if (y >= 0 && y <= 32) {
            double halfWidth = (32 - y);
            if (x >= (72 - halfWidth) && x <= (72 + halfWidth)) return true;
        }
        return false;
    }

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

    private static final int APRILTAG_PIPELINE = 7;

    @Override
    public void onInit() {
        red = true;

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
        red = false;
    }
}
