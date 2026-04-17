package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;


import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpBlue")
public class TeleOpBlue extends NextFTCOpMode {

    public MotorEx intakeMotor;
    public MotorEx transfer;
    public TeleOpBlue() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(DriveTrain.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE


        );
    }

    public static boolean blue;
    public static boolean isBlue(){
        return blue;
    }







    @Override
    public void onInit() {
        blue=true;
        intakeMotor = new MotorEx("intake").reversed();
        transfer = new MotorEx("transfer").reversed();
        Gamepads.gamepad1().leftTrigger().greaterThan(0.3).whenBecomesTrue(()-> intakeMotor.setPower(1))
                .whenBecomesFalse(() -> intakeMotor.setPower(0));
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(()-> transfer.setPower(1))
                .whenBecomesFalse(() -> transfer.setPower(0));
        Gamepads.gamepad2().leftTrigger().greaterThan(0.3).whenBecomesTrue(()->intakeMotor.setPower(-1))
                .whenBecomesFalse(() -> intakeMotor.setPower(0));
        Gamepads.gamepad2().rightTrigger().greaterThan(0.3).whenBecomesTrue(()-> transfer.setPower(-1))
                .whenBecomesFalse(() -> intakeMotor.setPower(0));




    }

    @Override
    public void onUpdate() {
        float newtps=1000;

    }
    public boolean shoot;
    @Override
    public void onStartButtonPressed() {
    }

    public void onStop(){
        blue=false;
    }
}