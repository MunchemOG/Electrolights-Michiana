package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class DistanceBlue implements Subsystem {

    private Limelight3A limelight3A;
    static double ta;
    static double tx;

    public static boolean hasTag;

    private IMU imu;

    public DistanceBlue() {

    }

    public static final DistanceBlue INSTANCE = new DistanceBlue();


    public void initialize() {
        limelight3A = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(8); //april tag 8 pipeline
        limelight3A.start();
        imu = ActiveOpMode.hardwareMap().get(IMU.class, "imu");
    }

    public void periodic() {
        LLResult result = limelight3A.getLatestResult();
        hasTag = (result != null) && result.isValid() && !result.getFiducialResults().isEmpty();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight3A.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            ta = llResult.getTa();
            tx = llResult.getTx();
        }
        ActiveOpMode.telemetry().addData("hasTag:", hasTag);
    }

    public double getDistanceFromTag() {
        if (hasTag) {
            double distance = 1.892 * Math.pow(ta, -0.513) + 0.08;
            return distance;
        }
        else {
            return 0.00;
        }

    }
    public static double getTx() {
        if (hasTag) {
            return tx;
        }
        else {
            return 0.00;
        }

    }

}
