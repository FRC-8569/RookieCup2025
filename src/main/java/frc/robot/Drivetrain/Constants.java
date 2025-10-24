package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class Constants {
    public static final int[] LeftIDs = {11,12};
    public static final int[] RightIDs = {14,13};
    public static final double GearRatio = 10.71;
    public static final double WheelCirc = Inches.of(6).times(Math.PI).in(Meters);
    public static final double PositionConvertionFactor = 1/GearRatio*WheelCirc;
    public static final double VelocityConvertionFactor = PositionConvertionFactor/60;
    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Inches.of(20));
    public static final Pose2d InitialPose = new Pose2d(7.6,7, Rotation2d.kZero);
    public static final int PDHCANID = 20;
    public static final IdleMode MotorMode = IdleMode.kBrake;
    public static final int SlipCurrent = 40;

    public static final ClosedLoopConfig LeftPID = new ClosedLoopConfig()
        .pidf(0, 0, 0, 1.0/473)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    public static final ClosedLoopConfig RightPID = new ClosedLoopConfig()
        .pidf(0, 0, 0, 1.0/473)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
}
