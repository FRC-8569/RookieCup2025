package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.wpilog.WPILOGWriter.AdvantageScopeOpenBehavior;
import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Vision.Vision;
import frc.utils.Scoring;

public class Telemetry extends SubsystemBase{
    public Drivetrain drivetrain;
    //public Shooter shooter;
    
    public StructPublisher<Pose2d> RobotPose;
    public StructPublisher<Pose3d> RawPose;
    public StructPublisher<ChassisSpeeds> RobotSpeeds;
    public StringPublisher ChassisNowDoing, ShooterNowDoing;
    public DoublePublisher LeftSpeed, RightSpeed, BatteryVoltage, MatchTime, ChassisTemp, ShooterTemp, RobotCurrent;
    public BooleanPublisher isVisionUsable, isFuckingCoral;
    public StructArrayPublisher<Pose3d> VisionTargets;
    public IntegerPublisher CurrentScore, DeltaCoral;
    public Field2d EasyField;

    public Telemetry(){
        drivetrain = Drivetrain.getInstance();
        // shooter = Shooter.getInstance();


        Logger.registerURCL(URCL.startExternal());
        LoggedPowerDistribution.getInstance(20, ModuleType.kCTRE);
        Logger.addDataReceiver(new WPILOGWriter(AdvantageScopeOpenBehavior.ALWAYS));
        Logger.start();

        RobotPose = NetworkTableInstance.getDefault().getStructTopic("Drivetrain/RobotPose", Pose2d.struct).publish();
        RobotSpeeds = NetworkTableInstance.getDefault().getStructTopic("Drivetrain/RobotSpeeds", ChassisSpeeds.struct).publish();
        ChassisNowDoing = NetworkTableInstance.getDefault().getStringTopic("NowDoing/Drivetrain").publish();
        ShooterNowDoing = NetworkTableInstance.getDefault().getStringTopic("NowDoing/Shooter").publish();
        BatteryVoltage = NetworkTableInstance.getDefault().getDoubleTopic("Utils/Battery").publish();
        MatchTime = NetworkTableInstance.getDefault().getDoubleTopic("Utils/MatchTime").publish();
        LeftSpeed = NetworkTableInstance.getDefault().getDoubleTopic("DataLog/Drivetrain/LeftSpeed").publish();
        RightSpeed = NetworkTableInstance.getDefault().getDoubleTopic("DataLog/Drivetrain/RightSpeed").publish();
        ChassisTemp = NetworkTableInstance.getDefault().getDoubleTopic("DataLog/Drivetrain/Temp").publish();
        ShooterTemp = NetworkTableInstance.getDefault().getDoubleTopic("DataLog/Shooter/Temp").publish();
        isVisionUsable = NetworkTableInstance.getDefault().getBooleanTopic("Vision/isVisionUsable").publish();
        VisionTargets = NetworkTableInstance.getDefault().getStructArrayTopic("Vision/Targets", Pose3d.struct).publish();
        RawPose = NetworkTableInstance.getDefault().getStructTopic("Vision/RawPose", Pose3d.struct).publish();
        RobotCurrent = NetworkTableInstance.getDefault().getDoubleTopic("DataLog/CurrentDraw").publish();
        isFuckingCoral = NetworkTableInstance.getDefault().getBooleanTopic("isFuckingCoral").publish();
        isFuckingCoral.setDefault(false);
        CurrentScore = NetworkTableInstance.getDefault().getIntegerTopic("NowDoing/CurrentScore").publish();
        DeltaCoral = NetworkTableInstance.getDefault().getIntegerTopic("NowDoing/DeltaCoral").publish();
        EasyField = new Field2d();
        SmartDashboard.putData(EasyField);
    }

    @Override
    public void periodic(){
        RobotPose.accept(drivetrain.getRobotPose());
        RobotSpeeds.accept(frc.robot.Drivetrain.Constants.kinematics.toChassisSpeeds(drivetrain.getSpeeds()));
        ChassisNowDoing.accept(drivetrain.NowDoing);
        // ShooterNowDoing.accept(shooter.NowDoing);
        BatteryVoltage.accept(RobotController.getBatteryVoltage());
        MatchTime.accept(DriverStation.getMatchTime());
        LeftSpeed.accept(drivetrain.getSpeeds().leftMetersPerSecond);
        RightSpeed.accept(drivetrain.getSpeeds().rightMetersPerSecond);
        // ShooterTemp.accept(shooter.getMotorTemp());
        isVisionUsable.accept(Vision.getInstance().hasVision);
        VisionTargets.accept(Vision.getInstance().getTargets());
        CurrentScore.accept(Scoring.getInstance().score);
        DeltaCoral.accept(Scoring.getInstance().getDeltaCoral());
        RawPose.accept(Vision.getInstance().RawPose);
        EasyField.setRobotPose(drivetrain.getRobotPose());

        DataLog();
    }

    @Override
    public void simulationPeriodic(){
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(drivetrain.SystemSim.getCurrentDrawAmps()));
    }

    public void DataLog(){
        Logger.recordOutput("Drivetrain/Robotpose", drivetrain.getRobotPose());
        Logger.recordOutput("Drivetrain/RobotSpeeds", frc.robot.Drivetrain.Constants.kinematics.toChassisSpeeds(drivetrain.getSpeeds()));
        Logger.recordOutput("Drivetrain/LeftSpeed", drivetrain.getSpeeds().leftMetersPerSecond);
        Logger.recordOutput("Drivetrain/RightSpeed", drivetrain.getSpeeds().rightMetersPerSecond);
        Logger.recordOutput("Drivetrain/Temp", drivetrain.getMaxTemp());
        Logger.recordOutput("Drivetrain/Accel", Math.sqrt(Math.pow(drivetrain.gyro.getRawAccelX(), 2)+Math.pow(drivetrain.gyro.getRawAccelY(),2)+Math.pow(drivetrain.gyro.getRawAccelZ(),2)));

        Logger.recordOutput("Utils/Battery", RobotController.getBatteryVoltage());
        Logger.recordOutput("Utils/MatchTime", DriverStation.getMatchTime());

        Logger.recordOutput("NowDoing/Drivetrain", drivetrain.NowDoing);
        // Logger.recordOutput("NowDoing/Shooter", shooter.NowDoing);
        // Logger.recordOutput("Shooter/Temp", shooter.getMotorTemp());

        Logger.recordOutput("Vision/hasVision", Vision.getInstance().hasVision);
        Logger.recordOutput("Vision/Targets", Vision.getInstance().Targets);
        Logger.recordOutput("Vision/RawPose", Vision.getInstance().getPose());
    }
}
