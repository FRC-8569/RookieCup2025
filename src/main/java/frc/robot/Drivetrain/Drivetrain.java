package frc.robot.Drivetrain;

import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase{
    public SparkMax LeftMotor, LeftBackMotor, RightMotor, RightBackMotor;
    public RelativeEncoder LeftEncoder, RightEncoder;
    public SparkClosedLoopController LeftPID, RightPID;
    public AHRS gyro;
    public DifferentialDrivePoseEstimator PoseEstimator;
    public SparkMaxSim LeftSim, RightSim;
    public SparkRelativeEncoderSim LeftEncoderSim, RightEncoderSim;
    public DifferentialDrivetrainSim SystemSim;

    private SparkMaxConfig LeftConfig, LeftBackConfig, RightConfig, RightBackConfig;

    public String NowDoing = "null";

    private static Drivetrain system;
    //public Vision vision;

    private Drivetrain(){
        LeftMotor = new SparkMax(Constants.LeftIDs[0], MotorType.kBrushless);
        LeftBackMotor = new SparkMax(Constants.LeftIDs[1], MotorType.kBrushless);
        RightMotor = new SparkMax(Constants.RightIDs[0], MotorType.kBrushless);
        RightBackMotor = new SparkMax(Constants.RightIDs[1], MotorType.kBrushless);

        LeftEncoder = LeftMotor.getEncoder();
        RightEncoder = RightMotor.getEncoder();
        LeftPID = LeftMotor.getClosedLoopController();
        RightPID = RightMotor.getClosedLoopController();
        gyro = new AHRS(NavXComType.kMXP_SPI);
        PoseEstimator = new DifferentialDrivePoseEstimator(Constants.kinematics, gyro.getRotation2d(), getPosition().leftMeters, getPosition().rightMeters, Constants.InitialPose);
        
        NowDoing = "null";

        LeftConfig = new SparkMaxConfig();
        RightConfig = new SparkMaxConfig();
        LeftBackConfig = new SparkMaxConfig();
        RightBackConfig = new SparkMaxConfig();

        LeftConfig
            .idleMode(Constants.MotorMode)
            .inverted(false)
            .voltageCompensation(12)
            .smartCurrentLimit(Constants.SlipCurrent);
        LeftConfig.encoder
            .positionConversionFactor(Constants.PositionConvertionFactor)
            .velocityConversionFactor(Constants.VelocityConvertionFactor);
        LeftConfig.closedLoop.apply(Constants.LeftPID);

        LeftBackConfig.idleMode(Constants.MotorMode);
        LeftBackConfig.smartCurrentLimit(Constants.SlipCurrent);
        LeftBackConfig.follow(LeftMotor);

        RightConfig
            .idleMode(Constants.MotorMode)
            .inverted(true)
            .voltageCompensation(12)
            .smartCurrentLimit(Constants.SlipCurrent);
        RightConfig.encoder
            .positionConversionFactor(Constants.PositionConvertionFactor)
            .velocityConversionFactor(Constants.VelocityConvertionFactor);
        RightConfig.closedLoop.apply(Constants.RightPID);
        RightBackConfig.idleMode(Constants.MotorMode);
        RightBackConfig.smartCurrentLimit(Constants.SlipCurrent);
        RightBackConfig.follow(RightMotor);

        LeftMotor.configure(LeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        RightMotor.configure(RightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        LeftBackMotor.configure(LeftBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        RightBackMotor.configure(RightBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        AutoInit();
        //vision = Vision.getInstance();
        if(RobotBase.isSimulation()) simInit();
        SmartDashboard.putData("PDH", new PowerDistribution(20, ModuleType.kCTRE));
    }

    public void drive(double LeftSpeed, double RightSpeed){
        if(RobotBase.isReal()){
            // LeftPID.setReference(LeftSpeed, ControlType.kVelocity);
            // RightPID.setReference(RightSpeed, ControlType.kVelocity);
            LeftMotor.set(LeftSpeed/(5676*Constants.VelocityConvertionFactor));
            RightMotor.set(RightSpeed/(5676*Constants.VelocityConvertionFactor));
        }else{
            LeftSim.setAppliedOutput(LeftSpeed/(5676*Constants.VelocityConvertionFactor));
            RightSim.setAppliedOutput(RightSpeed/(5676*Constants.VelocityConvertionFactor));
        }
    }

    public Command drive(Supplier<Double> Throttle,Supplier<Double>  Rotataion){
        return run(() -> drive((Throttle.get()+Rotataion.get())*(5676*Constants.VelocityConvertionFactor), (Throttle.get()-Rotataion.get())*(5676*Constants.VelocityConvertionFactor)));
    }

    public Pose2d getRobotPose(){
        return PoseEstimator.getEstimatedPosition();
    }

    public DifferentialDriveWheelPositions getPosition(){
        return new DifferentialDriveWheelPositions(
            LeftEncoder.getPosition(),
            RightEncoder.getPosition());
    }

    public DifferentialDriveWheelSpeeds getSpeeds(){
        return new DifferentialDriveWheelSpeeds(
            LeftEncoder.getVelocity(),
            RightEncoder.getVelocity()
        );
    }

    public void resetPose(Pose2d pose){
        PoseEstimator.resetPose(pose);
    }

    public void AutoInit(){
        try{
            AutoBuilder.configure(
                () -> this.getRobotPose(), 
                this::resetPose, 
                () -> Constants.kinematics.toChassisSpeeds(getSpeeds()), 
                (speeds, ff) -> drive(Constants.kinematics.toWheelSpeeds(speeds).leftMetersPerSecond, Constants.kinematics.toWheelSpeeds(speeds).rightMetersPerSecond), 
                new PPLTVController(
                    VecBuilder.fill(0.8, 0.8, .625), //qX qY, qθ,Like kP — how aggressively it corrects error
                    VecBuilder.fill(1, 1), //rV, r⍵ Like kD — damps oscillations, makes it smoother
                    0.02, 5676*Constants.VelocityConvertionFactor), 
                RobotConfig.fromGUISettings(), 
                () -> DriverStation.getAlliance().orElseThrow() == Alliance.Red, 
                this);
        }catch(Exception e){
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
        }
    }

    public void simInit(){
        SystemSim = new DifferentialDrivetrainSim(
            DCMotor.getNEO(2), 
            Constants.GearRatio, 
            2.300, 
            50, 
            Constants.WheelCirc/Math.PI/2, 
            Constants.kinematics.trackWidthMeters, 
            null);

        LeftSim = new SparkMaxSim(LeftMotor, DCMotor.getNEO(2));
        RightSim = new SparkMaxSim(RightMotor, DCMotor.getNEO(2));
        LeftEncoderSim = LeftSim.getRelativeEncoderSim();
        RightEncoderSim = RightSim.getRelativeEncoderSim();
        LeftSim.useDriverStationEnable();
        RightSim.useDriverStationEnable();
    }

    @Override
    public void simulationPeriodic(){
        SystemSim.setInputs(LeftSim.getAppliedOutput()*12, RightSim.getAppliedOutput()*12);
        LeftEncoderSim.setPosition(SystemSim.getLeftPositionMeters());
        RightEncoderSim.setPosition(SystemSim.getRightPositionMeters());
        LeftEncoderSim.setVelocity(SystemSim.getLeftVelocityMetersPerSecond());
        RightEncoderSim.setVelocity(SystemSim.getRightVelocityMetersPerSecond());
        gyro.setAngleAdjustment(-SystemSim.getHeading().getDegrees());
        SystemSim.update(0.02);
    }

    public double getMaxTemp(){
        return Collections.max(List.of(LeftMotor.getMotorTemperature(),LeftBackMotor.getMotorTemperature(), RightMotor.getMotorTemperature(), RightBackMotor.getMotorTemperature()));
    }

    @Override
    public void periodic(){
        PoseEstimator.update(gyro.getRotation2d(), getPosition());
        if((getSpeeds().leftMetersPerSecond+getSpeeds().rightMetersPerSecond)/2 < 0.1){
            NowDoing = "idle";
        }else{
            NowDoing = "drive";
        }
    }

    public static Drivetrain getInstance(){
        if(system == null) system = new Drivetrain();
        return system;
    }
}
