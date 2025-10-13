package frc.robot.Shooter;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.FieldObject;
import frc.utils.Scoring;

public class Shooter extends SubsystemBase{
    public SparkMax Motor;
    public SparkMaxSim SimMotor;
    private SparkMaxConfig config;

    public static Shooter shooter;
    public String NowDoing = "null";

    private Shooter(){
        Motor = new SparkMax(Constants.Motor, MotorType.kBrushless);
        config = new SparkMaxConfig();

        config
            .idleMode(IdleMode.kCoast)
            .inverted(false)
            .voltageCompensation(12)
            .smartCurrentLimit(15);

        if(RobotBase.isSimulation()){
            SimMotor = new SparkMaxSim(Motor, DCMotor.getNeo550(1).withReduction(3));
        }
    }

    public Command runShooter(){
        return runEnd(
            () -> {
                if(RobotBase.isReal())Motor.set(0.8);
                else SimMotor.setAppliedOutput(0.8);
                NowDoing = "PutCoral";
            }, 
            () -> {
                if(RobotBase.isReal())Motor.stopMotor();
                else SimMotor.setAppliedOutput(0);
                NowDoing = "null";
            }).withTimeout(Seconds.of(0.8)).andThen(Scoring.getInstance().score(FieldObject.L1));
    }

    public double getMotorTemp(){
        return Motor.getMotorTemperature();
    }

    public double getCurrentDraw(){
        return RobotBase.isReal() ? Motor.getOutputCurrent() : SimMotor.getMotorCurrent();
    }

    public static Shooter getInstance(){
        if(shooter == null) shooter = new Shooter();
        return shooter;
    }
}
