package frc.robot.Auto;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Drivetrain.Constants;
import frc.robot.Drivetrain.Drivetrain;

public class Auto {
    public static Drivetrain drivetrain = Drivetrain.getInstance();

    public static Command getAuto(){
        return new SequentialCommandGroup(
            drivetrain.runOnce(() -> drivetrain.resetPose(Constants.InitialPose)),
            drivetrain.drive(() -> 1.0, () -> 1.0).withTimeout(Seconds.of(1))
        );
    }
}
