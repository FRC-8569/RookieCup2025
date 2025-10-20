// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Drivetrain.Drivetrain;
import frc.utils.FieldObject;
import frc.utils.Scoring;

public class RobotContainer {
  public Drivetrain drivetrain = Drivetrain.getInstance();
  // public Shooter shooter = Shooter.getInstance();
  public static XboxController controller = new XboxController(0);
  public static XboxController ShooterController = new XboxController(1);
  public Telemetry telemetry = new Telemetry();
  public BooleanSubscriber isFuckingCoral = NetworkTableInstance.getDefault().getBooleanTopic("isFuckingCoral").subscribe(false);

  public RobotContainer() {
    drivetrain.setDefaultCommand(drivetrain.drive(
      () -> -controller.getLeftY()*0.8, 
      () -> controller.getLeftX()*0.3));
    configureBindings();
  }

  private void configureBindings() {
    // new Trigger(() -> controller.getXButton())
    //   .whileTrue(shooter.runShooter());
    
    // new Trigger(() -> ShooterController.getLeftY() > 0.5)
    //   .whileTrue(shooter.runShooter());

    new Trigger(() -> ShooterController.getXButton())
      .onTrue(Scoring.getInstance().score(FieldObject.L1));

    new Trigger(() -> ShooterController.getYButton())
    .onTrue(Scoring.getInstance().score(FieldObject.L2));

    new Trigger(() -> ShooterController.getAButton())
      .onTrue(Scoring.getInstance().score(FieldObject.L3));

    new Trigger(() -> ShooterController.getBButton())
      .onTrue(Scoring.getInstance().score(FieldObject.Processor));

    new Trigger(() -> ShooterController.getRightBumperButton())
      .onTrue(Scoring.getInstance().score(FieldObject.Railing));
    
    new Trigger(() -> ShooterController.getLeftBumperButton())
      .onTrue(Scoring.getInstance().descore());
  }

  public Command getAutonomousCommand(){
    return Commands.none();
  }

}
