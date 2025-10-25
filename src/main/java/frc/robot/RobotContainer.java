// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Auto.Auto;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Shooter.Shooter;
import frc.utils.FieldObject;

public class RobotContainer {
  public Drivetrain drivetrain = Drivetrain.getInstance();
  public Shooter shooter = Shooter.getInstance();
  public static XboxController controller = new XboxController(0);
  public static XboxController ShooterController = new XboxController(1);
  public Telemetry telemetry = new Telemetry();
  public BooleanSubscriber isFuckingCoral = NetworkTableInstance.getDefault().getBooleanTopic("isFuckingCoral").subscribe(false);

  public RobotContainer() {
    drivetrain.setDefaultCommand(drivetrain.drive(
      () -> -controller.getLeftY()*0.6, 
      () -> controller.getLeftX()*0.5));
    configureBindings();
  }

  private void configureBindings() {
    new Trigger(() -> ShooterController.getXButton())
      .whileTrue(shooter.runShooter());
    new Trigger(() -> controller.getStartButton())
      .onTrue(drivetrain.reverseDirection());
  }

  public Command getAutonomousCommand(){
    return Auto.getAuto();
  }

}
