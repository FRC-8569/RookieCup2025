package frc.robot.Vision;

import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class Constants {
    public static final String LocalizationCamera = "LocalizationCamera";
    public static final String DriverCamera = "DriverCamera";
    public static final Pose3d CameraPose = new Pose3d(Millimeters.of(120.650),Millimeters.of( 266.300), Millimeters.of(-269.87050), Rotation3d.kZero);
}
