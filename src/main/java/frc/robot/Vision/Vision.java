package frc.robot.Vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Drivetrain.Drivetrain;

public class Vision extends SubsystemBase{
    public PhotonCamera LocalizationCamera;
    public PhotonCameraSim SimCamera;
    public VisionSystemSim SimSystem;
    public PhotonPoseEstimator PoseEstimator;
    public UsbCamera DriverCamera;
    public Pose3d[] Targets;
    public boolean hasVision = false;
    public Pose3d RawPose;
    public Alert LocalizationAlert, DriverAlert;

    public static Vision vision;
    public PhotonPipelineResult VisionResult;

    private Vision(){
        LocalizationCamera = new PhotonCamera(Constants.LocalizationCamera);
        if(!RobotBase.isSimulation()){
            DriverCamera = new UsbCamera(Constants.DriverCamera, 0);
            DriverCamera.setConnectionStrategy(ConnectionStrategy.kAutoManage);
            CameraServer.addCamera(DriverCamera);
        }else{
            simInit();
        }
        PoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(Constants.CameraPose.toMatrix()));
        LocalizationAlert = new Alert("System Alerts", "Localization Camera Not Connected", AlertType.kWarning);
        DriverAlert = new Alert("System Alerts", "Driver Camera Not Connected", AlertType.kWarning);
        LocalizationAlert.set(false);
        LocalizationAlert.set(false);
    }

    public Pose2d getPose(){
        try{
            if(!LocalizationCamera.isConnected()) throw new RuntimeException("");
            if(VisionResult == null) throw new RuntimeException("");
            var pose = PoseEstimator.update(VisionResult).orElseThrow();
            RawPose = pose.estimatedPose;
            hasVision = true;
            return pose.estimatedPose.toPose2d();
        }catch(Exception e){
            hasVision = false;
            return null;
        }
    }

    public void simInit(){
        SimCamera = new PhotonCameraSim(LocalizationCamera, SimCameraProperties.LL2_640_480());
        SimSystem = new VisionSystemSim("VisionSystemSim");
        SimSystem.addCamera(SimCamera, new Transform3d(Constants.CameraPose.toMatrix()));
        SimSystem.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark));
    }

    public Pose3d[] getTargets(){
        Pose2d currentPose = getPose();
        if(currentPose != null && VisionResult != null){
            List<Pose3d> tars = new ArrayList<Pose3d>();
            VisionResult.targets.forEach(tar -> tars.add(new Pose3d(currentPose).plus(tar.getBestCameraToTarget())));
            hasVision = true;
            return tars.toArray(Pose3d[]::new);
        }else{
            return null;
        }
    }

    @Override
    public void simulationPeriodic(){
        SimSystem.update(Drivetrain.getInstance().getRobotPose());
    }

    @Override
    public void periodic(){
        LocalizationAlert.set(LocalizationCamera.isConnected());
        if(!RobotBase.isSimulation())DriverAlert.set(DriverCamera.isConnected());
        var res = LocalizationCamera.getAllUnreadResults();
        VisionResult = !res.isEmpty() ? res.get(res.size() - 1) : null;
    }

    public static Vision getInstance(){
        if(vision == null) vision = new Vision();
        return vision;
    }
}
