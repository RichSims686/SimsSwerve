package frc.robot.subsystems.apriltagvision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;

public class AprilTagCameraIOPhotonVision implements AprilTagCameraIO {

    private final PhotonCamera camera;
    private PhotonPoseEstimator photonPoseEstimator;
    private final  Transform3d robotToCamera;

    public AprilTagCameraIOPhotonVision(String cameraName, Transform3d robotToCamera) {
        camera = new PhotonCamera(cameraName);
        this.robotToCamera = robotToCamera;

        try {
            // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
            String resourceFile = Filesystem.getDeployDirectory() + "/apriltags/sims-basement.json";            
            AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(resourceFile);

            // test that I made the JSON file correctly
            // List<AprilTag> tags = fieldLayout.getTags();
            // for (AprilTag tag : tags) {
            //     System.out.println(tag);
            //     System.out.println(tag.pose.getRotation().toRotation2d());
            // }

            photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, camera, robotToCamera);
            photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        } catch (IOException e) {
            // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
            // where the tags are.
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            photonPoseEstimator = null;
        }
    }

    public void updateInputs(AprilTagCameraIOInputs inputs) {
        // set default values
        inputs.robotToCamera = robotToCamera;
        inputs.isConnected = camera.isConnected();
        inputs.visionPose = Optional.empty();
        inputs.timestamp = Timer.getFPGATimestamp();
        inputs.cameraToTargets.clear();

        if ((!inputs.isConnected) || (photonPoseEstimator == null)) {
            return;
        }

        PhotonPipelineResult cameraResult = camera.getLatestResult();
        photonPoseEstimator.setReferencePose(RobotState.getInstance().getPose());
        Optional<EstimatedRobotPose> optRobotPose = photonPoseEstimator.update(cameraResult);

        if (optRobotPose.isPresent()) {
            inputs.visionPose = Optional.of(optRobotPose.get().estimatedPose);
            inputs.timestamp = optRobotPose.get().timestampSeconds;
        }

        // populate targets array
        if (!cameraResult.hasTargets()) {
            return;
        }

        for (var target : cameraResult.getTargets()) {
            
            inputs.cameraToTargets.add(
                new AprilTagTarget(
                    target.getFiducialId(), 
                    target.getBestCameraToTarget(), 
                    target.getPoseAmbiguity()));
        }
    }
}
