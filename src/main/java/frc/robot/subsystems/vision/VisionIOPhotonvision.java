package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;

public class VisionIOPhotonvision implements VisionIO {
  protected final PhotonCamera camera;
  private final PhotonPoseEstimator estimator;
  private final int cameraIndex;
  private final Supplier<Rotation2d> fieldRelativeYaw;


  public VisionIOPhotonvision(String name, int index, Supplier<Rotation2d> fieldRelativeYaw) {
    camera = new PhotonCamera(name);
    cameraIndex = index;
    this.fieldRelativeYaw = fieldRelativeYaw;
    estimator =
        new PhotonPoseEstimator(
            VisionConstants.APRIL_TAG_FIELD_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.CAMERA_TRANSFORM[index]);
  
    estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    List<PoseObservation> observations = new ArrayList<PoseObservation>();

    List<Integer> allTagIDs = new ArrayList<Integer>();

    for (int frameIndex = 0; frameIndex < results.size(); ++frameIndex) {
      PhotonPipelineResult frame = results.get(frameIndex);
      if (!frame.hasTargets()) continue;

      Optional<EstimatedRobotPose> optEstimation;

      optEstimation = estimator.estimateCoprocMultiTagPose(frame);
      // if (optEstimation.isEmpty()) {
      //   optEstimation = estimator.estimateLowestAmbiguityPose(frame);      
      // }

      if(optEstimation.isEmpty()){
        PhotonTrackedTarget target = frame.getTargets().get(0);
        
        double distance = target.getBestCameraToTarget().getTranslation().getNorm();
        if (target.getPoseAmbiguity() > 0.15) continue;

        int id = target.getFiducialId();
        Optional<Pose3d> tagPoseOpt = VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(id);
        
        if (tagPoseOpt.isPresent()) {
          Pose3d tagPose = tagPoseOpt.get();
          Pose3d robotPoseBest = tagPose.transformBy(target.getBestCameraToTarget().inverse())
                                        .transformBy(VisionConstants.CAMERA_TRANSFORM[cameraIndex].inverse());
          Pose3d robotPoseAlt = tagPose.transformBy(target.getAlternateCameraToTarget().inverse())
                                        .transformBy(VisionConstants.CAMERA_TRANSFORM[cameraIndex].inverse());
          
          Rotation2d currentGyro = fieldRelativeYaw.get();

          double bestPoseDiff = Math.abs(robotPoseBest.toPose2d().getRotation().minus(currentGyro).getRadians());
          double altPoseDiff = Math.abs(robotPoseAlt.toPose2d().getRotation().minus(currentGyro).getRadians());

          Pose3d selectedPose = null;


          // if (bestPoseDiff < altPoseDiff && bestPoseDiff < Math.toRadians(20)) {
          //   selectedPose = robotPoseBest;
          // } else if (altPoseDiff < bestPoseDiff && altPoseDiff < Math.toRadians(20)) {
          //   selectedPose = robotPoseAlt;
          // } else {
          //   selectedPose = null;
          // }
          if (bestPoseDiff < altPoseDiff) {
            selectedPose = robotPoseBest;
          } else {
            selectedPose = robotPoseAlt;
          }

          if (selectedPose != null) {
            optEstimation = Optional.of(new EstimatedRobotPose(selectedPose, frame.getTimestampSeconds(), 
                                                          List.of(target), estimator.getPrimaryStrategy()));

          Logger.recordOutput("VisionIOPhotonvision/SelectedPose", selectedPose.toPose2d());
          Logger.recordOutput("VisionIOPhotonvision/SelectedPoseActive", true);                      
        
          }
          
      }}
   
      if (optEstimation.isEmpty()) continue;
      EstimatedRobotPose estimation = optEstimation.get();

      double totalDistance = 0;
      for (PhotonTrackedTarget target : frame.getTargets()) {
        totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
      }

      List<Integer> FIDs = new ArrayList<Integer>();
      boolean badTag = false;
      for (PhotonTrackedTarget target : estimation.targetsUsed) {
        int id = target.getFiducialId();
        if (IntStream.of(VisionConstants.IGNORE_TAGS).anyMatch(x -> x == id)) {
          badTag = true;
          break;
        }
        FIDs.add(id);
      }
      if (badTag) continue;
      allTagIDs.addAll(FIDs);

      var observation =
          new PoseObservation(
              frame.getTimestampSeconds(),
              estimation.estimatedPose,
              estimation.targetsUsed.get(0).poseAmbiguity,
              results.get(frameIndex).targets.size(),
              totalDistance / results.get(frameIndex).targets.size());
      observations.add(observation);
    }

    inputs.observations = observations.toArray(new PoseObservation[observations.size()]);

    inputs.tagIDs = allTagIDs.stream().mapToInt(i -> i).toArray();
  }
}
