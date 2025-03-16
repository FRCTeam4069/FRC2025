package frc.robot.subsystems.swerve;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionBetter {
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    private Optional<EstimatedRobotPose> pose;
    private PhotonPipelineResult result;

    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    private Matrix<N3, N1> bad = VecBuilder.fill(6.0, 6.0, 5.0);
    private Matrix<N3, N1> good = VecBuilder.fill(0.6, 0.6, 2.0);

    public VisionBetter(String name, Transform3d robotToCam, Pose2d startingPose) {
        this(name, robotToCam, startingPose, VecBuilder.fill(6.0, 6.0, 5.0), VecBuilder.fill(0.6, 0.6, 2.0));
    }

    public VisionBetter(String name, Transform3d robotToCam, Pose2d startingPose, Matrix<N3, N1> bad, Matrix<N3, N1> good) {
        camera = new PhotonCamera(name);
        poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        poseEstimator.setLastPose(startingPose);

        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        this.bad = bad;
        this.good = good;
    }

    public PhotonPipelineResult getLatestPose() {
        result = camera.getLatestResult();
        return result;
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        result = getLatestPose();
        if (!result.hasTargets()) {
            return Optional.empty();
        }

        if (result.targets.size() >= 2) {
            pose = poseEstimator.update(getLatestPose());
            return pose;
        }

        if (getDistance(result.targets.get(0).getBestCameraToTarget()) < closeEnough) {
            pose = poseEstimator.update(getLatestPose());
            return pose;
        }

        return Optional.empty();
    }

    private double lowestDistance = Double.MAX_VALUE;

    private static final double tooFar = 1.3;
    private static final double closeEnough = 1.7;

    public Matrix<N3, N1> getStdDeviations() {
        if (!pose.isPresent()) {
            return bad;
        } 

        lowestDistance = Double.MAX_VALUE;

        pose.get().targetsUsed.forEach(target -> {
            var transform = target.getBestCameraToTarget();
            double distance = getDistance(transform);
            if (distance < lowestDistance) {
                lowestDistance = distance;
            }
        });

        if (lowestDistance > tooFar) {
            return bad;
        }

        return good;
    }

    public double getDistance(Transform3d transform) {
        var x = transform.getX();
        var y = transform.getY();
        var z = transform.getZ();
        return Math.sqrt(x*x + y*y + z*z);
    }

}
