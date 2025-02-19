package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

/**
 * 管理场地上的目标位置
 */
public class FieldTargets {
    public static Transform2d leftPip = new Transform2d(-0.04592, -0.16429-0.05, Rotation2d.kZero);
    public static Transform3d leftPip3d = new Transform3d(-0.04592, -0.16429, 0.5, Rotation3d.kZero);
    public static Transform2d rightPip = new Transform2d(-0.04592, 0.16429, Rotation2d.kZero);

    // 红方目标位置
    public static final Pose3d[] redStations = {
            aprilTagLayout.getTagPose(1).get(),
            aprilTagLayout.getTagPose(2).get(),
    };
    public static final Pose3d redProcessor = aprilTagLayout.getTagPose(3).get();
    public static final Pose3d[] redReef = {
            aprilTagLayout.getTagPose(10).get(),
            aprilTagLayout.getTagPose(11).get(),
            aprilTagLayout.getTagPose(6).get(),
            aprilTagLayout.getTagPose(7).get(),
            aprilTagLayout.getTagPose(8).get(),
            aprilTagLayout.getTagPose(9).get()
    };

    // 蓝方目标位置
    public static final Pose3d[] blueStations = {
            aprilTagLayout.getTagPose(12).get(),
            aprilTagLayout.getTagPose(13).get()
    };
    public static final Pose3d blueProcessor = aprilTagLayout.getTagPose(16).get();
    public static final Pose3d[] blueReef = {
            aprilTagLayout.getTagPose(21).get(),
            aprilTagLayout.getTagPose(20).get(),
            aprilTagLayout.getTagPose(19).get(),
            aprilTagLayout.getTagPose(18).get(),
            aprilTagLayout.getTagPose(17).get(),
            aprilTagLayout.getTagPose(22).get()
    };

    public CircularTag station;
    public Pose3d processor;
    public CircularTag reef;

    public FieldTargets(Alliance alliance) {
        switch (alliance) {
            case Red:
                station = new CircularTag(redStations);
                processor = redProcessor;
                reef = new CircularTag(redReef);
                break;
            default:
                station = new CircularTag(blueStations);
                processor = blueProcessor;
                reef = new CircularTag(blueReef);
                break;
        }
    }

    public Pose3d update(Pose3d currentPose) {
        return update(currentPose.toPose2d());
    }

    /**
     * 获取最近的目标位置
     * 
     * @param currentPose 当前位置
     * @param targets     目标位置数组
     * @return 最近的目标位置
     */
    public Pose3d update(Pose2d currentPose) {
        CircularTag nearestTarget = reef;
        double minDistance = currentPose.getTranslation().getDistance(reef.getTranslation());

        double distance = currentPose.getTranslation().getDistance(reef.leftTag.getTranslation());
        if (distance < minDistance) {
            minDistance = distance;
            nearestTarget = reef.leftTag;
        }
        distance = currentPose.getTranslation().getDistance(reef.rightTag.getTranslation());
        if (distance < minDistance) {
            minDistance = distance;
            nearestTarget = reef.rightTag;
        }

        reef = nearestTarget;
        return nearestTarget.pose;
    }

    public Pose3d getNearestStation(Pose3d currentPose) {
        if (currentPose.getY() > aprilTagLayout.getFieldWidth() / 2) {
            return station.rightTag.pose;
        } else {
            return station.pose;
        }
    }

    public class CircularTag {

        public Pose3d pose;
        public CircularTag leftTag;
        public CircularTag rightTag;

        public CircularTag(Pose3d... tags) {
            if (tags.length == 1) {
                this.pose = tags[0];
                return;
            }
            this.pose = tags[0];
            CircularTag[] nodes = new CircularTag[tags.length];
            nodes[0] = this;

            // 创建其他节点
            for (int i = 1; i < tags.length; i++) {
                nodes[i] = new CircularTag(tags[i]);
            }

            // 连接所有节点
            for (int i = 0; i < tags.length; i++) {
                nodes[i].rightTag = nodes[(i + 1) % tags.length];
                nodes[i].leftTag = nodes[(i - 1 + tags.length) % tags.length];
            }
        }

        public Translation2d getTranslation() {
            return pose.getTranslation().toTranslation2d();
        }
    }
}