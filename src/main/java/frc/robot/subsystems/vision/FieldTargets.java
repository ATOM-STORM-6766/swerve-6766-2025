package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

/**
 * 管理场地上的目标位置
 */
public class FieldTargets {

    // 目标类型枚举
    public enum TargetType {
        STATION, PROCESSOR, REEF
    }

    // 红方目标位置
    public static final Pose3d[] redStations = {
            aprilTagLayout.getTagPose(1).get(),
            aprilTagLayout.getTagPose(2).get(),
    };
    public static final Pose3d redProcessor = aprilTagLayout.getTagPose(3).get();
    public static final Pose3d[] redReef = {
            aprilTagLayout.getTagPose(6).get(),
            aprilTagLayout.getTagPose(7).get(),
            aprilTagLayout.getTagPose(8).get(),
            aprilTagLayout.getTagPose(9).get(),
            aprilTagLayout.getTagPose(10).get(),
            aprilTagLayout.getTagPose(11).get()
    };

    // 蓝方目标位置
    public static final Pose3d[] blueStations = {
            aprilTagLayout.getTagPose(12).get(),
            aprilTagLayout.getTagPose(13).get()
    };
    public static final Pose3d blueProcessor = aprilTagLayout.getTagPose(16).get();
    public static final Pose3d[] blueReef = {
            aprilTagLayout.getTagPose(17).get(),
            aprilTagLayout.getTagPose(18).get(),
            aprilTagLayout.getTagPose(19).get(),
            aprilTagLayout.getTagPose(20).get(),
            aprilTagLayout.getTagPose(21).get(),
            aprilTagLayout.getTagPose(22).get()
    };

    /**
     * 根据位置自动确定联盟颜色
     * @param pose 当前位置
     * @return 联盟颜色
     */
    public static Alliance getAlliance(Pose3d pose) {
        return pose.getX() < aprilTagLayout.getFieldLength() / 2 ? Alliance.Blue : Alliance.Red;
    }

    /**
     * 获取指定联盟和类型的目标
     * @param alliance 联盟颜色
     * @param type 目标类型
     * @return 目标位置数组或单个位置
     */
    public static Object getTargets(Alliance alliance, TargetType type) {
        return switch (type) {
            case STATION -> alliance == Alliance.Blue ? blueStations : redStations;
            case PROCESSOR -> alliance == Alliance.Blue ? blueProcessor : redProcessor;
            case REEF -> alliance == Alliance.Blue ? blueReef : redReef;
        };
    }

    /**
     * 获取最近的目标位置
     * @param currentPose 当前位置
     * @param targets 目标位置数组
     * @return 最近的目标位置
     */
    public static Pose3d getNearestTarget(Pose3d currentPose, Pose3d[] targets) {
        double minDistance = Double.POSITIVE_INFINITY;
        Pose3d nearestTarget = null;

        for (Pose3d target : targets) {
            double distance = currentPose.getTranslation().getDistance(target.getTranslation());
            if (distance < minDistance) {
                minDistance = distance;
                nearestTarget = target;
            }
        }
        return nearestTarget;
    }
} 