package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class x {
    public static void main(String[] args) {

        Pose2d a = new Pose2d(0, 0, new Rotation2d(0));
        Transform2d t = new Transform2d(new Translation2d(1, 1), new Rotation2d());
        Pose2d b = new Pose2d(0, 0, new Rotation2d(Math.PI / 2));

        System.out.println(b.plus(t));
        Translation2d translation = b.transformBy(t).relativeTo(a).getTranslation();
        System.out.println(translation);
        double angle = translation.getAngle().getDegrees();
        System.out.println(angle); // 应该输出 45.0
    }
}
