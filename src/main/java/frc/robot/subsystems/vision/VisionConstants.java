// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
        // AprilTag布局
        public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // 相机名称，必须与协处理器上配置的名称匹配
        public static String camera0Name = "Arducam_OV9281_Front";// "Arducam_OV9281_USB_Camera";
        public static String camera1Name = "Arducam_OV9281_Back";
        public static String camera2Name = "Arducam_OV9281_Left";

        // 机器人到相机的变换
        // (Limelight不使用此配置，请在web界面中配置)
        public static Transform3d robotToCamera0 = new Transform3d(0.251435, -0.240755, 0.204634,
                        new Rotation3d(0.0, -0.279253, 0.4715375));
        public static Transform3d robotToCamera1 = new Transform3d(-0.143012, 0.165, 1.076118,
                        new Rotation3d(0.0, -0.174533, Math.PI));
        public static Transform3d robotToCamera2 = new Transform3d(0.322033 ,  0.312082 , 0.204634,
                        new Rotation3d(0.0, -0.279253, -1.099557));

        // 基本过滤阈值
        public static double maxAmbiguity = 0.3;
        public static double maxZError = 0.75;

        // 标准差基线，适用于1米距离和1个标签
        // (根据距离和标签数量自动调整)
        public static double linearStdDevBaseline = 0.02; // 米
        public static double angularStdDevBaseline = 0.06; // 弧度

        // 每个相机的标准差乘数
        // (调整以使某些相机比其他相机更可信)
        public static double[] cameraStdDevFactors = new double[] {
                        1.0, // 相机 0
                        1.0, // 相机 1
                        1.0
        };

        // 应用于MegaTag 2观测的乘数
        public static double linearStdDevMegatag2Factor = 0.5; // 比完整3D解算更稳定
        public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // 无旋转数据可用
}
