package frc.robot.util;

public class FourBarLinkage {
    // 默认连杆参数
    private final double length1;
    private final double length2;
    private final double length3;
    private final double length4;

    // 缓存一些常用值
    private final double length1Squared;
    private final double length4Squared;
    private final double length2Squared;
    private final double length3Squared;

    // 复用的数组对象，避免重复创建
    private final double[][] A = new double[2][2];
    private final double[] B = new double[2];
    private final double[] omega = new double[2];
    private final double[][] result = new double[2][2];

    // 缓存上一次的结果
    private double lastTheta1;
    private double lastTargetTheta3;

    public FourBarLinkage() {
        this(30, 170, 150, 83);
    }

    public FourBarLinkage(double l1, double l2, double l3, double l4) {
        this.length1 = l1;
        this.length2 = l2;
        this.length3 = l3;
        this.length4 = l4;

        // 预计算平方值
        this.length1Squared = l1 * l1;
        this.length2Squared = l2 * l2;
        this.length3Squared = l3 * l3;
        this.length4Squared = l4 * l4;
    }

    private void solveLinearEquation(double[] x) {
        // 内联2x2线性方程组求解，直接修改传入的数组
        double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
        x[0] = (B[0] * A[1][1] - B[1] * A[0][1]) / det;
        x[1] = (A[0][0] * B[1] - A[1][0] * B[0]) / det;
    }

    private double[][] calculateDisplacementVelocity(double theta1, double omega1) {
        // 计算三角函数值，避免重复计算
        double sinTheta1 = Math.sin(theta1);
        double cosTheta1 = Math.cos(theta1);

        // 计算辅助线，使用预计算的平方值
        double guideLine = Math.sqrt(length1Squared + length4Squared
                - 2 * length1 * length4 * cosTheta1);

        // 计算辅助角
        double temp = length1 * sinTheta1 / guideLine;
        if (temp > 1)
            temp = 1;
        else if (temp < -1)
            temp = -1;
        double guideAngle1 = Math.asin(temp);

        temp = (guideLine * guideLine + length3Squared - length2Squared)
                / (2 * guideLine * length3);
        if (temp > 1)
            temp = 1;
        else if (temp < -1)
            temp = -1;
        double guideAngle2 = Math.acos(temp);

        double theta3 = Math.PI - guideAngle1 - guideAngle2;

        // 计算三角函数值
        double sinTheta3 = Math.sin(theta3);
        double cosTheta3 = Math.cos(theta3);

        temp = (length3 * sinTheta3 - length1 * sinTheta1) / length2;
        if (temp > 1)
            temp = 1;
        else if (temp < -1)
            temp = -1;
        double theta2 = Math.asin(temp);

        // 计算三角函数值
        double sinTheta2 = Math.sin(theta2);
        double cosTheta2 = Math.cos(theta2);

        // 计算角速度，复用数组
        A[0][0] = -length2 * sinTheta2;
        A[0][1] = length3 * sinTheta3;
        A[1][0] = length2 * cosTheta2;
        A[1][1] = -length3 * cosTheta3;

        B[0] = length1 * sinTheta1 * omega1;
        B[1] = -length1 * cosTheta1 * omega1;

        solveLinearEquation(omega);

        // 填充结果数组
        result[0][0] = theta2;
        result[0][1] = theta3;
        result[1][0] = omega[0];
        result[1][1] = omega[1];

        return result;
    }

    public double  calculate(double targetTheta3) {
        return calculate(targetTheta3, 1.0, 1e-6, 50);
    }

    public double calculate(double targetTheta3, double omega1,
            double tolerance, int maxIterations) {
        // 使用上一次的结果作为初始值
        double theta1 = lastTheta1;
        if (Math.abs(targetTheta3 - lastTargetTheta3) > Math.PI / 4) {
            theta1 = 130 * Math.PI / 180;
        }
        double bestTheta1 = theta1;
        double minError = Double.MAX_VALUE;

        for (int i = 0; i < maxIterations; i++) {
            try {
                double[][] result = calculateDisplacementVelocity(theta1, omega1);
                double f = result[0][1] - targetTheta3;
                double df = result[1][1] / omega1;

                // 记录最佳结果
                double currentError = Math.abs(f);
                if (currentError < minError) {
                    minError = currentError;
                    bestTheta1 = theta1;
                }

                // 处理导数接近零的情况
                if (Math.abs(df) < 1e-10) {
                    if (i < maxIterations / 2) {
                        theta1 = Math.PI * (2.0 * i / maxIterations);
                        continue;
                    } else {
                        break;
                    }
                }

                // 牛顿迭代步骤
                double delta = f / df;
                // 限制步长，提高稳定性
                if (Math.abs(delta) > Math.PI / 2) {
                    delta = Math.PI / 2 * Math.signum(delta);
                }

                theta1 = theta1 - delta;
                // 保持在[0, 2π]范围内
                while (theta1 < 0)
                    theta1 += 2 * Math.PI;
                while (theta1 >= 2 * Math.PI)
                    theta1 -= 2 * Math.PI;

                if (Math.abs(delta) < tolerance) {
                    if (Math.abs(f) < tolerance) {
                        return theta1;
                    }
                }
            } catch (Exception e) {
                if (i < maxIterations / 2) {
                    theta1 = Math.PI * (2.0 * i / maxIterations);
                } else {
                    break;
                }
            }
        }

        // 如果没有找到精确解，但有接近的解，返回最佳结果
        return minError < tolerance * 10 ? bestTheta1 : 2.500708;
    }
}