// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

public class OptimizePose {
    public static double softLimitOptimize(double targetPose, double softLimitMin, double softLimitMax)
    {
        double correctPose = targetPose;
        if (targetPose > softLimitMax) {
            correctPose = 180 - targetPose;
        }
        else if (targetPose < softLimitMin) {
            correctPose = Math.abs(targetPose + 180);
        }
        return correctPose;
    }
}
