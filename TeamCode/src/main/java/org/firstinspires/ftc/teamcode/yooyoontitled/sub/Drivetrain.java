package org.firstinspires.ftc.teamcode.yooyoontitled.sub;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;

public class Drivetrain extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public void drive(double x, double y, double rot) {
        robot.follower.setTeleOpDrive(x, y, rot, true);
    }

    public void follow(Path path) {
        robot.follower.followPath(path);
    }

    public void follow(PathChain chain) {
        robot.follower.followPath(chain);
    }

    public void updatePose() {
        robot.follower.update();
    }

    public Pose getPose() {
        return robot.follower.getPose();
    }

    public void relocalize(Pose pose) {
        robot.follower.setPose(pose);
    }

    @Override
    public void periodic() {
        robot.follower.update();
    }
}
