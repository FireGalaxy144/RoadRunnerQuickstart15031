package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.subsystems.arm;
import org.firstinspires.ftc.teamcode.subsystems.claw;
import org.firstinspires.ftc.teamcode.subsystems.drive;
import org.firstinspires.ftc.teamcode.subsystems.lift;
import org.firstinspires.ftc.teamcode.subsystems.wrist;
import org.firstinspires.ftc.teamcode.util.MercurialAction;
import org.firstinspires.ftc.teamcode.util.SilkRoad;
import org.firstinspires.ftc.teamcode.util.WaitAction;

import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent;
import dev.frozenmilk.dairy.core.util.features.BulkRead;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.util.Wait;

@Autonomous(name = "2 sample")
@SilkRoad.Attach
@Mercurial.Attach
@lift.Attach
@claw.Attach
@wrist.Attach
@arm.Attach
@BulkRead.Attach
@drive.Attach
public class Auto extends OpMode {
    private final Pose2d initialPose = new Pose2d(-24, -62.5, Math.toRadians(180));
    private Action path0, path1, path2, path3;
    private SparkFunOTOSDrive drive;

    @Override
    public void init() { drive = new SparkFunOTOSDrive(hardwareMap, initialPose); }

    @Override
    public void init_loop(){

        TrajectoryActionBuilder tab0 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(-50, -58), Math.toRadians(225));
        TrajectoryActionBuilder tab1 = drive.actionBuilder(new Pose2d(-50, -58, Math.toRadians(225)))
                .setReversed(true)
                .splineTo(new Vector2d(-34, -40), Math.toRadians(0))
                .setReversed(false)
                .splineTo(new Vector2d( -46, -60), Math.toRadians(90))
                .lineToY(62);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-46, -62, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(-34, -48), Math.toRadians(0))
                .setReversed(false)
                .splineTo(new Vector2d(-58, -49), Math.toRadians(225));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-58, -49, Math.toRadians(225)))
                .setReversed(true)
                .splineTo(new Vector2d(-34, -48), Math.toRadians(0))
                .setReversed(false)
                .splineTo(new Vector2d(-30, -34), Math.toRadians(0));
        path0 = tab0.build();
        path1 = tab1.build();
        path2 = tab2.build();
        path3 = tab3.build();
    }

    @Override
    public void start(){
        SilkRoad.RunAsync(
                new SequentialAction(
                        new MercurialAction(new Parallel(lift.goTo(4500), wrist.wristFlat(), claw.closeClaw(), arm.armUp())),
                        path0,
                        new MercurialAction(claw.openClaw()),
                        new SleepAction(1),
                        new MercurialAction(new Parallel(claw.closeClaw(), lift.goTo(0), wrist.wristFlat(), arm.armStow())),
                        path1,
                        new MercurialAction(new Parallel(lift.goTo(0), arm.armOut(), claw.openClaw())),
                        new SleepAction(2),
                        new MercurialAction(wrist.wristDown()),
                        new SleepAction(1),
                        new MercurialAction(claw.closeClaw()),
                        new SleepAction(1),
                        new MercurialAction(new Parallel(arm.armUp(), wrist.wristFlat(), lift.goTo(4500))),
                        path2,
                        new MercurialAction(claw.openClaw()),
                        new SleepAction(1),
                        new MercurialAction(new Parallel(arm.armStow(), wrist.wristDown(), claw.closeClaw(), lift.goTo(0))),
                        path3,
                        new SleepAction(1),
                        new MercurialAction(new Parallel(arm.armUp(), wrist.wristFlat()))
                )
        );
    }

    @Override
    public void loop() {
    }
}
