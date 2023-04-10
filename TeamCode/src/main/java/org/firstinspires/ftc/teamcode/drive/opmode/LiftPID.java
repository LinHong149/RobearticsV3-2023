package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class LiftPID extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degree = 159936/377; //change this please

    private DcMotorEx arm_motor_right;
    private DcMotorEx arm_motor_left;

    @Override
    public void init(){
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor_left = hardwareMap.get(DcMotorEx.class, "Leftlift");
        arm_motor_right = hardwareMap.get(DcMotorEx.class, "Rightlift");

        arm_motor_left.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){
        controller.setPID(p,i,d);
        int armPosLeft = arm_motor_left.getCurrentPosition();
        int armPosRight = arm_motor_right.getCurrentPosition();
        double pid = controller.calculate(armPosRight, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

        double power = pid + ff;

        arm_motor_left.setPower(power);
        arm_motor_right.setPower(power);

        telemetry.addData("Left position", armPosLeft);
        telemetry.addData("Right position", armPosRight);

        telemetry.addData("target", target);
        telemetry.update();

    }
}
