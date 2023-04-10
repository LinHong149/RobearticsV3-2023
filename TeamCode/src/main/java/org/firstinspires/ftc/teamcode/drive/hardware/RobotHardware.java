package org.firstinspires.ftc.teamcode.drive.hardware;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class RobotHardware {
//    Wheels
    public DcMotorEx fl;
    public DcMotorEx fr;
    public DcMotorEx bl;
    public DcMotorEx br;

//    Arm Servos
    public Servo armLeft;
    public Servo armRight;
    public Servo clawRotation;
    public Servo claw;

//    Lift Motors
    public DcMotorEx liftLeft;
    public DcMotorEx liftRight;


    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        br = hardwareMap.get(DcMotorEx.class, "br");
        br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        armLeft = hardwareMap.get(Servo.class, "armLeft");
        armLeft.setDirection(Servo.Direction.REVERSE);
        armRight = hardwareMap.get(Servo.class, "armRight");
        clawRotation = hardwareMap.get(Servo.class, "clawRotation");
        claw = hardwareMap.get(Servo.class, "claw");

        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        liftRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    private static RobotHardware instance = null;

    public boolean enabled;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }
}
