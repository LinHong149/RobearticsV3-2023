package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {

    private PIDController controller;

    public static double p = 0.003, i = 0, d = 0.001;
    public static double f = 0.1;

    public int target = 0;

    private final double ticks_in_degree = 159936/377; //change this please



    public DcMotorEx lr = null;
    public DcMotorEx ll = null;
    public Servo LArmServo = null;
    public Servo RArmServo = null;
    public Servo OCServo = null;
    public Servo RotateServo = null;
    public DcMotorEx frontleftDrive = null;
    public DcMotorEx frontrightDrive = null;
    public DcMotorEx backleftDrive = null;
    public DcMotorEx backrightDrive = null;

    int OP = 0; //Original position
    double OPSA = 0.85; //Original servo arm position
    int LJ = 800; //Low junction
    int MJ = 1300; //Medium junction
    int HJ = 2000; //High junction
    double FLJS = 0.7; //Front Low junction servo position
    double FMJS = 0.6; //Front Medium junction servo position
    double FHJS = 0.5; //Front High junction servo position
    double LJS = 0.2; //Back Low junction servo position
    double MJS = 0.3; //Back Medium junction servo position
    double HJS = 0.3; //Back High junction servo position
    double SRP = 0.95; //Servo rotate position
    double SRO = 0.2; //Servo rotate original position
    double open = 1; //Servo position for open claw
    double close = 0; //Servo position for close claw
    double x1;
    double y1;
    double rx;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();



    @Override
    public void runOpMode() { //init

        frontleftDrive = hardwareMap.get(DcMotorEx.class, "fl");
        frontrightDrive = hardwareMap.get(DcMotorEx.class, "fr");
        backleftDrive = hardwareMap.get(DcMotorEx.class, "bl");
        backrightDrive = hardwareMap.get(DcMotorEx.class, "br");
        lr = hardwareMap.get(DcMotorEx.class, "Rightlift");
        ll = hardwareMap.get(DcMotorEx.class, "Leftlift");


        LArmServo = hardwareMap.get(Servo.class, "LArmServo");
        RArmServo = hardwareMap.get(Servo.class, "RArmServo");
        OCServo = hardwareMap.get(Servo.class, "OpenCloseServo");
        RotateServo = hardwareMap.get(Servo.class, "RotateServo");



        frontleftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontrightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backrightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backleftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        ll.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        ll.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontleftDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontrightDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backrightDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backleftDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        frontleftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontrightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backleftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backrightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        ll.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        RotateServo.setDirection(Servo.Direction.REVERSE);
        LArmServo.setDirection(Servo.Direction.REVERSE);
        ll.setDirection(DcMotorSimple.Direction.REVERSE);
        frontrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDController(p, i, d);


        waitForStart();
        OCServo.setPosition(open);
        LArmServo.setPosition(OPSA);
        RArmServo.setPosition(OPSA);

        while (opModeIsActive()) {
            controller.setPID(p,i,d);

            int motor = lr.getCurrentPosition();
            double pid = controller.calculate(motor, target);
            //double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;
            double ff = 0;

            double power = pid + ff;


            ll.setPower(power);
            lr.setPower(power);

            //DRIVING CODE
             x1 = gamepad1.left_stick_x;
             y1 = -gamepad1.left_stick_y;
             rx = gamepad1.right_stick_x;

             double denominator = Math.max(Math.abs(y1) + Math.abs(x1) + Math.abs(rx), 1);
             double frontLeftPower = (y1 + x1 + rx) / denominator;
             double backLeftPower = (y1 - x1 + rx) / denominator;
             double frontRightPower = (y1 - x1 - rx) / denominator;
             double backRightPower = (y1 + x1 - rx) / denominator;

             frontleftDrive.setPower(frontLeftPower * 0.75);
             frontrightDrive.setPower(frontRightPower * 0.75);
             backleftDrive.setPower(backLeftPower * 0.75);
             backrightDrive.setPower(backRightPower * 0.75);

             //MANUAL
            target += Math.round(gamepad1.right_trigger - gamepad1.left_trigger) * 20;

            //FRONT LOW JUNCTION
            if (gamepad1.a && gamepad1.back != true) {
                target = LJ;

                LArmServo.setPosition(FLJS);
                RArmServo.setPosition(FLJS);
                RotateServo.setPosition(SRO);
            }

            //FRONT MEDIUM JUNCTION
            else if (gamepad1.x && gamepad1.back != true) {
                target = MJ;

                LArmServo.setPosition(FMJS);
                RArmServo.setPosition(FMJS);
                RotateServo.setPosition(SRO);
            }

            //FRONT HIGH JUNCTION
            else if (gamepad1.y && gamepad1.back != true) {
                target = HJ;

                LArmServo.setPosition(FHJS);
                RArmServo.setPosition(FHJS);
                RotateServo.setPosition(SRO);
            }

            //BACK LOW JUNCTION
            else if (gamepad1.a && gamepad1.back) {
                target = LJ;

                LArmServo.setPosition(LJS);
                RArmServo.setPosition(LJS);
                RotateServo.setPosition(SRP);
            }

            //BACK MEDIUM JUNCTION
            else if (gamepad1.x && gamepad1.back) {
                target = MJ;

                LArmServo.setPosition(MJS);
                RArmServo.setPosition(MJS);
                RotateServo.setPosition(SRP);
            }

            //BACK HIGH JUNCTION
            else if (gamepad1.y && gamepad1.back) {
                target = HJ;

                LArmServo.setPosition(HJS);
                RArmServo.setPosition(HJS);
                RotateServo.setPosition(SRP);
            }

            //RETURN TO ORIGINAL POSITION
            else if (gamepad1.right_bumper){
                target = OP;

                LArmServo.setPosition(OPSA);
                RArmServo.setPosition(OPSA);
                RotateServo.setPosition(SRO);

            }

            //CLAW
            if (gamepad1.b && OCServo.getPosition() < 1){
                telemetry.addLine("iusehfiushefiuhsef");
                OCServo.setPosition(open);
                sleep(300);
            }

           else if (gamepad1.b && OCServo.getPosition() > 0.9){
                OCServo.setPosition(close);
                sleep(300);
            }

            target = Math.max(Math.min(target, 2000), 0);


            telemetry.addData("Servo Position", OCServo.getPosition());
            telemetry.addData("target", target);
            telemetry.addData("position", motor);
            telemetry.addData("power", power);
            telemetry.update();



        }


    }





}

