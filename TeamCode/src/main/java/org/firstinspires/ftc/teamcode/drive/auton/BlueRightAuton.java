///*
// * Copyright (c) 2021 OpenFTC Team
// *
// * Permission is hereby granted, free of charge, to any person obtaining a copy
// * of this software and associated documentation files (the "Software"), to deal
// * in the Software without restriction, including without limitation the rights
// * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// * copies of the Software, and to permit persons to whom the Software is
// * furnished to do so, subject to the following conditions:
// *
// * The above copyright notice and this permission notice shall be included in all
// * copies or substantial portions of the Software.
// * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// * SOFTWARE.
// */
//
//package org.firstinspires.ftc.teamcode.drive.auton;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.openftc.apriltag.AprilTagDetection;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import java.util.ArrayList;
//
//@Config
//@Autonomous (group = "drive")
//public class BlueRightAuton extends LinearOpMode
//{
//    OpenCvCamera camera;
//    AprilTagDetectionPipeline aprilTagDetectionPipeline;
//
//    static final double FEET_PER_METER = 3.28084;
//
//    // Lens intrinsics
//    // UNITS ARE PIXELS
//    // NOTE: this calibration is for the C920 webcam at 800x448.
//    // You will need to do your own calibration for other configurations!
//    double fx = 578.272;
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;
//    // Initializing the lift and open/close servo
//    public DcMotorEx lift;
//    private Servo ocServo;
//    // Getting the hardware map for lift and open/close servo
//    public BlueRightAuton(HardwareMap hardwareMap) {
//        lift = hardwareMap.get(DcMotorEx.class, "lift");
//        ocServo = hardwareMap.get(Servo.class, "openClose");
//
//        // Adding ZeroPowerBehavior
//        lift.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
//        // Set motor to run without encoder
//        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//
//
//    // UNITS ARE METERS
//    double tagsize = 0.166;
//
//  // Tag ID 1,2,3 from the 36h11 family
//    int LEFT = 1;
//    int MIDDLE = 2;
//    int RIGHT = 3;
//    AprilTagDetection tagOfInterest = null;
//
//    @Override
//    public void runOpMode() {
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//
//        camera.setPipeline(aprilTagDetectionPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
//
//        telemetry.setMsTransmissionInterval(50);
//
//        /*
//         * The INIT-loop:
//         * This REPLACES waitForStart!
//         * This is as soon as you press the button, not when you start
//         */
//        while (!isStarted() && !isStopRequested()) {
//
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//            if (currentDetections.size() != 0) {
//                boolean tagFound = false;
//
//                for (AprilTagDetection tag : currentDetections) {
//                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//
//                if (tagFound) {
//                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                    tagToTelemetry(tagOfInterest);
//                } else {
//                    telemetry.addLine("Don't see tag of interest :(");
//
//                    if (tagOfInterest == null) {
//                        telemetry.addLine("(The tag has never been seen)");
//                    } else {
//                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                        tagToTelemetry(tagOfInterest);
//                    }
//                }
//
//            } else {
//                telemetry.addLine("Don't see tag of interest :(");
//
//                if (tagOfInterest == null) {
//                    telemetry.addLine("(The tag has never been seen)");
//                } else {
//                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//
//            }
//
//            telemetry.update();
//            sleep(20);
//        }
//
//        /*
//         * The START command just came in: now work off the latest snapshot acquired
//         * during the init loop.
//         */
//
//        /* Update the telemetry */
//        if (tagOfInterest != null) {
//            telemetry.addLine("Tag snapshot:\n");
//            tagToTelemetry(tagOfInterest);
//            telemetry.update();
//        } else {
//            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
//            telemetry.update();
//        }
//        // Getting the hardwaremap
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        /* Actually do something useful */
//        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
//            // Getting the hardwaremap
//
//            //trajectory
//            // checking for how fast it is
////            lift.setTargetPosition(3700); //lifts all the way up
////            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            lift.setPower(1);
//
//            //code for parking only
//
//            Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
//                    .forward(10)
//                    .build();
//
//            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                    .strafeLeft(5)
//                    .build();
////
////            drive.followTrajectory(traj1);
////            drive.followTrajectory(traj2);
//
//            // --Actual code--
//
//
////            Trajectory cone1 = drive.trajectoryBuilder(new Pose2d()) // True is to make it reversed
////                    .lineToLinearHeading(new Pose2d(0, -61, Math.toRadians(270))) //goes back to line
////                    .splineTo(new Vector2d(9, -65), Math.toRadians(0)) //goes to high junction
////                    .build();
////
////            Trajectory reverse_to_line = drive.trajectoryBuilder(cone1.end(), true)
////                    .splineTo(new Vector2d(0, -60), Math.toRadians(0)) //goes back to line (reversed)
////                    .build();
////
////            Trajectory grabbing_cone = drive.trajectoryBuilder(reverse_to_line.end(), true)
////                    .splineTo(new Vector2d(-70, 12), Math.toRadians(0)) //goes to cones area
////                    .build();
////
////            Trajectory high_junction = drive.trajectoryBuilder(grabbing_cone.end())
////                    .splineTo(new Vector2d(9, -65), Math.toRadians(0)) //goes to the high junction
////                    .build();
////
////            ocServo.setPosition(0.8);
////            lift.setTargetPosition(3700);
////            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            lift.setPower(1);
////            while (lift.isBusy()) {
////                drive.followTrajectory(cone1);
////            }
////            ocServo.setPosition(1);
////            lift.setTargetPosition(0); //the correct value for this
////            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            lift.setPower(-1);
////            while (lift.isBusy()) {
////                drive.followTrajectory(reverse_to_line);
////            }
////            drive.followTrajectory(grabbing_cone);
////            ocServo.setPosition(0.8);
//
//            //This is for one cycle lol just to check values and everything, if it doesnt work (many reasons), then we go parking only until it's correct.
//
//
//
//
//
//
//
////           drive.followTrajectory(myTrajectory);
//
//       }else if(tagOfInterest.id == MIDDLE){
//           //trajectory
//
//           //code for parking only
//           Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
//                   .forward(10)
//                   .build();
//
//           drive.followTrajectory(traj1);
//
//
//       }else{
//           //trajectory
//
//           //code for parking only
//           Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
//                   .forward(10)
//                   .build();
//           Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                   .strafeRight(5)
//                   .build();
//
//           drive.followTrajectory(traj1);
//           drive.followTrajectory(traj2);
//       }
//
//
//        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
//        while (opModeIsActive()) {sleep(20);}
//    }
//
//    void tagToTelemetry(AprilTagDetection detection)
//    {
//        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//    }
//}