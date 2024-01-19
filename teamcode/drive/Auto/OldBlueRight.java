package org.firstinspires.ftc.teamcode.drive.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;



@Disabled
@Autonomous(name = "RedLeft58Close", group = "RED")
public class OldBlueRight extends LinearOpMode {
    //armsNStuff arm;
    DcMotor Arm;
    Servo Box;
    CRServo Intake;
    OpenCvCamera webcam;
    GamePropLeft.gamePropPosition propPosition = GamePropLeft.gamePropPosition.LEFT;
    SampleMecanumDrive robot;
    @Override
    public void runOpMode() throws InterruptedException {

        // arm=new armsNStuff(hardwareMap);
        Arm=hardwareMap.get(DcMotor.class, "Arm");
        Box=hardwareMap.get(Servo.class, "Box");
        Intake=hardwareMap.get(CRServo.class, "Intake");
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Box.setPosition(0.93);
        telemetry.addData("Start OpMode", "BLUE LEFT");
        telemetry.update();
        startCamera();
        telemetry.addData("Selected Starting Position", propPosition);
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        while (!isStopRequested() && !opModeIsActive())
        {propPosition = getPropPosition();
            telemetry.addData("Identified Prop Location", propPosition);
            telemetry.update();
        }

        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            webcam.stopStreaming(); //Stop Webcam to preserve Controlhub cycles.
            runAutonoumousMode();
        }
    }

    public void runAutonoumousMode() {

        Pose2d initPose = new Pose2d(0, 0, Math.toRadians(0.0)); // Starting Pose --Update CoOrdinates
        Pose2d itPose = new Pose2d(0, 0, Math.toRadians(0.0));
        Pose2d nitPose = new Pose2d(0, 0, Math.toRadians(0.0));
        Pose2d innitPose = new Pose2d(0, 0, Math.toRadians(0.0)); // Starting Pose --Update CoOrdinates
        Pose2d wallPose1 = new Pose2d(0, 0, 0);
        Pose2d wallPose2 = new Pose2d(0, 0, 0);
        Pose2d purplePixelPose = new Pose2d(0, 0, Math.toRadians(0.0));
        Pose2d leftPose  = new Pose2d(0, 0, Math.toRadians(0.0));
        switch (propPosition) { //UPDATE THESE POSITIONS
            case LEFT:
                telemetry.addData("Left", "Left");
                wallPose1 = new Pose2d(20,-7, Math.toRadians(-90));
                wallPose2 = new Pose2d(48.5,15, Math.toRadians(-90));
                itPose = new Pose2d(10,14, Math.toRadians(-90));
                nitPose = new Pose2d(10,-82, Math.toRadians(-90));
                innitPose = new Pose2d(36,-82, Math.toRadians(-90));
                leftPose = new Pose2d(19,9.5, Math.toRadians(0));
                purplePixelPose = new Pose2d(19.5,9.5, Math.toRadians(0));
                break;
            case CENTER:
                telemetry.addData("Center Position", "Center");
                wallPose1 = new Pose2d(20,9, Math.toRadians(-90));
                wallPose2 = new Pose2d(48,15, Math.toRadians(-90));
                itPose = new Pose2d(10,14, Math.toRadians(-90));
                nitPose = new Pose2d(10,-82, Math.toRadians(-90));
                innitPose = new Pose2d(32.5,-82, Math.toRadians(-90));
                purplePixelPose = new Pose2d(26, -3, Math.toRadians(0.0));
                leftPose = new Pose2d(25, -3, Math.toRadians(0.0));
                break;
            case RIGHT:
                wallPose1 = new Pose2d(20,9, Math.toRadians(-90));
                wallPose2 = new Pose2d(48,15, Math.toRadians(-90));
                itPose = new Pose2d(10,14, Math.toRadians(-90));
                nitPose = new Pose2d(10,-82, Math.toRadians(-90));
                innitPose = new Pose2d(28.3,-82, Math.toRadians(-90));
                purplePixelPose = new Pose2d(29, -6, Math.toRadians(-90.0));
                leftPose = new Pose2d(29, -1, Math.toRadians(-90.0));

                break;

        }

        // Pose2d parkingPose = new Pose2d(24, 70, Math.toRadians(0)); //UPDATE

        telemetry.update();
        sleep(200);

        robot = new SampleMecanumDrive(hardwareMap);
        robot.followTrajectorySequence(robot.trajectorySequenceBuilder(initPose) //Starting Pose
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Arm.setTargetPosition(300);Arm.setPower(0.8);Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);})
                .lineToLinearHeading(leftPose)
                .lineToLinearHeading(purplePixelPose)//Drop Purple Pixel
                .back(5)
                .lineToLinearHeading(wallPose1)
                .waitSeconds(.5)
                .lineToLinearHeading(wallPose2)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Arm.setTargetPosition(374);Arm.setPower(0.8);Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);})
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Intake.setPower(0.75);})
                .back(5)
                .waitSeconds(1.25)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Intake.setPower(0);})
                .lineToLinearHeading(itPose)
                .lineToLinearHeading(nitPose)
                .lineToLinearHeading(innitPose)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Arm.setTargetPosition(4900);Arm.setPower(0.8);Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);})
                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {Box.setPosition(0.43);})
                .waitSeconds(3)
                .forward(3.75)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Intake.setPower(-0.2);})
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Intake.setPower(0);})
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Box.setPosition(0.95);})
                .waitSeconds(1.4)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Arm.setTargetPosition(0);Arm.setPower(0.5);Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);})
                .waitSeconds(4)
                //.UNSTABLE_addTemporalMarkerOffset(0, () -> {225})
                //.UNSTABLE_addTemporalMarkerOffset(0, () -> {0.45;})
                /*.lineToLinearHeading(yellowPixelPose)
                //.waitSeconds(1)
                // .forward(4)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Arm.setTargetPosition(2250);Arm.setPower(0.4);Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);})
                .UNSTABLE_addTemporalMarkerOffset(3, () -> {Box.setPosition(0.45);})
                .waitSeconds(3)
                .forward(4.5)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Intake.setPower(-0.2);})
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Intake.setPower(0);})
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Box.setPosition(0.93);})
                .waitSeconds(0.75)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Arm.setTargetPosition(0);Arm.setPower(0.25);Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);})
                .waitSeconds(4)*/
                .build());

    }
    public void startCamera() {
        //Initialize Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "FrontCam"), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        webcam.setPipeline(new GamePropLeft());
    }

    public GamePropLeft.gamePropPosition getPropPosition() {
        return GamePropLeft.position;
    }

}