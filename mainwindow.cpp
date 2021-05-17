#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "camera.h"
#include <opencv2/surface_matching/ppf_helpers.hpp>
#include <cmath>
using namespace Convert;
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    init();
}

void MainWindow::init()
{
    ui->setupUi(this);
    ReadSettings();
    //UDP Connection Initialize
    motoudp = new MotoUDP(QHostAddress(ui->lineEditRobotIP->text()),10040);
    connect(motoudp, &MotoUDP::triggerPPF, this, &MainWindow::triggerByRobot);
    connect(motoudp, &MotoUDP::triggerGripper, this, &MainWindow::triggerGripperByRobot);
    motoudp->start();



    if(camFile.isNull()||camFile.isEmpty()||!QFile(camFile).exists()){
        cout << "Please set your camera parameters file (.json) " << endl;
        return;
    }
    camera = new Camera(640, 360, 1280, 720, 30,ui->lineEditCameraParam->text().toStdString());
    connect(camera, &Camera::framesReady, this, &MainWindow::receiveFrame);
    connect(camera, &Camera::pclReady, this, &MainWindow::receivePcl);
    connect(camera, &Camera::pointCloudReady, this, &MainWindow::receivePointCloud);
    camera->maxRange = ui->verticalSliderMax->value()/1000.0;
    camera->minRange = ui->verticalSliderMin->value()/1000.0;
    connect(camera,&Camera::connected,this,&MainWindow::readyCameraStatus);
    camera->start();

    viewer = new pclViewer(ui->qvtkWidget);
    viewer_ = new pclViewer(ui->qvtkWidgetModel);


    servo = new ServoControl();

    if(modelPath.isNull()||modelPath.isEmpty()||!QFile(modelPath).exists()){
        cout << "Please set your model file (.ply) " << endl;
        return;
    }
    ppf = new PPF(modelPath.toStdString());
    ppf->start();
    connect(ppf,&PPF::complete,this,&MainWindow::readyPPFStatus);
    if(yoloPath.isNull()||yoloPath.isEmpty()||QDir(yoloPath).isEmpty()){
        cout << "Please set your Yolo config path (include .cfg .weights .names) " << endl;
        return;
    }
    yolo = new Yolo(yoloPath.toStdString());


}

MainWindow::~MainWindow()
{
    WriteSettings();
    delete ui;
}

void MainWindow::choosePointCloud(const vector<Yolo::yoloResult> & results,const Mat& depth_image,const rs2_intrinsics& instrict, Mat &mat)
{
    cout << "choosePointCloud" << endl;
    if(results.size()==0) return;
    vector<cv::Mat>* roi = new vector<cv::Mat>();
    vector<float> depth_medium;
    roi->resize(results.size());
    depth_medium.resize(results.size());
    for (size_t i=0;i<results.size();i++) {
        int sum=0;
        depth_medium.at(i)=0;
        roi->at(i).create(results.at(i).pixW*results.at(i).pixH, 3, CV_32FC1);
        for(int j=results.at(i).y;j<results.at(i).y+results.at(i).pixH;j++){
            for(int k=results.at(i).x;k<results.at(i).x+results.at(i).pixW;k++){
                //cout << depth_image.at<uint16_t>(j,k) << " ";
                if(depth_image.at<uint16_t>(j,k)>0){
                    float z = depth_image.at<uint16_t>(j,k)*1e-5;//depth unit = 1e-5
//                    cout << z << " ";
                    //instrict.fx fy ppx ppy
                    roi->at(i).at<float>(sum,0) =  (float)(k-instrict.ppx)*z/instrict.fx;
                    roi->at(i).at<float>(sum,1) = (float)(j-instrict.ppy)*z/instrict.fy;
                    roi->at(i).at<float>(sum,2) = z;
                    depth_medium.at(i) += z;
                    sum++;
                    //cout << " sum  " << i << " " <<  sum ;
                }
            }
        }
      depth_medium.at(i) = depth_medium.at(i)/sum;
      cout << "depth_medium.at(i)  " << i << " " <<  depth_medium.at(i) << endl;
      cout << "sum  " << i << " " <<  sum << endl;
      roi->at(i).resize(sum);
    }
    mat = roi->at(std::distance(depth_medium.begin(),std::min_element(depth_medium.begin(), depth_medium.end())));
    cout << "finish choosePointCloud" << endl;
    delete roi;
}

void MainWindow::displayPose(const Matx44d &ready_pose,const Matx44d &pick_pose)
{
    Vec3d ready_pos(ready_pose(0,3),ready_pose(1,3),ready_pose(2,3));
    ready_pos = ready_pos*1000.0;
    Vec3d pick_pos(pick_pose(0,3),pick_pose(1,3),pick_pose(2,3));
    pick_pos = pick_pos*1000.0;
    Mat temp_ready(ready_pose);
    temp_ready.convertTo(temp_ready,CV_32FC1);
    Mat rot_ready(temp_ready.colRange(0,3).rowRange(0,3));
    Vec3f ready_rotation = Convert::rotationMatrixToEulerAngles(rot_ready)*180/M_PI;
    Mat temp_pick(pick_pose);
    temp_pick.convertTo(temp_pick,CV_32FC1);
    Mat rot_pick(temp_pick.colRange(0,3).rowRange(0,3));
    Vec3f pick_rotation = Convert::rotationMatrixToEulerAngles(rot_ready)*180/M_PI;

    ui->doubleSpinBoxX1->setValue(ready_pos[0]);
    ui->doubleSpinBoxY1->setValue(ready_pos[1]);
    ui->doubleSpinBoxZ1->setValue(ready_pos[2]);
    ui->doubleSpinBoxRX1->setValue(double(ready_rotation[0]));
    ui->doubleSpinBoxRY1->setValue(double(ready_rotation[1]));
    ui->doubleSpinBoxRZ1->setValue(double(ready_rotation[2]));

    ui->doubleSpinBoxX2->setValue(pick_pos[0]);
    ui->doubleSpinBoxY2->setValue(pick_pos[1]);
    ui->doubleSpinBoxZ2->setValue(pick_pos[2]);
    ui->doubleSpinBoxRX2->setValue(double(pick_rotation[0]));
    ui->doubleSpinBoxRY2->setValue(double(pick_rotation[1]));
    ui->doubleSpinBoxRZ2->setValue(double(pick_rotation[2]));


}

void MainWindow::fail()
{
    ui->radioButtonGood->setChecked(true);
    ui->radioButtonFail->setChecked(true);
    motoudp->fail = true;
    motoudp->good = true;
    motoudp->triggerDone = true;
    ui->spinBoxCount->setValue(count);
    ui->radioButtonTrigger->setChecked(false);
    camera->isBlock = false;
    return;
}

void MainWindow::receiveFrame(QImage rgb)
{
    QImage input = rgb;
    if(boudingBox.size()>0){
        for (int i=0;i<boudingBox.size();i++) {
            struct Yolo::yoloResult yolo = boudingBox.at(i);
            QPainter qPainter(&input);
            QPen pen;
            pen.setBrush(Qt::NoBrush);
            //qPainter.setBrush(Qt::NoBrush);
            pen.setColor(Qt::green);
            pen.setWidth(5);

            //qPainter.setPen(Qt::green);
            qPainter.setPen(pen);
            qPainter.drawRect(yolo.x,yolo.y,yolo.pixW,yolo.pixH);
            qPainter.drawText(yolo.x,yolo.y,QString("ID: ")+QString::fromStdString(std::to_string(yolo.classID)));
            qPainter.end();
        }
    }
    ui->labelImage->setPixmap(QPixmap::fromImage(input).scaled(ui->labelImage->size().width(),ui->labelImage->size().height(),Qt::KeepAspectRatio));

}

void MainWindow::receivePcl(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointcloud)
{
    viewer->displayPCL(pointcloud);
}

void MainWindow::receivePointCloud()
{
    //imshow("depth receive",camera->depth_image);
    if(isTrigger){
        isTrigger = false;
        camera->isBlock = true;
        ui->radioButtonGood->setChecked(false);
        if(isReady){
            ppf_match_3d::Pose3DPtr result;
            yolo->computeYolo(camera->rgb_image,boudingBox);
            cv::Mat input;
            choosePointCloud(boudingBox,camera->depth_image,camera->inrist,input);
            writePLY(input,"testYolo.ply");
            cv::Mat pc;
            if(input.rows==0) {
                cout << "There is no object in this scene!" << endl;
                fail();
                return;
            }
            if(!ppf->caculatePPF(input,result,pc)){
                cout << "Cannot caculate PPF!" << endl;
                fail();
                return;
            }
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_(new pcl::PointCloud<pcl::PointXYZRGBA>);
            mat2Pcl(pc,pcl_);
            Vec3d xa,ya,za;
            Vec3d zy(result->pose.col(2).row(0).val[0],result->pose.col(2).row(1).val[0],result->pose.col(2).row(2).val[0]);
            xa = zy.cross(Vec3d(0,0,-1));
            xa = xa/norm(xa);
            za = zy.cross(xa);
            za = za/norm(za);
            ya = za.cross(xa);
            ya = ya/norm(ya);

            Matx44d oMg{xa[0],ya[0],za[0],result->pose.col(3).row(0).val[0],
                        xa[1],ya[1],za[1],result->pose.col(3).row(1).val[0],
                        xa[2],ya[2],za[2],result->pose.col(3).row(2).val[0],
                        0,0,0,1,
                        };

            Matx44d gMt = calib_pose_;
            Eigen::Affine3f pose;
            mat2eigen(oMg,pose);
            viewer->displayPCLScene(pcl_,"result",pose);

            Matx44d tMb_ready = calib_pose*oMg*gMt*Matx44d{0,-1,0,0, 1,0,0,0, 0,0,1,-0.04, 0,0,0,1};
            Matx44d tMb_pick = calib_pose*oMg*gMt*Matx44d{0,-1,0,0, 1,0,0,0, 0,0,1,0.005, 0,0,0,1};
            displayPose(tMb_ready,tMb_pick);

            vector<double> joints1,joints2;
            vector<int32_t> pulse1,pulse2;
            if(!Convert::inverseKinematic(tMb_ready,joints1)){
                cout << "Cannot move to approach position!" << endl;
                fail();
                return;
            }
            Convert::joint2Pulse(joints1,pulse1);
            std::cout << "ready pulse: ";
            for(int i=0;i<pulse1.size();i++){
                std::cout << pulse1[i] << " ";
            }
            std::cout << std::endl;

            if(!Convert::inverseKinematic(tMb_pick,joints2)){
                cout << "Cannot move to pick position!" << endl;
                fail();
                return;
            }
            Convert::joint2Pulse(joints2,pulse2);
            std::cout << "pick pulse: ";
            for(int i=0;i<pulse2.size();i++){
                std::cout << pulse2[i] << " ";
            }
            std::cout << std::endl;
            memcpy(motoudp->position,pulse1.data(),24);
            memcpy(motoudp->position+6,pulse2.data(),24);
            camera->isBlock = false;
            motoudp->fail = false;
            motoudp->good = true;
            motoudp->triggerWritePositions = true;
            ui->radioButtonTrigger->setChecked(false);
            ui->radioButtonGood->setChecked(true);
            count++;
        }
        else{
            camera->isBlock = false;
            std::cout << "Not Ready" << std::endl;
            ui->radioButtonFail->setChecked(false);
            ui->radioButtonGood->setChecked(false);
            ui->radioButtonTrigger->setChecked(false);
        }

    }

}



void MainWindow::addCoordinate(Eigen::Affine3f aswer)
{
    viewer_->displayCoordiante(aswer);
}

void MainWindow::on_pushButtonGetPose_clicked()
{
    calib->startTrigger();
}

void MainWindow::on_pushButtonCaculate_clicked()
{
    calib->caculatePose();
}

void MainWindow::ReadSettings() {
  // Read Settings
    std::cout << "Read Settings" << std::endl;
    QSettings settings("setting", "ros_6d_pose_estimation");
    robotIP = settings.value("robotIP",QString("192.168.1.11")).toString();
    ui->lineEditRobotIP->setText(robotIP);
    gripperAddress = settings.value("gripperAddress",QString("/dev/ttyUSB0")).toString();
    ui->lineEditGripperAddress->setText(gripperAddress);
    firstPV = settings.value("firstPV",40).toInt();
    ui->spinBoxFirstPV->setValue(firstPV);
    firstBV = settings.value("firstBV",40).toInt();
    ui->spinBoxFirstBV->setValue(firstBV);
    calibFile = settings.value("calibFile").toString();
    ui->lineEditCalibrationFile->setText(calibFile);
    camFile = settings.value("camFile").toString();
    ui->lineEditCameraParam->setText(camFile);
    modelPath = settings.value("modelPath").toString();
    ui->lineEditModelPath->setText(modelPath);
    jobName = settings.value("jobName").toString();
    ui->lineEditJobName->setText(jobName);
    Convert::loadMatFile(calib_pose,calibFile.toStdString()+"/calib.yml");
    Convert::loadMatFile(calib_pose_,calibFile.toStdString()+"/calib_.yml");
    yoloPath = settings.value("yoloPath").toString();
    ui->lineEditYoloPath->setText(yoloPath);
//    std::cout << settings.value("minRange",200).toInt() << settings.value("maxRange",600).toInt() << endl;
//    ui->verticalSliderMax->setTracking(false);
//    ui->verticalSliderMin->setTracking(false);
////    ui->verticalSliderMax->setSliderPosition(settings.value("maxRange",600).toInt());
////    ui->verticalSliderMin->setSliderPosition(settings.value("minRange",200).toInt());
//    std::cout << ui->verticalSliderMax->value() << ui->verticalSliderMin->value() << std::endl;
//    ui->verticalSliderMax->setValue(settings.value("maxRange",600).toInt());
//    ui->verticalSliderMin->setValue(settings.value("minRange",200).toInt());
//    ui->verticalSliderMax->setTracking(true);
//    ui->verticalSliderMin->setTracking(true);
//    std::cout << ui->verticalSliderMax->value() << ui->verticalSliderMin->value() << std::endl;
}

void MainWindow::WriteSettings() {
    //Write Setting
    std::cout << "Write Settings" << std::endl;
    QSettings settings("setting", "ros_6d_pose_estimation");
    settings.setValue("robotIP",ui->lineEditRobotIP->text());
    settings.setValue("gripperAddress",ui->lineEditGripperAddress->text());
    settings.setValue("firstPV",QVariant(ui->spinBoxFirstPV->value()));
    settings.setValue("firstBV", QVariant(ui->spinBoxFirstBV->value()));
    settings.setValue("calibFile", ui->lineEditCalibrationFile->text());
    settings.setValue("camFile",ui->lineEditCameraParam->text());
    settings.setValue("modelPath",ui->lineEditModelPath->text());
    settings.setValue("jobName",ui->lineEditJobName->text());
    settings.setValue("yoloPath",ui->lineEditYoloPath->text());
    settings.setValue("minRange",ui->verticalSliderMin->value());
    settings.setValue("maxRange",ui->verticalSliderMax->value());
}

void MainWindow::readyCameraStatus()
{
    if(camera->camera_running){

        isCameraRunning = true;
    }
    if(camera->camera_running&&ppf->isComplete){
        ui->radioButtonReady->setChecked(true);
        isReady = true;
    }
}

void MainWindow::readyPPFStatus()
{
    std::cout << "PPF Ready" << std::endl;
    if(camera->camera_running&&ppf->isComplete){
        ui->radioButtonReady->setChecked(true);
        isReady = true;
        motoudp->ready = 1;
    }
}

void MainWindow::triggerByRobot()
{
    isTrigger = true;
    std::cout << "Trigger by robot" << std::endl;
}

void MainWindow::triggerGripperByRobot(int value)
{
    servo->WriteServo(value);
    std::cout << "Trigger gripper by robot" << std::endl;
}


void MainWindow::on_toolButtonCalib_clicked()
{
    QFileDialog dialog;
    dialog.setFileMode(QFileDialog::DirectoryOnly);
    dialog.setOption(QFileDialog::ShowDirsOnly, false);
    if(dialog.exec()){
        QString fileName = dialog.directory().path();
        if(!fileName.isNull()&&!fileName.isEmpty()){
             ui->lineEditCalibrationFile->setText(fileName);
        }
    }

}

void MainWindow::on_toolButtonCalibCameraParam_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Chose File"),
                                                    ui->lineEditCameraParam->text(),
                                                    tr("JSON (*.json)"));

    if(!fileName.isNull()&&!fileName.isEmpty()){
         ui->lineEditCameraParam->setText(fileName);
    }
}

void MainWindow::on_toolButtonModelPath_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Chose File"),
                                                    ui->lineEditModelPath->text(),
                                                    tr("PLY (*.ply)"));

    if(!fileName.isNull()&&!fileName.isEmpty()){
         ui->lineEditModelPath->setText(fileName);
         pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_(new pcl::PointCloud<pcl::PointXYZRGBA>);
         cv::Mat input = ppf_match_3d::loadPLYSimple(fileName.toStdString().c_str(),0);
         mat2Pcl(input,pcl_);
         viewer_->displayPCLModel(pcl_,ui->lineEditModelPath->text().toStdString());
    }
}

void MainWindow::on_pushButtonStartCalib_clicked()
{
    isRuntime=false;
    camera->stop();
    delete camera;
    calib = new HandEyeCalibration(this->ui->lineEditRobotIP->text(),this->ui->lineEditCameraParam->text().toStdString());
    connect(calib, &HandEyeCalibration::finishCalibrate, this, &MainWindow::addCoordinate);
    //Realrun
    calib->start();
    //Just Test
    //calib->test();
}

void MainWindow::on_pushButtonStartSave_clicked()
{
    isRuntime = true;
    calib->stop();
    Convert::saveMatFile(calib->calib_pose,ui->lineEditCalibrationFile->text().toStdString()+"/calib.yml");
    Convert::saveMatFile(calib->calib_pose_,ui->lineEditCalibrationFile->text().toStdString()+"/calib_.yml");
    delete calib;

    camera = new Camera(848, 480, 848, 480, 30,ui->lineEditCameraParam->text().toStdString());
    connect(camera, &Camera::framesReady, this, &MainWindow::receiveFrame);
    connect(camera, &Camera::pclReady, this, &MainWindow::receivePcl);
    connect(camera, &Camera::pointCloudReady, this, &MainWindow::receivePointCloud);
    camera->start();
}

void MainWindow::on_radioButtonTrigger_toggled(bool checked)
{
    isTrigger=checked;
}

void MainWindow::on_pushButtonSaveImage_clicked()
{
    camera->capture = true;
}

void MainWindow::on_pushButtonStart_clicked()
{
    motoudp->triggerStartJob = true;
}

void MainWindow::on_pushButtonServoOn_clicked()
{
    if(ui->pushButtonServoOn->text()=="Servo On"){
        motoudp->triggerTurnOnServo = true;
        ui->pushButtonServoOn->setText("Servo Off");
    }
    else{
        motoudp->triggerTurnOffServo = true;
        ui->pushButtonServoOn->setText("Servo On");
    }
}

void MainWindow::on_toolButtonYoloPath_clicked()
{
    QFileDialog dialog;
    dialog.setFileMode(QFileDialog::DirectoryOnly);
    dialog.setOption(QFileDialog::ShowDirsOnly, false);
    if(dialog.exec()){
        QString fileName = dialog.directory().path();
        if(!fileName.isNull()&&!fileName.isEmpty()){
             ui->lineEditYoloPath->setText(fileName);
        }
    }
}

void MainWindow::on_pushButtonSaveSettings_clicked()
{
    WriteSettings();
    exit(0);
}

void MainWindow::on_verticalSliderMax_valueChanged(int value)
{
    camera->maxRange = ui->verticalSliderMax->value()/1000.0;
}

void MainWindow::on_verticalSliderMin_valueChanged(int value)
{
    camera->minRange = ui->verticalSliderMin->value()/1000.0;
}

void MainWindow::on_pushButtonGripperOpen_clicked()
{
    servo->WriteServo(ui->spinBoxGripperWidth->value());
}
