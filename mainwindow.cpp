#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "camera.h"
#include <opencv2/surface_matching/ppf_helpers.hpp>
using namespace Convert;
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ReadSettings();
    //UDP Connection Initialize
    motoudp = new MotoUDP(QHostAddress(ui->lineEditRobotIP->text()),10040);
    connect(motoudp, &MotoUDP::triggerPPF, this, &MainWindow::triggerByRobot);
    motoudp->start();

//    motoudp->connectMotoman();


    if(camFile.isNull()||camFile.isEmpty()){
        cout << "Please set your camera parameters file (.json) " << endl;
        return;
    }
    camera = new Camera(424, 240, 848, 480, 30,ui->lineEditCameraParam->text().toStdString());
    connect(camera, &Camera::framesReady, this, &MainWindow::receiveFrame);
    connect(camera, &Camera::pclReady, this, &MainWindow::receivePcl);
    connect(camera, &Camera::pointCloudReady, this, &MainWindow::receivePointCloud);
    camera->setMaxRange(ui->verticalSlider->value());
    connect(ui->verticalSlider,&QSlider::valueChanged,camera,&Camera::maxRangeChanged);
    connect(camera,&Camera::connected,this,&MainWindow::readyCameraStatus);
    camera->start();

    viewer = new pclViewer(ui->qvtkWidget);
    viewer_ = new pclViewer(ui->qvtkWidgetModel);


    servo = new ServoControl();

    if(modelPath.isNull()||modelPath.isEmpty()){
        cout << "Please set your model file (.ply) " << endl;
        return;
    }
    ppf = new PPF(modelPath.toStdString());
    ppf->start();
    connect(ppf,&PPF::complete,this,&MainWindow::readyPPFStatus);
    if(yoloPath.isNull()||yoloPath.isEmpty()){
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
    if(results.size()==0) return;
    vector<cv::Mat> roi;
    vector<float> depth_medium;
    roi.resize(results.size());
    depth_medium.resize(results.size());
    for (size_t i=0;i<results.size();i++) {
        int sum=0;
        depth_medium.at(i)=0;
        roi.at(i).create(results.at(i).pixW*results.at(i).pixH, 3, CV_32FC1);
        for(int j=results.at(i).y;j<results.at(i).y+results.at(i).pixH;j++){
            for(int k=results.at(i).x;k<results.at(i).x+results.at(i).pixW;k++){
                if(depth_image.at<uint16_t>(j,k)>0){
                    float z = depth_image.at<uint16_t>(j,k)*1e-5;//depth unit = 1e-5
//                    cout << z << " ";
                    //instrict.fx fy ppx ppy
                    roi.at(i).at<float>(sum,0) =  (float)(k-instrict.ppx)*z/instrict.fx;
                    roi.at(i).at<float>(sum,1) = (float)(j-instrict.ppy)*z/instrict.fy;
                    roi.at(i).at<float>(sum,2) = z;
                    depth_medium.at(i) += roi.at(i).at<float>(sum,2);
                    sum++;
                }
            }
        }
      depth_medium.at(i) *= 1/sum;
      roi.at(i).resize(sum);
    }
    mat = roi.at(std::distance(depth_medium.begin(),std::min_element(depth_medium.begin(), depth_medium.end())));


}

void MainWindow::receiveFrame(QImage rgb)
{
    ui->labelImage->setPixmap(QPixmap::fromImage(rgb).scaled(ui->labelImage->size().width(),ui->labelImage->size().height(),Qt::KeepAspectRatio));
    QString sizeString = QString("(%1,%2)").arg(ui->labelImage->size().width()).arg(ui->labelImage->size().height());
    //ui->labelSize->setText(sizeString);
}

void MainWindow::receivePcl(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointcloud)
{
    //std::cout <<"Got PCL: " << pointcloud->points.size() << std::endl;
    viewer->displayPCL(pointcloud);
}

void MainWindow::receivePointCloud(cv::Mat depth_image,cv::Mat rgb_image)
{
    if(isTrigger){
        if(isReady){
            ppf_match_3d::Pose3DPtr result;

            vector<struct Yolo::yoloResult> results;
            yolo->computeYolo(rgb_image,results);
            cv::Mat input;
            choosePointCloud(results,depth_image,camera->inrist,input);
            writePLY(input,"testYolo.ply");
            cv::Mat pc;
            if(input.rows==0) {
                isTrigger = false;
                ui->radioButtonTrigger->setChecked(false);
                ui->radioButtonFail->setChecked(true);
                return;
            }
            if(!ppf->caculatePPF(input,result,pc)){
                isTrigger = false;
                ui->radioButtonTrigger->setChecked(false);
                ui->radioButtonFail->setChecked(true);
                return;
            }
//            if(result->residual>1||result->numVotes<1000){
//                isTrigger = false;
//                ui->radioButtonTrigger->setChecked(false);
//                ui->radioButtonFail->setChecked(true);
//                return;
//            }

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_(new pcl::PointCloud<pcl::PointXYZRGBA>);
            mat2Pcl(pc,pcl_);
//            Eigen::Affine3f pose;
//            mat2eigen(result->pose,pose);
//            viewer->displayPCLScene(pcl_,"result",pose);

            Matx44d test{0,-1,0,0,
                         0,0,-1,0.1,
                         1,0,0,0,
                         0,0,0,1};

            Eigen::Affine3f pose;
            mat2eigen(result->pose*test,pose);
            viewer->displayPCLScene(pcl_,"result",pose);

            Matx44d robot_pos = calib_pose*result->pose*test;
            cout << "pose: " << robot_pos << std::endl;
            Mat as(robot_pos);
            Mat rot;
            as.colRange(0,3).rowRange(0,3).convertTo(rot,CV_32F);
            cout << "translation: " << as.rowRange(0,3).col(3)*1000 << std::endl;;
            cout << " rotation: " << Convert::rotationMatrixToEulerAngles(rot)*180/M_PI << std::endl;
            Vec3f rotation = Convert::rotationMatrixToEulerAngles(rot)*180/M_PI;

            motoudp->position[0] = round(as.at<double>(0,3)*1000000);
            motoudp->position[1] = round(as.at<double>(1,3)*1000000);
            motoudp->position[2] = round(as.at<double>(2,3)*1000000);
            motoudp->position[3] = round(rotation[0]*10000);
            motoudp->position[4] = round(rotation[1]*10000);
            motoudp->position[5] = round(rotation[2]*10000);
            motoudp->position[6] = round(as.at<double>(0,3)*1000000);
            motoudp->position[7] = round(as.at<double>(1,3)*1000000);
            motoudp->position[8] = round(as.at<double>(2,3)*1000000);
            motoudp->position[9] = round(rotation[0]*10000);
            motoudp->position[10] = round(rotation[1]*10000);
            motoudp->position[11] = round(rotation[2]*10000);
            motoudp->triggerWritePositions = true;



        }
        else{
            std::cout << "Not Ready" << std::endl;
        }
        isTrigger = false;
        ui->radioButtonTrigger->setChecked(false);
        ui->radioButtonFail->setChecked(false);
    }

}

void MainWindow::addCoordinate(Eigen::Affine3f aswer)
{
    viewer_->displayCoordiante(aswer);
}


void MainWindow::on_pushButton_3_clicked()
{
    std::cout << "Result " << servo->WriteServo(ui->spinBox->value()) << std::endl;
}

void MainWindow::on_pushButtonSendPosition_clicked()
{
    calib->startSendPosition();
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
    Convert::loadMatFile(calib_pose,calibFile.toStdString());
    yoloPath = settings.value("yoloPath").toString();
    ui->lineEditYoloPath->setText(yoloPath);
    //std::cout << calib_pose << std::endl;
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
}

void MainWindow::readyCameraStatus()
{
    if(camera->camera_running){
        ui->radioButtonRunning->setChecked(true);
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
    }
}

void MainWindow::triggerByRobot()
{
    isTrigger = true;
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
    Convert::saveMatFile(calib->calib_pose,ui->lineEditCalibrationFile->text().toStdString());
    delete calib;

    camera = new Camera(848, 480, 848, 480, 30,ui->lineEditCameraParam->text().toStdString());
    connect(camera, &Camera::framesReady, this, &MainWindow::receiveFrame);
    connect(camera, &Camera::pclReady, this, &MainWindow::receivePcl);
    connect(camera, &Camera::pointCloudReady, this, &MainWindow::receivePointCloud);
    camera->start();
}

void MainWindow::on_verticalSlider_valueChanged(int value)
{

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
