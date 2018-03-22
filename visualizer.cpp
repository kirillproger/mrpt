/* Copyright 18.10.2017 Ikol */
#include "visualizer.hpp"

using namespace vis;

vis::Visualizer::Visualizer(bool displayPath) {
    #pragma region Storage Draw Data

        this->_listReposDigitizeD = std::list<mrpt::opengl::CSetOfLinesPtr>();
        this->_listReposCurrentD = std::list<mrpt::opengl::CSetOfLinesPtr>();
        this->_listReposDigitizeDObj = mrpt::opengl::CSetOfObjects::Create();
        this->_listReposCurrentDObj = mrpt::opengl::CSetOfObjects::Create();

    #pragma endregion

    this->_listReposErrorPoints = mrpt::opengl::CSetOfObjects::Create();

    this->_displayPath = displayPath;

    this->_listDWindows = std::vector<CDisplayWindow*>();

    win3D = new CDisplayWindow3D("Visualizer", 1024, 800);
    win3D->setPos(100, 100);
	theScene = win3D->get3DSceneAndLock();
    current_position = mrpt::poses::CPose3D();
	

    this->_vehicleSet = mrpt::opengl::CSetOfObjects::Create();

    // Vehicle
    {
        this->vehicle =
        mrpt::opengl::CAssimpModel::Create();

        this->vehicle->loadScene(this->_pathCarModel);

        this->vehicle->setName("Vehicle");
        this->vehicle->setScale(20.0);  // 0.9

        this->_vehicleSet->insert(this->vehicle);
    }
    this->theScene->insert(this->_vehicleSet);

    // Top cam
    {
        mrpt::opengl::CFrustumPtr obj = mrpt::opengl::CFrustum::Create();
        // obj->setPlaneColor(0, 1, 0);
        obj->setHorzFOV(67);
        obj->setVertFOV(80);
        obj->setScale(5, 3, 3);

        mrpt::poses::CPose3D pointTopCam = mrpt::poses::CPose3D(
            0.8,
            0.005,
            0.66/*(z)*/,
            0 /*yaw */,
            DEG2RAD(11.0) /*pitch roll*/ );

            obj->setPose(pointTopCam);

        this->_vehicleSet->insert(obj);
    }

    // Gps error
    {
        this->_gpsError = mrpt::opengl::CDisk::Create();
        this->_gpsError->setPose(mrpt::poses::CPose3D(0, 0, 0));
        this->_gpsError->setColor(1, 0, 0);
        this->_gpsError->setDiskRadius(1 + 0.05, 1 - 0.05);
        this->_gpsError->setLoopsCount(10);
        this->theScene->insert(this->_gpsError);

        this->_centerPointGpsError = mrpt::opengl::CDisk::Create();
        this->_centerPointGpsError->setDiskRadius(0.04, 0);
        this->_centerPointGpsError->setColor(1, 0, 0);
        this->_centerPointGpsError->setLoopsCount(1);
        this->_centerPointGpsError->setPose(mrpt::poses::CPose3D(0, 0, 0));
        this->theScene->insert(this->_centerPointGpsError);
    }

    if (this->_displayPath) {
        gl_robot_path = mrpt::opengl::CSetOfLines::Create();
        gl_robot_path->setLineWidth(1);
        gl_robot_path->setColor_u8(TColor(255, 255, 0, 255));
        theScene->insert(gl_robot_path);
    }

    // XY Grid
    {
        _gl_ground =
        mrpt::opengl::CGridPlaneXY::Create(-100, 100, -100, 100, 0, 1);

        _gl_ground->setColor(0, 0, 0);
        _gl_ground->setGridFrequency(1);
        theScene->insert(_gl_ground);
    }

    this->_vehicleSet->setPose(mrpt::poses::CPose3D(0, 0, 0.15, mrpt::utils::DEG2RAD(90)));

    win3D->unlockAccess3DScene();

    this->_displayThread = new std::thread(&vis::Visualizer::Run, this);
}

vis::Visualizer::~Visualizer(void) {
    if (this->_displayPath) {
        gl_robot_path->clear();
    }

    this->_listReposDigitizeDObj.clear();

    if (win3D != NULL) {
        cout << "Distructor win3D" << endl;
        // delete win3D;
        // win3D = NULL;
    }
}

bool vis::Visualizer::isRun(void) {
    bool isOpen;
    this->win3D->get3DSceneAndLock();
    isOpen = this->win3D->isOpen();
    this->win3D->unlockAccess3DScene();
    return isOpen;
}

void vis::Visualizer::Run(void) {
    cout << "Close the window to end.\n";
    while (win3D->isOpen()) {
        win3D->addTextMessage(
            5,
            5,
            format("%.02fFPS", win3D->getRenderingFPS()));

        mrpt::system::sleep(1);

        if (this->_displayPath) {

            win3D->get3DSceneAndLock();
            // Update path graph:
            const TPoint3D  cur_pt(
                current_position.x(),
                current_position.y(),
                0.01);

            static int decim_path = 0;
            if (gl_robot_path->empty() || ++decim_path > 10) {
                gl_robot_path->appendLine(cur_pt, cur_pt);
            } else {
                gl_robot_path->appendLineStrip(cur_pt);
                decim_path = 0;
            }
            win3D->unlockAccess3DScene();
        }
		
        win3D->repaint();
	
    }
}

void vis::Visualizer::SetCarPos(road_data::ObjPos* pose) {
    win3D->get3DSceneAndLock();

    current_position = mrpt::poses::CPose3D(
        pose->x(),
        pose->y(),
        0.15,
        1.5708 - pose->ang(),  // 90 grad - angl
        0);

    this->_vehicleSet->setPose(current_position);

    // mrpt::poses::CPose3D p = mrpt::poses::CPose3D(x, y, 0.15, 0, 0, 0);

    // this->_gl_ground->setPose(p);

    win3D->setCameraPointingToPoint(
        this->current_position.x(),
        this->current_position.y(),
        this->current_position.z());

    // std::cout << current_position << std::endl;

    win3D->unlockAccess3DScene();
	
}

void vis::Visualizer::StartVisualizer(void) {
    this->_displayThread->joinable();
}

void vis::Visualizer::SelectLineProperties(mrpt::opengl::CSetOfLinesPtr line,
    road_data::LimitType limitType,
    road_data::MarkupType markupType,
    bool DigitizeOrCurrent) {
        int brightness = 0;
        int lineWidth = 0;

        if (DigitizeOrCurrent) {
            // Digitize data
            brightness = 255;
            lineWidth = 3;
        } else {
            // Current data
            brightness = 80;
            lineWidth = 8;
        }

        switch (limitType) {
            case  road_data::Unknown: {
                line->setLineWidth(lineWidth);
                line->setColor_u8(
                    TColor(255, 0, 0, brightness));
            }
                break;

            case road_data::VirtualLimit: {
                line->setLineWidth(lineWidth);
                line->setColor_u8(
                    TColor(255, 102, 0, brightness));
            }
                break;

            case road_data::Bump: {
                line->setLineWidth(lineWidth);
                line->setColor_u8(
                    TColor(0, 255, 0, brightness));
            }
                break;

            case road_data::Border: {
                line->setLineWidth(lineWidth);
                line->setColor_u8(
                    TColor(0, 150, 0, brightness));
            }
                break;

            case road_data::Markup: {
                line->setLineWidth(lineWidth);
                line->setColor_u8(
                    TColor(0, 0, 255, brightness));
            }
                break;

            case road_data::Runaway: {
                line->setLineWidth(lineWidth);
                line->setColor_u8(
                    TColor(0, 0, 0, brightness));
            }
                break;

            case road_data::Building: {
                line->setLineWidth(lineWidth);
                line->setColor_u8(
                    TColor(165, 45, 45, brightness));
            }
                break;

            default: {
                line->setLineWidth(lineWidth);
                line->setColor_u8(
                TColor(255, 0, 0, brightness));
            }
                break;
            }
}

void vis::Visualizer::CreateLine(
    mrpt::opengl::CSetOfLinesPtr storLine,
    std::list<mrpt::math::TPoint3D> listPoints,
    road_data::LimitType limitType,
    road_data::MarkupType markupType,
    bool DigitizeOrCurrent ) {
    // select propertis fo this line
    this->SelectLineProperties(storLine,
        limitType,
        markupType,
        DigitizeOrCurrent);

    for (std::list<mrpt::math::TPoint3D>::iterator it =
        listPoints.begin();
        it != listPoints.end();
        ++it) {
            if (listPoints.size() > 1) {

                win3D->get3DSceneAndLock();

                if (storLine->empty()) {
                    storLine->appendLine(*it, *(std::next(it)));
                } else {
                    storLine->appendLineStrip(*it);
                }

                win3D->unlockAccess3DScene();
            }
    }
}

void vis::Visualizer::AddData(
    vis::DataType dataType,
    std::list<mrpt::math::TPoint3D> listPoints,
    road_data::LimitType limitType,
    road_data::MarkupType markupType) {
        mrpt::opengl::CSetOfLinesPtr storLine =
        mrpt::opengl::CSetOfLines::Create();

        switch (dataType) {
            case vis::Digitized: {
                this->CreateLine(
                            storLine,
                            listPoints,
                            limitType,
                            markupType,
                            true);

                win3D->get3DSceneAndLock();

                this->_listReposDigitizeD.push_back(storLine);

                win3D->unlockAccess3DScene();
            } break;
            case vis::Current: {
                this->CreateLine(
                    storLine,
                    listPoints,
                    limitType,
                    markupType,
                    false);

                win3D->get3DSceneAndLock();

                this->_listReposCurrentD.push_back(storLine);

                win3D->unlockAccess3DScene();
            } break;
            default: std::cout << "Error data type" << std::endl;
        }

        theScene->insert(storLine);

        listPoints.clear();
}

void vis::Visualizer::AddData(
    vis::DataType dataType,
    std::list<mrpt::opengl::CBoxPtr> listObjects,
    road_data::ObjectType objectType) {
        win3D->get3DSceneAndLock();

        switch (dataType) {
            case vis::Digitized: {
                // --> Remove old objects
                this->_listReposDigitizeDObj->clear();

                for (std::list<mrpt::opengl::CBoxPtr>::iterator it =
                    listObjects.begin();
                    it != listObjects.end();
                    ++it) {
                        (*it)->setWireframe(true);
                        (*it)->setColor(1, 0, 0);
                        (*it)->setLineWidth(5.0);

                        this->_listReposDigitizeDObj->insert(*it);
                    }

                theScene->insert(this->_listReposDigitizeDObj);
            } break;
            case vis::Current: {
                // --> Remove old objects
                this->_listReposCurrentDObj->clear();

                for (std::list<mrpt::opengl::CBoxPtr>::iterator it =
                    listObjects.begin();
                    it != listObjects.end();
                    ++it) {
                        (*it)->setWireframe(true);
                        (*it)->setColor(0, 1, 0);
                        (*it)->setLineWidth(5.0);

                        this->_listReposCurrentDObj->insert(*it);
                }

                theScene->insert(this->_listReposCurrentDObj);
            } break;
            default: std::cout << "Error data type" << std::endl;
        }

    win3D->unlockAccess3DScene();

    listObjects.clear();
}

void vis::Visualizer::ClearData(vis::DataType dataType) {
     win3D->get3DSceneAndLock();

    switch (dataType) {
        case vis::Digitized: {
            for (std::list<mrpt::opengl::CSetOfLinesPtr>::iterator it =
                this->_listReposDigitizeD.begin();
                it != this->_listReposDigitizeD.end();
                ++it) {
                    it->clear();
                }

                this->_listReposDigitizeD.clear();
        } break;
        case vis::Current: {
            for (std::list<mrpt::opengl::CSetOfLinesPtr>::iterator it =
                this->_listReposCurrentD.begin();
                it != this->_listReposCurrentD.end();
                ++it) {
                    it->clear();
                }

                this->_listReposCurrentD.clear();
        } break;
        default: std::cout << "Error data type" << std::endl;
    }

    win3D->unlockAccess3DScene();
}
void vis::Visualizer::ShowImage(cv::Mat img, int numberWindow) {
	if (this->_listDWindows.size() < numberWindow + 1) {
		this->_listDWindows.push_back(
			new mrpt::gui::CDisplayWindow(std::to_string(numberWindow)));

	}

	IplImage iplImage = img;

	try {
		this->_listDWindows[numberWindow]->showImage(&iplImage);
	}
	catch (...) {
		std::cout << "Error ShowImage!!" << std::endl;
	}
}

void vis::Visualizer::ShowImages(CImage img) {
	COpenGLViewportPtr vi;
	{
		vi = theScene->createViewport("ShowImage");//create viewport
		vi->setViewportPosition(0.0, 0.0, 0.25, 1);//(x,y,width,height);
		vi->setImageView(img);//загрузка изображения
		vi->setTransparent(true);//прозрачность

	}
}
void vis::Visualizer::SetGpsPos(poses::CPose2D pose, float radius) {
    win3D->get3DSceneAndLock();

    this->_gpsError->setPose(pose);
    this->_gpsError->setDiskRadius(radius + 0.02, radius - 0.02);

    this->_centerPointGpsError->setPose(pose);

    win3D->unlockAccess3DScene();
}

void vis::Visualizer::clearTraveledPath(void) {
    win3D->get3DSceneAndLock();

    this->gl_robot_path->clear();

    win3D->unlockAccess3DScene();
}

void vis::Visualizer::removeErrorPointByIndex(int index) {
    int type = this->_listTypeErrorPoints[index].first;
    int amount = this->_listTypeErrorPoints[index].second;

    win3D->get3DSceneAndLock();

    for (auto i = 1; i <= amount; i++) {
        std::string name = this->createName(type, i);

        mrpt::opengl::CRenderizablePtr obj =
            this->_listReposErrorPoints->getByName(name);

        this->_listReposErrorPoints->removeObject(obj);
    }

    win3D->unlockAccess3DScene();

    this->_listTypeErrorPoints.erase(
            this->_listTypeErrorPoints.begin() + index);
}

int vis::Visualizer::isType(int type) {
    for (std::vector< std::pair<int, int> >::iterator it =
            this->_listTypeErrorPoints.begin();
            it != this->_listTypeErrorPoints.end();
            ++it) {
        if (it->first == type) {
            int index = std::distance(this->_listTypeErrorPoints.begin(), it);
            return index;
        }
    }

    return -1;
}

std::string vis::Visualizer::createName(int type, int number) {
    std::string name = std::string();

    std::stringstream ss;
    ss << type;  // error type
    name = ss.str() + "_";
    ss = std::stringstream();
    ss << number;  // amount errors
    name += ss.str();

    return name;
}

void vis::Visualizer::RemoveErrorPoints(int type) {
    win3D->get3DSceneAndLock();
    bool isEmpty = this->_listReposErrorPoints->empty();
    win3D->unlockAccess3DScene();

    // remove all
    if (type == -1 && !isEmpty) {
        win3D->get3DSceneAndLock();
        this->_listReposErrorPoints->clear();
        win3D->unlockAccess3DScene();

        this->_listTypeErrorPoints.clear();

    } else {
        int index = this->isType(type);

        if (index != -1) {
            this->removeErrorPointByIndex(index);
        }
    }
}

void vis::Visualizer::DrawErrorPoints(
        poses::CPose2D pose,
        float radius,
        mrpt::utils::TColorf color,
        int type) {
    int index = this->isType(type);

    mrpt::opengl::CDiskPtr disk = mrpt::opengl::CDisk::Create();
    disk->setPose(pose);
    disk->setColor(color);
    disk->setDiskRadius(radius + 0.05, radius - 0.05);
    disk->setLoopsCount(10);

    std::string name = std::string();

    // add new type and object
    if (index == -1) {
        name = this->createName(type, 1);

        this->_listTypeErrorPoints.push_back(
            std::make_pair(type, 1));

    } else {
        this->_listTypeErrorPoints[index].second++;
        name =
                this->createName(type,
                            this->_listTypeErrorPoints[index].second);
    }

    disk->setName(name);

    win3D->get3DSceneAndLock();
    this->_listReposErrorPoints->insert(disk);
    this->theScene->insert(this->_listReposErrorPoints);
    win3D->unlockAccess3DScene();
}
