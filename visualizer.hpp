/* Copyright 18.10.2017 Ikol */
/*!
\file
\brief Header file whit Visualization class description
*/
#ifndef HEADERS_VISUALIZER_HPP_
#define HEADERS_VISUALIZER_HPP_

#include <iostream>
#include <utility>
#include <vector>
#include <thread>
#include <list>
#include <string>

#include <mrpt/opengl/COpenGLViewport.h> 
#include <chrono>
#include <Windows.h>

#include <mrpt/gui.h>
#include <mrpt/opengl.h>
#include <mrpt/hwdrivers/CCameraSensor.h>
#include <mrpt/utils/PLY_import_export.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/opengl/CBox.h>
#include <mrpt/utils/TColor.h>

#include <opencv2/opencv.hpp>
#include <opencv/cv.h>

#include "data_types.hpp"

using namespace std;

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::obs;

namespace vis {
/*!
	\brief Visualizer
	\author I_cool
	\version 1.0
	\date 31.01.2018
	\warning This class implements the displeing of objects is 3D-space
	This class is wrapper on mrpt library. It provides access to the 3D-space.
*/
class Visualizer {
 public:
        /**
         * @brief Constructor
        * 
        * @param drawPath - display passed path
        */
        explicit Visualizer(bool drawPath = false);
        /**
         * @brief Distructor
         */
        ~Visualizer(void);
        /**
         * @brief Check visualization
         * 
         * @return true visualization is working
         * @return false visualization does not work
         */
        bool isRun(void);
        /**
         * @brief change vehicle position in 3D-space
         * 
         * @param pose - new vehicle position
         */
        void SetCarPos(road_data::ObjPos* pose);
        /**
         * @brief Launch visualization module
         */
        void StartVisualizer(void);
        /**
         * @brief Add lines for displaying 
         * 
         * @param dataType
         * @param listPoints 
         * @param limitType 
         * @param markupType 
         */
        void AddData(vis::DataType dataType,
                                std::list<mrpt::math::TPoint3D> listPoints,
                                road_data::LimitType limitType =
                                road_data::Unknown,
                                road_data::MarkupType markupType =
                                road_data::UnknownMarkup);
        /**
         * @brief Add objects (rectangle) for displaying
         * 
         * @param dataType 
         * @param listObjects 
         * @param objectType 
         */
        void AddData(vis::DataType dataType,
                std::list<mrpt::opengl::CBoxPtr> listObjects,
                road_data::ObjectType objectType =
                road_data::UnknownObject);
        /**
         * @brief Clear visualization data depending on the type
         * 
         * @param dataType 
         */
        void ClearData(vis::DataType dataType);
        /**
         * @brief Display another images
         * 
         * @param img 
         * @param numberWindow 
         */
        void ShowImage(cv::Mat img, int numberWindow);
        /**
         * @brief Set GPS position and radius 
         * 
         * @param pose 
         * @param radius 
         */
		void ShowImages(CImage img);

        void SetGpsPos(poses::CPose2D pose, float radius);
        /**
         * @brief Clear passed path
         */
        void clearTraveledPath(void);
        /**
         * @brief Add error-points for displaying
         * 
         * @param pose 
         * @param radius 
         * @param color 
         * @param type 
         */
        void DrawErrorPoints(poses::CPose2D pose,
                                float radius,
                                mrpt::utils::TColorf color,
                                int type);
        /**
         * @brief Remove error-points by type
         * 
         * @param type if set -1 remove all points
         */
        void RemoveErrorPoints(int type);

 private:
        CDisplayWindow3D* win3D;  ///< main window
        std::thread* _displayThread;  ///< thread for independent display

        bool _displayPath;  ///< display or not display passed path
        const std::string _pathCarModel =
        // "/usr/objects/Robotized Platform 2.dae";
        "/usr/objects/leganza1.dae";  ///< path to the 3D vehicle model

        mrpt::opengl::CAssimpModelPtr vehicle;  ///< 3D vehicle model
        mrpt::opengl::CGridPlaneXYPtr _gl_ground;  ///< a grid like a road

        #pragma region Storage Draw Data

        std::list<mrpt::opengl::CSetOfLinesPtr> _listReposDigitizeD;  ///< repository for digitized line
        std::list<mrpt::opengl::CSetOfLinesPtr> _listReposCurrentD;  ///< repository for current recognition line

        mrpt::opengl::CSetOfObjectsPtr _listReposDigitizeDObj;  ///< repository for current digitized objects
        mrpt::opengl::CSetOfObjectsPtr _listReposCurrentDObj;  ///< repository for current recognition objects

        mrpt::opengl::CSetOfObjectsPtr _listReposErrorPoints;  ///< repository for points of error
        std::vector< std::pair<int, int> > _listTypeErrorPoints;  ///< repository for type end amount of objects

        #pragma endregion

        #pragma region GPS

        mrpt::opengl::CDiskPtr _gpsError;  ///< circle is GPS error
        mrpt::opengl::CDiskPtr _centerPointGpsError;  ///< central point of GPS error

        #pragma endregion GPS

        mrpt::opengl::CSetOfObjectsPtr _vehicleSet;  ///< set of vehicle's devices

        std::vector<CDisplayWindow*> _listDWindows;  ///< repository for windows (for display another images)

        // ========= Opengl View: Map & robot  =======
        COpenGLScenePtr theScene;  ///< scene in 3D space
		COpenGLViewportPtr vi;
        mrpt::poses::CPose3D current_position;  ///< curent position vehicle (for display passed path)
        mrpt::opengl::CSetOfLinesPtr gl_robot_path;  ///< repository for passed path
        /**
         * @brief Launch visualization main thread 
         */
        void Run(void);
        /**
         * @brief Create string name for storage in CSetOfObjectsPtr
         * 
         * @param type 
         * @param number 
         * @return std::string 
         */
        std::string createName(int type, int number);
        /**
         * @brief Check if the type exist in the _listTypeErrorPoints
         * 
         * @param type 
         * @return int -1 if the type not exist else return index
         */
        int isType(int type);
        /**
         * @brief Remove error-point from _listTypeErrorPoints by index
         * 
         * @param index 
         */
        void removeErrorPointByIndex(int index);
        /**
         * @brief Select line properties
         * 
         * @param line 
         * @param limitType 
         * @param markupType 
         * @param DigitizeOrCurrent 
         */
        void SelectLineProperties(mrpt::opengl::CSetOfLinesPtr line,
                road_data::LimitType limitType,
                road_data::MarkupType markupType,
                bool DigitizeOrCurrent);
        /**
         * @brief Create line
         * 
         * @param mrpt::opengl::CSetOfLinesPtr 
         * @param listPoints 
         * @param limitType 
         * @param markupType 
         * @param DigitizeOrCurrent 
         */
        void CreateLine(
                mrpt::opengl::CSetOfLinesPtr,
                std::list<mrpt::math::TPoint3D> listPoints,
                road_data::LimitType limitType,
                road_data::MarkupType markupType,
                bool DigitizeOrCurrent);
};
}  // namespace vis

#endif  // HEADERS_VISUALIZER_HPP_
