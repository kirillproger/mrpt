/* Copyright 2017 Ikol */
#ifndef DATA_TYPES_DATA_TYPES_HPP_
#define DATA_TYPES_DATA_TYPES_HPP_

#include <iostream>
#include <list>
/*
#include "opencv2/opencv.hpp"

#define BLUE_COLOR cv::Scalar(255, 0, 0)
#define BLACK_COLOR cv::Scalar(0, 0, 0)
#define RED_COLOR cv::Scalar(0, 0, 255)
#define GREEN_COLOR cv::Scalar(0, 255, 0)
#define WHITE_COLOR cv::Scalar(255, 255, 255)
*/
namespace road_data {

    // Позиция объекта в сегменте. От начала сегмента в метрах.
class ObjPos {
 public:
	// расстояние от начала сегмента по оси х
	float x() {
        return _x;
    }
	// расстояние от начала сегмента по оси y
	float y() {
        return _y;
    }
    	// наш азимут, в радианах
	float ang() {
        return _ang;
    }
	ObjPos() {
	_x = 0;
	_y = 0;
	_ang = 0;
}
	ObjPos(float x, float y, float ang) {
	_x = x;
	_y = y;
	_ang = ang;
}

    void operator =(ObjPos newPos) {
    _x = newPos.x();
	_y = newPos.y();
	_ang = newPos.ang();
    }

 private:
	float _x;
	float _y;
	float _ang;


};

class Line {
 public:
	float startX;
	float startY;
    float endX;
	float endY;
};

class Obstacle {
 public:
    // Препятствие от цента бампера автомобиля, где
    // х и y центр объекта, вектор направленности (куда смотрит) объект (радианы)
    ObjPos pos;
    // height высота объекта
	float width;
	float height;
    float length;
// скорость движения
	float speedX;
// вектор движения (радианы)
	float speedY;

    Obstacle() {

    }
};

class ObstacleCamera {
 public:
 // metr
	float leftPointX;
	float leftPointY;
	float width;
	float height;
};

    enum Obstacles {
        UnknownObst = 0,
        PersonObst = 1,
        CarObst = 2,
        TruckObst = 3,
    };

    enum LimitType {
        // Red Line
        Unknown = 0,
        // ограничение, которого нет в реальности
        VirtualLimit = 1,
        // Отбойник
        Bump = 2,
        // бордюр
        Border = 3,
        // Разметка
        Markup = 4,
        // обрыв
        Runaway = 5,
        // здание
        Building = 6,
        // Нет ограничения(но ограничение сегмента)
        NoLimit = 255,
    };

    enum ObjectType {
        UnknownObject = -1,
        SingType = 0,
        ObstacleType = 1
    };

     enum MarkupType {
        // ГОСТ * 10
        UnknownMarkup = -1,
        TrafficLaneMarkings = 10,
        // Сплошная линия, разделяет противополжные направления
        EdgeRoadway = 21,
        // Край проезжей части
        EdgeRoadAtTwoWays = 22,
        //Прерывистая разделяет противоположные направления
        DoubleWhiteLines = 30,
        //Двойная сплошная
        ForbiddenStop = 40,
        // Остановка запрещена
        DashLine = 50,
        // Прерывистая линия
        ApproachingSolidLine = 60,
        //Приближение к сплошной линии
        LaneTrafficAtCrossroads = 70,
        // Полоса движения на перекрестке
        StopOrRunBand = 80,
        //Полоса разгона/торможения
        ReverseBands = 90,
        //Полоса реверсивного движения
        NoParking = 100,
        //Стоянка запрещена
        EdgeRoadAtTwoWaysWithDashLine = 110,
        //двойная сплошная с прерывистой
        EdgeRoadAtTwoWaysWithDashLineOur = 111,
        //двойная сплошная с прерывистой с нашей стороны
        EdgeRoadAtTwoWaysWithDashLineNotOur = 112,
        //двойная сплошная с прерывистой с противоположной стороны
        StopLine = 120,
        //Стоп линий
        StopTriangle = 130,
        //Стоп линий из треугольников
        CrossWalk = 141,
        // стандартый пешеходный переход
        CrossWalkWithVector = 142,
        // пешеходный переход с вектором движения пешеходов по нему
        CrossingRoadsAndBicyclePaths = 150,
        //пересечение велосипедной дорожки и проезжей части
        IslandSeparationStreams = 161,
        //Остров раделения потоков движения автомобиля
        PlaceStopBus = 170,
        //Место остановки автобусов
        PermittedTurnsAtCrossroads = 180,
        //стрелки указывающие разрешенное движение на перекрестке
        NarrowingRoadway = 190,
        //предупреждение о сужении
        OncomingStopTriangle = 200,
        //приближение к Стоп линий из треугольников(130)
        OncomingStopLine = 210,
        //приближение к Стоп линий(120)
        RoadNumber = 220,
        //номер дороги
        SpecialBusWays = 230,
        //Дорога для маршрутных транспортных средст
        DublicateWarningRoasSings = 241,
        //дублирование Предупреждающих Дорожных знаков
        DublicateProhibitingRoasSings = 242,
        //дублирование Запрещающих Дорожных знаков
        DublicateInvalidRoasSings = 243,
        //дублирование Дорожного знаков "Инвалиды"
        ArtificialIrregularities = 250
        //Обозначение искуственных неровностей
        };
    #define MARKUP_MIN_WIDTH 100
    #define MARKUP_MIN_HEIGHT 1000
    #define TrafficLaneMarkings_WIDTH_MIN 100
    #define TrafficLaneMarkings_WIDTH_MAX 150
    #define EdgeRoadway_WIDTH_MIN 100
    #define EdgeRoadway_WIDTH_MAX 200
    #define EdgeRoadAtTwoWays_WIDTH 100
    #define DashLine_WIDTH_MIN 100
    #define DashLine_WIDTH_MAX 150
    #define DashLine_HEIGHT_CITY_MIN 1000
    #define DashLine_HEIGHT_CITY_MAX 3000
    #define DashLine_HEIGHT_OUTSIDECITY_MIN 3000
    #define DashLine_HEIGHT_OUTSIDECITY_MAX 4000
    #define DashLine_STEP_CITY_MIN 3000
    #define DashLine_STEP_CITY_MAX 9000
    #define DashLine_STEP_OUTSIDECITY_MIN 9000
    #define DashLine_STEP_OUTSIDECITY_MAX 12000
    #define ApproachingSolidLine_WIDTH_MIN 100
    #define ApproachingSolidLine_WIDTH_MAX 150
    #define ApproachingSolidLine_HEIGHT_CITY_MIN 3000
    #define ApproachingSolidLine_HEIGHT_CITY_MAX 6000
    #define ApproachingSolidLine_HEIGHT_OUTSIDECITY_MIN 6000
    #define ApproachingSolidLine_HEIGHT_OUTSIDECITY_MAX 9000
    #define ApproachingSolidLine_STEP_CITY_MIN 1000
    #define ApproachingSolidLine_STEP_CITY_MAX 2000
    #define ApproachingSolidLine_STEP_OUTSIDECITY_MIN 2000
    #define ApproachingSolidLine_STEP_OUTSIDECITY_MAX 3000
}  // namespace road_data



namespace vis {
    enum DataType {
        Current,
        Digitized
    };
}  // namespace vis

#endif  // DATA_TYPES_DATA_TYPES_HPP_
