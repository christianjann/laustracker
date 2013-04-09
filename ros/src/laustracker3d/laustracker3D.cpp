/*
 * This file is part of Laustracker.
 *
 * Copyright (C) 2013 Christian Jann <christian.jann@ymail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstdlib>
#include <boost/bind/bind.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

//OSG includes
#include <osg/Node>
#include <osg/Group>
#include <osgDB/ReadFile>
#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>
#include <osg/ArgumentParser>
#include <osgUtil/Optimizer>
#include <osg/Camera>
#include <osg/Vec4>
#include <osg/LineWidth>
#include <osg/TextureRectangle>
#include <osg/TexMat>

#include <laustracker/laustracker.h>

#define WITH_CAMVIEW

using namespace std;
namespace enc = sensor_msgs::image_encodings;

RobotTracker lt_robots;
LabyrinthMap lt_map;

string marker_map_begin = "START_MAP:";
string marker_map_end = ":END_MAP";
string marker_robots_begin = "START_ROBOTS:";
string marker_robots_end = ":END_ROBOTS";

typedef struct model_t
{
    osg::Group* root;
    osg::Switch* base;
    osg::Switch* walls;
    osg::Switch* robots;
    osg::Group* text;
    osg::Geode* camview;

} LabyrinthModel;

string map_str = "START_MAP:[00001010][00001100][00001100][00001100][00001001]"
                 "[00001110][00001100][00001101][00000011][00001010][00001100]"
                 "[00001101][00000110][00001100][00001100][00001001][00000011]"
                 "[00000011][00001010][00001100][00001100][00001001][00001111]"
                 "[00000011][00000110][00000001][00000011][00001110][00001001]"
                 "[00000110][00001100][00000101][00001010][00000101][00000110]"
                 "[00001001][00000110][00001100][00001100][00001101][00000011]"
                 "[00001010][00001101][00000110][00001000][00001100][00001100]"
                 "[00001001][00000011][00000110][00001100][00001001][00000011]"
                 "[00001010][00001001][00000011][00000110][00001100][00001100]"
                 "[00000101][00000110][00000101][00000111][00000111]:END_MAP";
string robots_str = "START_ROBOTS:\n"
                    "Andi 0 5 2 275.906 0.0540541 -0.148649 1 1.38267 27.5045 38.0296\n"
                    "Pete 1 4 0 0 -0.0405405 -0.0540541 1 2.09443 19.6469 41.1491\n"
                    ":END_ROBOTS";

float wall_width = 0.5;
float field_width = 10;
float base_thickness = 1;
float wall_hight = 3;
int field_count = 8;
bool no_relativ_pos = false;
long int frame_counter = 0;
bool printdebug = false;
bool drawdebug = false;

enum {TOP, RIGHT, BOTTOM, LEFT, CENTER};
string direction[] = { "TOP", "RIGHT", "BOTTOM", "LEFT", "CENTER" };

enum { BASE_PLANE, BUOY_TOP, BUOY_BOTTOM, BUOY_LEFT, BUOY_RIGHT, BUOY_UNKNOWN};

// framerate
FpS fps;

osg::Vec3 pixel_pos(float x, float y, float z)
{
    osg::Vec3 pos;
    pos.set(x * (wall_width + field_width),
            (field_count - 1) * (wall_width + field_width) -
            y * (wall_width + field_width), base_thickness / 2 + z);
    return pos;
}

osg::Switch* baseElement(float x, float y)
{
    osg::ref_ptr<osg::TessellationHints> hints = new osg::TessellationHints;
    hints->setDetailRatio(1.0f);
    osg::ref_ptr<osg::ShapeDrawable> shape;
    osg::Geode* geode = new osg::Geode;
    osg::Switch* base_element = new osg::Switch;

    shape = new osg::ShapeDrawable(
        new osg::Box(osg::Vec3(x, y, 0.0f), field_width, field_width, base_thickness),
        hints.get());
    shape->setColor(osg::Vec4(0.0f, 0.0f, 0.8f, 1.0f));
    geode->addDrawable(shape.get());

    shape = new osg::ShapeDrawable(
        new osg::Box(osg::Vec3(x - field_width / 2 - wall_width / 2, y, 0.0f),
                     wall_width, field_width + 2 * wall_width, base_thickness),
        hints.get());
    shape->setColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
    geode->addDrawable(shape.get());

    shape = new osg::ShapeDrawable(
        new osg::Box(osg::Vec3(x + field_width / 2 + wall_width / 2, y, 0.0f),
                     wall_width, field_width + 2 * wall_width, base_thickness),
        hints.get());
    shape->setColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
    geode->addDrawable(shape.get());

    shape = new osg::ShapeDrawable(
        new osg::Box(osg::Vec3(x, y - field_width / 2 - wall_width / 2, 0.0f),
                     field_width, wall_width, base_thickness),
        hints.get());
    shape->setColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
    geode->addDrawable(shape.get());

    shape = new osg::ShapeDrawable(
        new osg::Box(osg::Vec3(x, y + field_width / 2 + wall_width / 2, 0.0f),
                     field_width, wall_width, base_thickness),
        hints.get());
    shape->setColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
    geode->addDrawable(shape.get());

    base_element->addChild(geode);

    //buoy up
    shape = new osg::ShapeDrawable(
        new osg::Box(osg::Vec3(x, y + (field_width / 2 - 0.5), 2.0f),
                     1, 1, 4.0f), hints.get());
    //red, with 60% opacity.
    shape->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 0.6f));

    // Activate blending.
    shape->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);

    geode = new osg::Geode;
    geode->addDrawable(shape.get());
    base_element->addChild(geode);

    //buoy down
    shape = new osg::ShapeDrawable(
        new osg::Box(osg::Vec3(x, y - (field_width / 2 - 0.5), 2.0f),
                     1, 1, 4.0f), hints.get());
    //red, with 60% opacity.
    shape->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 0.6f));
    // Activate blending.
    shape->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
    geode = new osg::Geode;
    geode->addDrawable(shape.get());
    base_element->addChild(geode);

    //buoy left
    shape = new osg::ShapeDrawable(
        new osg::Box(osg::Vec3(x - (field_width / 2 - 0.5), y, 2.0f),
                     1 , 1, 4.0f), hints.get());
    //red, with 60% opacity.
    shape->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 0.6f));
    // Activate blending.
    shape->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
    geode = new osg::Geode;
    geode->addDrawable(shape.get());
    base_element->addChild(geode);

    //buoy right
    shape = new osg::ShapeDrawable(
        new osg::Box(osg::Vec3(x + (field_width / 2 - 0.5), y, 2.0f),
                     1 , 1, 4.0f), hints.get());
    //red, with 60% opacity.
    shape->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 0.6f));
    // Activate blending.
    shape->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
    geode = new osg::Geode;
    geode->addDrawable(shape.get());
    base_element->addChild(geode);

    //buoy center
    shape = new osg::ShapeDrawable(
        new osg::Box(osg::Vec3(x, y, 2.0f), 1 , 1, 4.0f),
        hints.get());
    //red, with 60% opacity.
    shape->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 0.6f));
    // Activate blending.
    shape->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
    geode = new osg::Geode;
    geode->addDrawable(shape.get());
    base_element->addChild(geode);

    base_element->setValue(BUOY_TOP, false);
    base_element->setValue(BUOY_BOTTOM, false);
    base_element->setValue(BUOY_RIGHT, false);
    base_element->setValue(BUOY_LEFT, false);
    base_element->setValue(BUOY_UNKNOWN, false);


    return base_element;
}

osg::Switch* createBase()
{
    osg::Switch* base = new osg::Switch;

    osg::ref_ptr<osg::TessellationHints> hints = new osg::TessellationHints;
    hints->setDetailRatio(2.0f);

    osg::ref_ptr<osg::ShapeDrawable> shape;

    float center_x, center_y;

    for (int y = 0; y < field_count; y++)
    {
        center_y = (field_count - 1) * (field_width + wall_width) -
                   y * (field_width + wall_width);

        for (int x = 0; x < field_count; x++)
        {
            center_x = x * (field_width + wall_width);
            base->addChild(baseElement(center_x, center_y));
        }

    }

    return base;
}

osg::Geode* createJunction(float x, float y)
{
    osg::ref_ptr<osg::TessellationHints> hints = new osg::TessellationHints;
    hints->setDetailRatio(1.0f);
    osg::ref_ptr<osg::ShapeDrawable> shape;
    osg::Geode* geode = new osg::Geode;
    shape = new osg::ShapeDrawable(
        new osg::Box(osg::Vec3(x, y, wall_hight / 2 + base_thickness / 2),
                     wall_width, wall_width, wall_hight),
        hints.get());
    shape->setColor(osg::Vec4(0.4f, 0.4f, 0.4f, 0.8f));
    // Activate blending.
    shape->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
    geode->addDrawable(shape.get());
    return geode;
}

osg::Switch* createWalls()
{
    osg::Switch* walls = new osg::Switch;
    osg::ref_ptr<osg::TessellationHints> hints = new osg::TessellationHints;
    hints->setDetailRatio(1.0f);
    osg::ref_ptr<osg::ShapeDrawable> shape;

    float x_center, y_center;

    y_center = field_count * (field_width + wall_width) -
               field_width / 2 - wall_width / 2;
    for (int y = 0; y < field_count * 2 + 1; y++)
    {
        if (y % 2 == 0)
        {
            walls->addChild(createJunction(-field_width / 2 - wall_width / 2, y_center));
            for (int x = 0; x < field_count; x++)
            {
                x_center = x * (field_width + wall_width);
                osg::Geode* geode = new osg::Geode;
                shape = new osg::ShapeDrawable(
                    new osg::Box(osg::Vec3(x_center, y_center,
                                           wall_hight / 2 + base_thickness / 2),
                                 field_width, wall_width, wall_hight),
                    hints.get());
                // white, with 50% opacity.
                shape->setColor(osg::Vec4(0.8f, 0.8f, 0.8f, 0.8f));
                // Activate blending.
                shape->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
                geode->addDrawable(shape.get());
                //shape->setUseDisplayList(false);
                walls->addChild(geode);
                walls->addChild(createJunction(x_center + field_width / 2 +
                                               wall_width / 2, y_center));
            }
        }
        else
        {
            for (int x = 0; x < field_count + 1; x++)
            {
                x_center = x * (field_width + wall_width) - field_width / 2 - wall_width / 2;
                osg::Geode* geode = new osg::Geode;
                shape = new osg::ShapeDrawable(
                    new osg::Box(osg::Vec3(x_center, y_center,
                                           wall_hight / 2 + base_thickness / 2),
                                 wall_width, field_width, wall_hight),
                    hints.get());
                // white, with 50% opacity.
                shape->setColor(osg::Vec4(0.8f, 0.8f, 0.8f, 0.8f));
                // Activate blending.
                shape->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
                geode->addDrawable(shape.get());
                //shape->setUseDisplayList(false);
                walls->addChild(geode);
            }
        }
        y_center -= field_width / 2 + wall_width / 2;
    }

    return walls;

}

void setWall(int x, int y, int where, bool state, LabyrinthModel& lab)
{
    int index = 0;
    switch (where)
    {
    case TOP:
        index = y * (field_count * 2 + 1) + y * (field_count + 1) + x * 2;
        lab.walls->setValue(index, state); // connector left
        lab.walls->setValue(index + 1, state); // wall top
        lab.walls->setValue(index + 2, state); // connector right
        break;
    case BOTTOM:
        index = (y + 1) * (field_count * 2 + 1) + (y + 1) * (field_count + 1) + x * 2;
        lab.walls->setValue(index, state); // connector left
        lab.walls->setValue(index + 1, state); // wall bottom
        lab.walls->setValue(index + 2, state); // connector right
        break;
    case LEFT:
        index = (y + 1) * (field_count * 2 + 1) + y * (field_count + 1) + x - 1;
        lab.walls->setValue(index + 1, state); // wall eft
        index = y * (field_count * 2 + 1) + y * (field_count + 1) + x * 2;
        lab.walls->setValue(index, state); // connector top
        index = (y + 1) * (field_count * 2 + 1) + (y + 1) * (field_count + 1) + x * 2;
        lab.walls->setValue(index, state); // connector bottom
        break;
    case RIGHT:
        index = (y + 1) * (field_count * 2 + 1) + y * (field_count + 1) + x;
        lab.walls->setValue(index + 1, state); // wall right
        index = y * (field_count * 2 + 1) + y * (field_count + 1) + x * 2;
        lab.walls->setValue(index + 2, state); // connector top
        index = (y + 1) * (field_count * 2 + 1) + (y + 1) * (field_count + 1) + x * 2;
        lab.walls->setValue(index + 2, state); // connector bottom
        break;
    }

}

void update_display(LabyrinthModel& lab)
{
    lab.walls->setAllChildrenOff();

    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            if (lt_map.fields[j][i].wall_top)
                setWall(j, i, TOP, true, lab);
            if (lt_map.fields[j][i].wall_bottom)
                setWall(j, i, BOTTOM, true, lab);
            if (lt_map.fields[j][i].wall_left)
                setWall(j, i, LEFT, true, lab);
            if (lt_map.fields[j][i].wall_right)
                setWall(j, i, RIGHT, true, lab);

        }
    }

    int robots_count = lt_robots.robots.size();
    int nodes_count = lab.robots->getNumChildren();
    if (robots_count < nodes_count)
        lab.robots->removeChildren(robots_count, nodes_count);
    if (robots_count > nodes_count)
    {
        for (int i = 0; i < robots_count - nodes_count; i++)
        {
            // Supported formats:
            //http://www.openscenegraph.org/projects/osg/wiki/Support/UserGuides/Plugins
            osg::Node* robotNode = osgDB::readNodeFile(get_selfdir(printdebug) +
                                   "../../../src/laustracker3d/robolaus.obj");
            osg::MatrixTransform* robotRotate = new osg::MatrixTransform;
            robotRotate->setMatrix(osg::Matrix::rotate(osg::inDegrees(0.0f), 0.0f, 0.0f, 1.0f));
            robotRotate->addChild(robotNode);
            osg::PositionAttitudeTransform* robotPos;
            robotPos = new osg::PositionAttitudeTransform();
            robotPos->addChild(robotRotate);
            robotPos->setPosition(pixel_pos(i, i, 0));

            osgText::Text* robotLabel = new osgText::Text();
            robotLabel->setCharacterSize(2);
            robotLabel->setFont(get_selfdir(printdebug) +
                                "../../../src/laustracker3d/Glass_TTY_VT220.ttf");
            robotLabel->setText("Robot #1");
            robotLabel->setAxisAlignment(osgText::Text::SCREEN);
            robotLabel->setDrawMode(osgText::Text::TEXT | osgText::Text::BOUNDINGBOX);
            robotLabel->setAlignment(osgText::Text::CENTER_TOP);
            robotLabel->setPosition(osg::Vec3(0, 0, 7));
            robotLabel->setColor(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
            osg::Geode* robotLabelGeode = new osg::Geode();
            robotLabelGeode->addDrawable(robotLabel);
            robotPos->addChild(robotLabelGeode);

            lab.robots->addChild(robotPos);
            lab.robots->setValue(0, true);
        }
    }

    if (robots_count > 0)
    {
        int i = 0;
        for (list<robot_t>::iterator robot = lt_robots.robots.begin();
                robot != lt_robots.robots.end(); ++robot)
        {
            osg::PositionAttitudeTransform* robotPos =
                dynamic_cast<osg::PositionAttitudeTransform *>(lab.robots->getChild(i));
            if (no_relativ_pos)
                robotPos->setPosition(
                    pixel_pos((*robot).field_idx.x, (*robot).field_idx.y, 0));
            else
            {
                osg::Vec3 pos = osg::Vec3(pixel_pos((*robot).field_idx.x, (*robot).field_idx.y, 0));
                pos.x() -= (*robot).relative_pos.x * field_width;
                pos.y() += (*robot).relative_pos.y * field_width;

                robotPos->setPosition(pos);
            }

            stringstream name;
            name << (*robot).name;
            if ((*robot).third_led_found)
            {
                name << std::fixed << std::setprecision(3); // display 3 digits
                name << " (" << (*robot).ratio << ")";
                name <<  std::setprecision(0);
                name << "[" << round((*robot).absolut_pos_cm.x) << ",";
                name << round((*robot).absolut_pos_cm.y) << "]";
                //name << "\n[" << (*robot).absolut_pos_px.x << ",";
                //name << (*robot).absolut_pos_px.y << "]";
            }
            osgText::Text* robotLabel =
                dynamic_cast<osgText::Text *>
                (robotPos->getChild(1)->asGeode()->getDrawable(0));
            robotLabel->setText(name.str());

            osg::MatrixTransform* robotRotate =
                dynamic_cast<osg::MatrixTransform *>(robotPos->getChild(0));
            robotRotate->setMatrix(
                osg::Matrix::rotate(
                    osg::inDegrees(-((*robot).angle - 90)), 0.0f, 0.0f, 1.0f));

            i++;
        }
    }
}

osg::Group* createHUDText(float windowWidth, float windowHeight)
{

    osg::Group* fontNode = new osg::Group;

    osgText::Font* font = osgText::readFontFile(get_selfdir(printdebug) +
                          "../../../src/laustracker3d/Glass_TTY_VT220.ttf");

    osg::Geode* geode  = new osg::Geode;
    fontNode->addChild(geode);

    float margin = 50.0f;
    osg::Vec4 layoutColor(1.0f, 1.0f, 0.0f, 1.0f);
    osg::Vec4 fontColor(0.0f, 1.0f, 1.0f, 1.0f);
    osg::Vec3 cursor = osg::Vec3(margin, windowHeight - margin, 0.0f);
    float fontCharacterSize = 25;

    osgText::Text* text = new osgText::Text;
    text->setFont(font);
    text->setColor(fontColor);
    text->setCharacterSize(fontCharacterSize);
    text->setPosition(cursor);

    // use text that uses 50 by 50 texels as a target resolution for fonts.
    // even smoother but again higher texture memory usage.
    text->setFontResolution(50, 50);
    text->setText("Laustracker3D using OpenSceneGraph");
    geode->addDrawable(text);

    cursor.x() = windowWidth - 304;
    cursor.y() = windowHeight - 304 - fontCharacterSize;

    osgText::Text* fps_text = new osgText::Text;
    fps_text->setFont(font);
    fps_text->setColor(fontColor);
    fps_text->setCharacterSize(fontCharacterSize);
    fps_text->setPosition(cursor);

    // use text that uses 50 by 50 texels as a target resolution for fonts.
    // even smoother but again higher texture memory usage.
    fps_text->setFontResolution(50, 50);
    fps_text->setText("FPS: ?");
    geode->addDrawable(fps_text);

    // Warning: if you add more text, then you have to update the indices
    // in draw_text() and draw_fps_text()

    fontCharacterSize = 14.0f;

    cursor.x() = margin;
    cursor.y() = margin + 2 * fontCharacterSize;

    for (int i = 0; i < 5; i++)
    {
        osgText::Text* text = new osgText::Text;
        text->setColor(fontColor);
        text->setPosition(cursor);
        text->setCharacterSize(fontCharacterSize);

        text->setFont(font);
        std::ostringstream str;
        str << i;
        text->setText("line " + str.str());
        text->setName("line" + str.str());
        geode->addDrawable(text);
        cursor.y() -= fontCharacterSize;

    }

    return fontNode;
}

void replaceAll(std::string& str, const std::string& from, const std::string& to)
{
    size_t start_pos = 0;
    while ((start_pos = str.find(from, start_pos)) != std::string::npos)
    {
        str.replace(start_pos, from.length(), to);
        // In case 'to' contains 'from', like replacing 'x' with 'yx'
        start_pos += to.length();
    }
}

void draw_text(string msg, LabyrinthModel& lab)
{
    osgText::Text *text1 =
        dynamic_cast<osgText::Text *>(lab.text->getChild(0)->asGeode()->getDrawable(2));
    osgText::Text *text2 =
        dynamic_cast<osgText::Text *>(lab.text->getChild(0)->asGeode()->getDrawable(3));
    osgText::Text *text3 =
        dynamic_cast<osgText::Text *>(lab.text->getChild(0)->asGeode()->getDrawable(4));
    osgText::Text *text4 =
        dynamic_cast<osgText::Text *>(lab.text->getChild(0)->asGeode()->getDrawable(5));
    osgText::Text *text5 =
        dynamic_cast<osgText::Text *>(lab.text->getChild(0)->asGeode()->getDrawable(6));

    static string str1 = "", str2 = "", str3 = "", str4 = "", str5 = "";

    str1 = str2; str2 = str3; str3 = str4; str4 = str5;

    replaceAll(msg, "\n", "\\n ");

    if (msg.length() > 160)
        str5 = "# " + msg.substr(0, 160) + " ...";
    else
        str5 = "# " + msg;

    text1->setText(str1); text2->setText(str2);
    text3->setText(str3); text4->setText(str4);
    text5->setText(str5);

}

void draw_fps_text(string fps, LabyrinthModel& lab)
{
    osgText::Text *text =
        dynamic_cast<osgText::Text *>(lab.text->getChild(0)->asGeode()->getDrawable(1));
    text->setText(fps);
}

osg::Geode* createCamView(float x, float y, float width, float height)
{
    osg::Geode* camview = new osg::Geode();
    osg::StateSet* stateset = camview->getOrCreateStateSet();
    stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    osg::ref_ptr<osg::Geometry> quad = osg::createTexturedQuadGeometry(osg::Vec3(x, y, 0.0f), //center
                                       osg::Vec3(width, 0.0f, 0.0f), //width
                                       osg::Vec3(0.0f, height, 0.0)); //height

    camview->addDrawable(quad.release());

    return camview;
}

void setCamViewImage(LabyrinthModel& lab, Mat image)
{
    // Adding borders to the image
    copyMakeBorder(image, image, 2, 2, 2, 2, BORDER_CONSTANT, Scalar(159, 159, 0));

    osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
    osg::Image* s = new osg::Image();
    cvtColor(image, image, CV_BGR2RGB);
    s->setImage(image.size().width,
                image.size().height,
                3, 3, GL_RGB, GL_UNSIGNED_BYTE,
                (unsigned char*)image.data, osg::Image::NO_DELETE, 1);
    texture->setResizeNonPowerOfTwoHint(false);
    //texture->setTextureSize(304, 304);
    texture->setImage(s);
    osg::StateSet* ss = lab.camview->getDrawable(0)->getOrCreateStateSet();
    ss->setTextureAttributeAndModes(0, texture.get());
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
}

const osg::Vec2 getScreenResolution(unsigned int screenID)
{
    osg::GraphicsContext::WindowingSystemInterface* wsi =
        osg::GraphicsContext::getWindowingSystemInterface();

    unsigned int pWidth, pHeight;
    wsi->getScreenResolution(osg::GraphicsContext::ScreenIdentifier(screenID),
                             pWidth, pHeight);
    return osg::Vec2(pWidth, pHeight);
}

void mapCallback(const std_msgs::String::ConstPtr& msg, LabyrinthModel& lab)
{
    if (printdebug) ROS_INFO("I heard:\n%s\n", msg->data.c_str());
    String msgstr(msg->data.c_str());
    if (msgstr.find(marker_map_begin) == 0)
    {
        if (printdebug) cout << "\nmap: " << msgstr << "\n";
        if (drawdebug) draw_text(msgstr, lab);
        lt_map.fromString(msgstr);
        update_display(lab);
    }
}

void robotsCallback(const std_msgs::String::ConstPtr& msg, LabyrinthModel& lab)
{
    if (printdebug)ROS_INFO("I heard:\n%s\n", msg->data.c_str());
    String msgstr(msg->data.c_str());
    if (msgstr.find(marker_robots_begin) == 0)
    {
        if (printdebug) cout << "\nrobots: " << msgstr << "\n";
        if (drawdebug) draw_text(msgstr, lab);
        lt_robots.fromString(msgstr);
        update_display(lab);
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg, LabyrinthModel& lab)
{
    frame_counter++;
    if (printdebug) ROS_INFO("I got a new Image: %lu\n", frame_counter);
    //================ get image and convert to cv::Mat ==============
    Mat input;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
        input = cv_ptr->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    setCamViewImage(lab, input);

    if (drawdebug)
    {
        stringstream ss;
        ss << "I got a new Image: " << frame_counter;
        draw_text(ss.str(), lab);
    }

    // print current framerate
    fps.measure();
    if (fps.isNew())
    {
        stringstream ss_fps;
        ss_fps << "FPS: ";
        ss_fps.precision(4);
        ss_fps << fps.fps << "\n";
        cout << ss_fps.str();
        draw_fps_text(ss_fps.str(), lab);
    }
}

int main(int argc, char **argv)
{
    osg::ArgumentParser arguments(&argc, argv);

    //The following section of code sets up a viewer to see the scene we create,
    //a 'group' instance to serve as the root of the scene graph,
    //a geometry node (geode) to collect drawables, and a geometry
    //instance to associate vertices and vertex data.

    // construct the viewer.
    osgViewer::Viewer viewer;

    osg::Vec2 screensize = getScreenResolution(0);

    LabyrinthModel lab;

    if (arguments.read("-h") || arguments.read("--help"))
    {
        cout << "usage:\n";
        cout << " -w\n";
        cout << "    window mode, not fullscreen\n";
        cout << " -n\n";
        cout << "    do not use relative positions\n";
        cout << " -p\n";
        cout << "    print debugging informations\n";
        cout << " -d\n";
        cout << "    draw debug messages on the screen\n";
        cout << " -t\n";
        cout << "    load test map\n";
        exit(0);
    }

    bool window = false;
    if (arguments.read("-w"))
    {
        window = true;
        screensize = osg::Vec2(screensize.x() - 100, screensize.y() - 100);
    }
    cout << "Screensize: " << screensize.x() << "x" << screensize.y() << "\n";
    if (arguments.read("-n"))
        no_relativ_pos = true;
    if (arguments.read("-p"))
        printdebug = true;
    if (arguments.read("-d"))
        drawdebug = true;

    lab.root = new osg::Group();
    lab.root->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    lab.robots = new osg::Switch();
    lab.root->addChild(lab.robots);

    lab.base = createBase();
    lab.root->addChild(lab.base);

    lab.walls = createWalls();
    lab.root->addChild(lab.walls);

    osg::MatrixTransform* rootnode = new osg::MatrixTransform;
    rootnode->setMatrix(osg::Matrix::rotate(osg::inDegrees(30.0f), 1.0f, 0.0f, 0.0f));
    rootnode->addChild(lab.root);

    // run optimization over the scene graph
    osgUtil::Optimizer optimzer;
    optimzer.optimize(rootnode);

    // create the hud.
    osg::Camera* camera = new osg::Camera;
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setProjectionMatrix(osg::Matrix::ortho2D(0, screensize.x(), 0, screensize.y()));
    camera->setViewMatrix(osg::Matrix::identity());
    camera->setClearMask(GL_DEPTH_BUFFER_BIT);
    lab.text = createHUDText(screensize.x(), screensize.y());
    camera->addChild(lab.text);
    camera->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    rootnode->addChild(camera);

    // set the scene to render
    viewer.setSceneData(rootnode);

    viewer.setCameraManipulator(new osgGA::TrackballManipulator());

    // TODO: replace that with a new handler, allowing only to
    // resize with fixed aspect ratio, or adjust the camview size
    // to keep the correct aspect ratio of the camview image
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());

    if (window)
        viewer.setUpViewInWindow(50, 50, screensize.x(), screensize.y());  //windowed

    viewer.getCamera()->setClearColor(osg::Vec4(0, 0, 0, 0)); //background color black
    viewer.realize();

    if (arguments.read("-t"))
    {
        lt_map.fromString(map_str);
        lt_robots.fromString(robots_str);
        update_display(lab);
    }

    ros::init(argc, argv, "laustracker3D");

    ros::NodeHandle n;

    ros::Subscriber map_sub = n.subscribe<std_msgs::String>("map_publisher",
                              5, boost::bind(mapCallback, _1, lab));
    ros::Subscriber robots_sub = n.subscribe<std_msgs::String>("robots_publisher",
                                 5, boost::bind(robotsCallback, _1, lab));

#ifdef WITH_CAMVIEW
    image_transport::ImageTransport it(n);
    image_transport::Subscriber imagesub;

    Mat init = Mat::zeros(Size(304, 304), CV_8UC3);
    // image + border
    lab.camview = createCamView(screensize.x() - 304, screensize.y() - 304, 304, 304);
    setCamViewImage(lab, init);
    camera->addChild(lab.camview);
    imagesub = it.subscribe("laustracker/image", 5, boost::bind(imageCallback, _1, lab));
#endif

    draw_text("Hello World!", lab);
    draw_text("This is Laustracker 3D", lab);
    draw_text("You are currently looking at the message area", lab);

    while (!viewer.done() && ros::ok())
    {
        viewer.frame();
        ros::spinOnce();
    }

    return 0;
}
