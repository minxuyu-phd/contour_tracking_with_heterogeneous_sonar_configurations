/*
    This file is a part of Stonefish.

    Stonefish is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Stonefish is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//
//  EchoSounder.cpp
//  Stonefish
//
//  Created for Echo Sounder support.
//  Based on Profiler implementation by Patryk Cieslak.
//

#include "sensors/scalar/EchoSounder.h"

#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
#include "core/SimulationApp.h"
#include "core/SimulationManager.h"
#include "utils/UnitSystem.h"
#include "sensors/Sample.h"
#include "graphics/OpenGLContent.h"

namespace sf
{

EchoSounder::EchoSounder(std::string uniqueName, Scalar beamWidthDeg, bool positiveZ,
                         Scalar frequency, int historyLength)
    : LinkSensor(uniqueName, frequency, historyLength)
{
    beamWidth = UnitSystem::Angle(true, beamWidthDeg);
    beamPosZ = positiveZ;
    depth = Scalar(0);
    detected = false;

    // Single output channel: depth/altitude
    channels.push_back(SensorChannel("Depth", QuantityType::LENGTH));
    channels[0].rangeMin = Scalar(0.5);   // Default minimum range 0.5m (blanking distance)
    channels[0].rangeMax = Scalar(100);   // Default maximum range 100m
}

void EchoSounder::InternalUpdate(Scalar dt)
{
    Transform sensorTrans = getSensorFrame();

    // Beam direction: along sensor frame Z axis
    Vector3 dir = sensorTrans.getBasis().getColumn(2);
    if(!beamPosZ)
        dir = -dir;  // Point downward (negative Z)

    Vector3 from = sensorTrans.getOrigin() + dir * channels[0].rangeMin;
    Vector3 to = sensorTrans.getOrigin() + dir * channels[0].rangeMax;

    // Perform ray test
    btCollisionWorld::ClosestRayResultCallback closest(from, to);
    closest.m_collisionFilterGroup = MASK_DYNAMIC;
    closest.m_collisionFilterMask = MASK_STATIC | MASK_DYNAMIC | MASK_ANIMATED_COLLIDING;
    SimulationApp::getApp()->getSimulationManager()->getDynamicsWorld()->rayTest(from, to, closest);

    if(closest.hasHit())
    {
        Vector3 p = from.lerp(to, closest.m_closestHitFraction);
        depth = (p - sensorTrans.getOrigin()).length();
        detected = true;
    }
    else
    {
        depth = channels[0].rangeMax;
        detected = false;
    }

    // Record sample (noise is applied automatically by AddSampleToHistory if stdDev > 0)
    Sample s{std::vector<Scalar>({depth})};
    AddSampleToHistory(s);
}

void EchoSounder::setRange(Scalar rangeMin, Scalar rangeMax)
{
    channels[0].rangeMin = btClamped(rangeMin, Scalar(0), Scalar(BT_LARGE_FLOAT));
    channels[0].rangeMax = btClamped(rangeMax, channels[0].rangeMin, Scalar(BT_LARGE_FLOAT));
}

void EchoSounder::setNoise(Scalar rangeStdDev)
{
    channels[0].setStdDev(btClamped(rangeStdDev, Scalar(0), Scalar(BT_LARGE_FLOAT)));
}

Scalar EchoSounder::getBeamWidth() const
{
    return beamWidth;
}

Scalar EchoSounder::getDepth() const
{
    return depth;
}

bool EchoSounder::hasDetection() const
{
    return detected;
}

bool EchoSounder::isBeamPositiveZ() const
{
    return beamPosZ;
}

std::vector<Renderable> EchoSounder::Render()
{
    std::vector<Renderable> items = Sensor::Render();
    if(isRenderable())
    {
        Renderable item;
        item.type = RenderableType::SENSOR_LINES;
        item.model = glMatrixFromTransform(getSensorFrame());
        item.data = std::make_shared<std::vector<glm::vec3>>();
        auto points = item.getDataAsPoints();

        // Draw beam line
        GLfloat dirZ = beamPosZ ? 1.f : -1.f;
        points->push_back(glm::vec3(0, 0, 0));
        points->push_back(glm::vec3(0, 0, dirZ * (GLfloat)depth));

        // Draw beam cone outline at detection point (if detected)
        if(detected && beamWidth > Scalar(0))
        {
            GLfloat radius = (GLfloat)depth * tanf((GLfloat)beamWidth / 2.f);
            GLfloat d = dirZ * (GLfloat)depth;

            // Draw circle at the detection point
            const int segments = 16;
            for(int i = 0; i < segments; ++i)
            {
                GLfloat angle1 = (GLfloat)i / segments * 2.f * (GLfloat)M_PI;
                GLfloat angle2 = (GLfloat)(i + 1) / segments * 2.f * (GLfloat)M_PI;
                points->push_back(glm::vec3(radius * cosf(angle1), radius * sinf(angle1), d));
                points->push_back(glm::vec3(radius * cosf(angle2), radius * sinf(angle2), d));
            }

            // Draw cone edges (4 lines from origin to circle)
            for(int i = 0; i < 4; ++i)
            {
                GLfloat angle = (GLfloat)i / 4.f * 2.f * (GLfloat)M_PI;
                points->push_back(glm::vec3(0, 0, 0));
                points->push_back(glm::vec3(radius * cosf(angle), radius * sinf(angle), d));
            }
        }

        items.push_back(item);
    }
    return items;
}

ScalarSensorType EchoSounder::getScalarSensorType() const
{
    return ScalarSensorType::ECHOSOUNDER;
}

}
