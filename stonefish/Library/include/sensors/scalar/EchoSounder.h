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
//  EchoSounder.h
//  Stonefish
//
//  Created for Echo Sounder support.
//  Based on Profiler implementation by Patryk Cieslak.
//

#ifndef __Stonefish_EchoSounder__
#define __Stonefish_EchoSounder__

#include "sensors/scalar/LinkSensor.h"

namespace sf
{
    //! A class representing a single beam echo sounder for depth/altitude measurement.
    class EchoSounder : public LinkSensor
    {
    public:
        //! A constructor.
        /*!
         \param uniqueName a name for the sensor
         \param beamWidthDeg the beam width for visualization [deg]
         \param positiveZ if true, beam points in positive Z direction; if false, beam points downward (negative Z)
         \param frequency the sampling frequency of the sensor [Hz] (-1 if updated every simulation step)
         \param historyLength defines: -1 -> no history, 0 -> unlimited history, >0 -> history with a specified length
         */
        EchoSounder(std::string uniqueName,
                    Scalar beamWidthDeg = Scalar(10),
                    bool positiveZ = false,
                    Scalar frequency = Scalar(-1),
                    int historyLength = -1);

        //! A method performing internal sensor state update.
        /*!
         \param dt the step time of the simulation [s]
         */
        void InternalUpdate(Scalar dt) override;

        //! A method used to set the range of the sensor.
        /*!
         \param rangeMin the minimum measured range (blanking distance) [m]
         \param rangeMax the maximum measured range [m]
         */
        void setRange(Scalar rangeMin, Scalar rangeMax);

        //! A method used to set the noise characteristics of the sensor.
        /*!
         \param rangeStdDev standard deviation of the range measurement noise [m]
         */
        void setNoise(Scalar rangeStdDev);

        //! A method returning the beam width.
        /*!
         \return beam width [rad]
         */
        Scalar getBeamWidth() const;

        //! A method returning the last measured depth/altitude.
        /*!
         \return depth/altitude [m]
         */
        Scalar getDepth() const;

        //! A method to check if bottom/surface was detected.
        /*!
         \return true if target was detected within range
         */
        bool hasDetection() const;

        //! A method returning whether beam points in positive Z direction.
        /*!
         \return true if beam points in positive Z, false if negative Z (downward)
         */
        bool isBeamPositiveZ() const;

        //! A method implementing the rendering of the sensor.
        std::vector<Renderable> Render() override;

        //! A method returning the type of the scalar sensor.
        ScalarSensorType getScalarSensorType() const override;

    private:
        Scalar beamWidth;       //!< Beam width for visualization [rad]
        bool beamPosZ;          //!< Beam direction: true = +Z, false = -Z (down)
        Scalar depth;           //!< Current measured depth/altitude [m]
        bool detected;          //!< Whether target was detected
    };
}

#endif
