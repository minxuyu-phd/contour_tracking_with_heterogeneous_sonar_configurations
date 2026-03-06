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
//  MSIS.h
//  Stonefish
//
//  Created by Patryk Cieslak on 21/07/20.
//  Copyright (c) 2020-2025 Patryk Cieslak. All rights reserved.
//

#ifndef __Stonefish_MSIS__
#define __Stonefish_MSIS__

#include <functional>
#include "sensors/vision/Camera.h"
#include "graphics/OpenGLSonar.h"

namespace sf
{
    class OpenGLMSIS;
    
    //! A class representing a mechanical scanning imaging sonar.
    class MSIS : public Camera
    {
    public:
        /**
         * @brief MSIS（机械扫描成像声纳）构造函数
         *
         * @param uniqueName           传感器唯一名称
         * @param stepAngleDeg         旋转步进角度 [度]
         * @param numOfBins            距离分辨率（声纳图像的径向采样点数）
         * @param horizontalBeamWidthDeg 水平波束宽度 [度]
         * @param verticalBeamWidthDeg   垂直波束宽度 [度]
         * @param minRotationDeg       声纳头最小旋转角度 [度]
         * @param maxRotationDeg       声纳头最大旋转角度 [度]
         * @param minRange             最小探测距离 [米]
         * @param maxRange             最大探测距离 [米]
         * @param cm                   声纳数据显示的颜色映射
         * @param outputFormat         传感器数据输出格式
         * @param frequency            传感器采样频率 [Hz]（-1表示根据最大距离自动计算）
         */
        MSIS(std::string uniqueName,
            Scalar stepAngleDeg,
            unsigned int numOfBins,
            Scalar horizontalBeamWidthDeg,
            Scalar verticalBeamWidthDeg,
            Scalar minRotationDeg,
            Scalar maxRotationDeg,
            Scalar minRange,
            Scalar maxRange,
            ColorMap cm,
            SonarOutputFormat outputFormat,
            Scalar frequency = Scalar(-1));
       
        //! A destructor.
        ~MSIS();
        
        //! A method performing internal sensor state update.
        /*!
         \param dt the step time of the simulation [s]
         */
        void InternalUpdate(Scalar dt) override;
        
        //! A method used to setup the OpenGL sonar transformation.
        /*!
         \param eye the position of the sonar eye [m]
         \param dir a unit vector parallel to the central axis of the sonar
         \param up a unit vector perpendicular to the sonar scanning plane
         */
        void SetupCamera(const Vector3& eye, const Vector3& dir, const Vector3& up) override;
        
        //! A method used to inform about new data.
        /*!
         \param index the id of the OpenGL camera (here sonar) uploading the data
         */
        void NewDataReady(void* data, unsigned int index = 0) override;
        
        //! A method used to set a callback function called when new data is available.
        /*!
         \param callback a function to be called
         */
        void InstallNewDataHandler(std::function<void(MSIS*)> callback);
        
        //! A method implementing the rendering of the sonar dummy.
        std::vector<Renderable> Render();

        //! A method setting the limits of the sonar head rotation.
        /*!
         \param l1Deg first limit of rotation angle [deg]
         \param l2Deg second limit of rotation angle [deg]
         */
        void setRotationLimits(Scalar l1Deg, Scalar l2Deg);
        
        //! A method setting the minimum range of the sonar.
        /*!
         \param r range [m]
         */
        void setRangeMin(Scalar r);

        //! A method setting the minimum range of the sonar.
        /*!
         \param r range [m]
         */
        void setRangeMax(Scalar r);

        //! A method setting the gain of the sonar.
        /*!
         \param g gain factor [1]
         */
        void setGain(Scalar g);

        //! A method setting the noise characteristics of the sensor.
        /*!
         \param multiplicativeStdDev the standard deviation of the multiplicative noise
         \param additiveStdDev the standard deviation of the additive noise
         */
        void setNoise(float multiplicativeStdDev, float additiveStdDev);

        //! A method setting the scanning mode to unidirectional.
        /*!
         \param enabled if true, the sonar scans in one direction only and jumps back to start
         */
        void setUnidirectional(bool enabled);

        //! A method returning whether unidirectional scanning is enabled.
        bool isUnidirectional() const;

        //! A method setting the step multiplier for faster scanning.
        /*!
         \param multiplier the step multiplier (1 = normal, 2 = 2x faster, etc.)
         */
        void setStepMultiplier(int multiplier);

        //! A method returning the current step multiplier.
        int getStepMultiplier() const;

        //! A method returning the rotation limits.
        /*!
         \param l1Deg first limit of rotation angle [deg]
         \param l2Deg second limit of rotation angle [deg]
         */
        void getRotationLimits(Scalar& l1Deg, Scalar& l2Deg) const;

        //! A method returning the minimum range of the sonar.
        Scalar getRangeMin() const;
        
        //! A method returning the maximum range of the sonar.
        Scalar getRangeMax() const;

        //! A method returning the gain of the sonar.
        Scalar getGain() const;

        //! A method returning the step size in degrees.
        Scalar getRotationStepAngle() const;

        //! A method returning the current rotation step.
        int getCurrentRotationStep() const;

        //! A method returning the current bean index in the sonar image.
        GLuint getCurrentBeamIndex() const;

        //! A method returning the output format of the sonar data.
        SonarOutputFormat getOutputFormat() const;

        //! A method returning a pointer to the sonar data.
        /*!
         \param index the id of the OpenGL camera (here sonar) for which the data pointer is requested
         \return pointer to the image data buffer
         */
        void* getImageDataPointer(unsigned int index = 0) override;
        
        //! A method returning the resolution of the simulated display image.
        /*!
         \param x a reference to a variable that will store the horizontal resolution [pix]
         \param y a reference to a variable that will store the vertical resolution [pix]
         */
        void getDisplayResolution(unsigned int& x, unsigned int& y) const;
        
        //! A method returning a pointer to the visualisation image data.
        GLubyte* getDisplayDataPointer();
        
        //! A method returning the type of the vision sensor.
        VisionSensorType getVisionSensorType() const override;

        //! A method returning a pointer to the underlaying OpenGLView object.
        OpenGLView* getOpenGLView() const override;
        
    private:
        void InitGraphics();
        
        OpenGLMSIS* glMSIS;
        void* sonarData;
        GLubyte* displayData;
        int currentStep;
        bool cw;
        glm::ivec2 roi;
        bool fullRotation;
        glm::vec2 range;
        glm::vec2 noise;
        Scalar gain;
        Scalar fovV;
        Scalar stepSize;
        ColorMap cMap;
        SonarOutputFormat outputFormat_;
        bool unidirectional;
        int stepMultiplier;
        std::function<void(MSIS*)> newDataCallback;
    };
}

#endif
