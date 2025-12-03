#include "motion_planner.h"
#include "math.h"

/**
 * @brief 初始化电流跟踪器。
 */
void MotionPlanner::CurrentTracker::Init()
{
    SetCurrentAcc(context->config->ratedCurrentAcc);
}


/**
 * @brief 启动新的电流跟踪任务。
 *
 * @param _realCurrent 当前实际电流。
 */
void MotionPlanner::CurrentTracker::NewTask(int32_t _realCurrent)
{
    currentIntegral = 0;
    trackCurrent = _realCurrent;
}


/**
 * @brief 计算电流跟踪的软目标（平滑过渡）。
 *
 * 根据加速度限制，逐步调整当前跟踪电流向目标电流逼近。
 *
 * @param _goalCurrent 目标电流。
 */
void MotionPlanner::CurrentTracker::CalcSoftGoal(int32_t _goalCurrent)
{
    int32_t deltaCurrent = _goalCurrent - trackCurrent;

    if (deltaCurrent == 0)
    {
        trackCurrent = _goalCurrent;
    } else if (deltaCurrent > 0)
    {
        if (trackCurrent >= 0)
        {
            CalcCurrentIntegral(currentAcc);
            if (trackCurrent >= _goalCurrent)
            {
                currentIntegral = 0;
                trackCurrent = _goalCurrent;
            }
        } else
        {
            CalcCurrentIntegral(currentAcc);
            if ((int32_t) trackCurrent >= 0)
            {
                currentIntegral = 0;
                trackCurrent = 0;
            }
        }
    } else if (deltaCurrent < 0)
    {
        if (trackCurrent <= 0)
        {
            CalcCurrentIntegral(-currentAcc);
            if ((int32_t) trackCurrent <= (int32_t) _goalCurrent)
            {
                currentIntegral = 0;
                trackCurrent = _goalCurrent;
            }
        } else
        {
            CalcCurrentIntegral(-currentAcc);
            if ((int32_t) trackCurrent <= 0)
            {
                currentIntegral = 0;
                trackCurrent = 0;
            }
        }
    }

    goCurrent = (int32_t) trackCurrent;
}


/**
 * @brief 计算电流积分，更新跟踪电流。
 *
 * @param _current 当前的变化量（加速度/加电流度）。
 */
void MotionPlanner::CurrentTracker::CalcCurrentIntegral(int32_t _current)
{
    currentIntegral += _current;
    trackCurrent += currentIntegral / context->CONTROL_FREQUENCY;
    currentIntegral = currentIntegral % context->CONTROL_FREQUENCY;
}


/**
 * @brief 设置电流变化率（加速度）。
 *
 * @param _currentAcc 电流加速度值。
 */
void MotionPlanner::CurrentTracker::SetCurrentAcc(int32_t _currentAcc)
{
    currentAcc = _currentAcc;
}


/**
 * @brief 初始化速度跟踪器。
 */
void MotionPlanner::VelocityTracker::Init()
{
    SetVelocityAcc(context->config->ratedVelocityAcc);
}


/**
 * @brief 设置速度加速度。
 *
 * @param _velocityAcc 速度加速度值。
 */
void MotionPlanner::VelocityTracker::SetVelocityAcc(int32_t _velocityAcc)
{
    velocityAcc = _velocityAcc;
}


/**
 * @brief 启动新的速度跟踪任务。
 *
 * @param _realVelocity 当前实际速度。
 */
void MotionPlanner::VelocityTracker::NewTask(int32_t _realVelocity)
{
    velocityIntegral = 0;
    trackVelocity = _realVelocity;
}


/**
 * @brief 计算速度跟踪的软目标。
 *
 * 根据加速度限制，逐步调整当前跟踪速度向目标速度逼近。
 *
 * @param _goalVelocity 目标速度。
 */
void MotionPlanner::VelocityTracker::CalcSoftGoal(int32_t _goalVelocity)
{
    int32_t deltaVelocity = _goalVelocity - trackVelocity;

    if (deltaVelocity == 0)
    {
        trackVelocity = _goalVelocity;
    } else if (deltaVelocity > 0)
    {
        if (trackVelocity >= 0)
        {
            CalcVelocityIntegral(velocityAcc);
            if (trackVelocity >= _goalVelocity)
            {
                velocityIntegral = 0;
                trackVelocity = _goalVelocity;
            }
        } else
        {
            CalcVelocityIntegral(velocityAcc);
            if (trackVelocity >= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        }
    } else if (deltaVelocity < 0)
    {
        if (trackVelocity <= 0)
        {
            CalcVelocityIntegral(-velocityAcc);
            if (trackVelocity <= _goalVelocity)
            {
                velocityIntegral = 0;
                trackVelocity = _goalVelocity;
            }
        } else
        {
            CalcVelocityIntegral(-velocityAcc);
            if (trackVelocity <= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        }
    }

    goVelocity = (int32_t) trackVelocity;
}


/**
 * @brief 计算速度积分，更新跟踪速度。
 *
 * @param _velocity 速度的变化量（加速度）。
 */
void MotionPlanner::VelocityTracker::CalcVelocityIntegral(int32_t _velocity)
{
    velocityIntegral += _velocity;
    trackVelocity += velocityIntegral / context->CONTROL_FREQUENCY;
    velocityIntegral = velocityIntegral % context->CONTROL_FREQUENCY;
}


/**
 * @brief 初始化位置跟踪器。
 */
void MotionPlanner::PositionTracker::Init()
{
    SetVelocityAcc(context->config->ratedVelocityAcc);

    /*
     *  Allow to locking-brake when velocity is lower than (speedLockingBrake).
     *  The best value should be (ratedMoveAcc/1000)
     */
    speedLockingBrake = context->config->ratedVelocityAcc / 1000;
}


/**
 * @brief 设置位置模式下的加速度。
 *
 * @param value 加速度值。
 */
void MotionPlanner::PositionTracker::SetVelocityAcc(int32_t value)
{
    velocityUpAcc = value;
    velocityDownAcc = value;
    quickVelocityDownAcc = 0.5f / (float) velocityDownAcc;
}


/**
 * @brief 启动新的位置跟踪任务。
 *
 * @param real_location 当前实际位置。
 * @param real_speed 当前实际速度。
 */
void MotionPlanner::PositionTracker::NewTask(int32_t real_location, int32_t real_speed)
{
    velocityIntegral = 0;
    trackVelocity = real_speed;
    positionIntegral = 0;
    trackPosition = real_location;
}


/**
 * @brief 计算位置跟踪的软目标。
 *
 * 使用梯形速度规划或类似的逻辑，计算下一时刻的目标位置和速度，
 * 确保在加速度限制内到达目标位置。
 *
 * @param _goalPosition 最终目标位置。
 */
void MotionPlanner::PositionTracker::CalcSoftGoal(int32_t _goalPosition)
{
    int32_t deltaPosition = _goalPosition - trackPosition;

    if (deltaPosition == 0)
    {
        if ((trackVelocity >= -speedLockingBrake) && (trackVelocity <= speedLockingBrake))
        {
            velocityIntegral = 0;
            trackVelocity = 0;
            positionIntegral = 0;
        } else if (trackVelocity > 0)
        {
            CalcVelocityIntegral(-velocityDownAcc);
            if (trackVelocity <= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        } else if (trackVelocity < 0)
        {
            CalcVelocityIntegral(velocityDownAcc);
            if (trackVelocity >= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        }
    } else
    {
        if (trackVelocity == 0)
        {
            if (deltaPosition > 0)
            {
                CalcVelocityIntegral(velocityUpAcc);
            } else
            {
                CalcVelocityIntegral(-velocityUpAcc);
            }
        } else if ((deltaPosition > 0) && (trackVelocity > 0))
        {
            if (trackVelocity <= context->config->ratedVelocity)
            {
                auto need_down_location = (int32_t) ((float) trackVelocity *
                                                     (float) trackVelocity *
                                                     (float) quickVelocityDownAcc);
                if (abs(deltaPosition) > need_down_location)
                {
                    if (trackVelocity < context->config->ratedVelocity)
                    {
                        CalcVelocityIntegral(velocityUpAcc);
                        if (trackVelocity >= context->config->ratedVelocity)
                        {
                            velocityIntegral = 0;
                            trackVelocity = context->config->ratedVelocity;
                        }
                    } else if (trackVelocity > context->config->ratedVelocity)
                    {
                        CalcVelocityIntegral(-velocityDownAcc);
                    }
                } else
                {
                    CalcVelocityIntegral(-velocityDownAcc);
                    if (trackVelocity <= 0)
                    {
                        velocityIntegral = 0;
                        trackVelocity = 0;
                    }
                }
            } else
            {
                CalcVelocityIntegral(-velocityDownAcc);
                if (trackVelocity <= 0)
                {
                    velocityIntegral = 0;
                    trackVelocity = 0;
                }
            }
        } else if ((deltaPosition < 0) && (trackVelocity < 0))
        {
            if (trackVelocity >= -context->config->ratedVelocity)
            {
                auto need_down_location = (int32_t) ((float) trackVelocity *
                                                     (float) trackVelocity *
                                                     (float) quickVelocityDownAcc);
                if (abs(deltaPosition) > need_down_location)
                {
                    if (trackVelocity > -context->config->ratedVelocity)
                    {
                        CalcVelocityIntegral(-velocityUpAcc);
                        if (trackVelocity <= -context->config->ratedVelocity)
                        {
                            velocityIntegral = 0;
                            trackVelocity = -context->config->ratedVelocity;
                        }
                    } else if (trackVelocity < -context->config->ratedVelocity)
                    {
                        CalcVelocityIntegral(velocityDownAcc);
                    }
                } else
                {
                    CalcVelocityIntegral(velocityDownAcc);
                    if (trackVelocity >= 0)
                    {
                        velocityIntegral = 0;
                        trackVelocity = 0;
                    }
                }
            } else
            {
                CalcVelocityIntegral(velocityDownAcc);
                if (trackVelocity >= 0)
                {
                    velocityIntegral = 0;
                    trackVelocity = 0;
                }
            }
        } else if ((deltaPosition < 0) && (trackVelocity > 0))
        {
            CalcVelocityIntegral(-velocityDownAcc);
            if (trackVelocity <= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        } else if (((deltaPosition > 0) && (trackVelocity < 0)))
        {
            CalcVelocityIntegral(velocityDownAcc);
            if (trackVelocity >= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        }
    }

    CalcPositionIntegral(trackVelocity);

    go_location = (int32_t) trackPosition;
    go_velocity = (int32_t) trackVelocity;
}


/**
 * @brief 计算位置积分，更新跟踪位置。
 *
 * @param value 速度值。
 */
void MotionPlanner::PositionTracker::CalcPositionIntegral(int32_t value)
{
    positionIntegral += value;
    trackPosition += positionIntegral / context->CONTROL_FREQUENCY;
    positionIntegral = positionIntegral % context->CONTROL_FREQUENCY;
}


/**
 * @brief 计算速度积分，更新跟踪速度。
 *
 * @param value 加速度值。
 */
void MotionPlanner::PositionTracker::CalcVelocityIntegral(int32_t value)
{
    velocityIntegral += value;
    trackVelocity += velocityIntegral / context->CONTROL_FREQUENCY;
    velocityIntegral = velocityIntegral % context->CONTROL_FREQUENCY;
}


/**
 * @brief 初始化位置插值器。
 */
void MotionPlanner::PositionInterpolator::Init()
{

}


/**
 * @brief 启动新的插值任务。
 *
 * @param _realPosition 当前实际位置。
 * @param _realVelocity 当前实际速度。
 */
void MotionPlanner::PositionInterpolator::NewTask(int32_t _realPosition, int32_t _realVelocity)
{
    recordPosition = _realPosition;
    recordPositionLast = _realPosition;
    estPosition = _realPosition;
    estVelocity = _realVelocity;
}


/**
 * @brief 计算位置插值器的软目标。
 *
 * 用于平滑来自上位机的步进/方向或粗略位置指令。
 *
 * @param _goalPosition 目标位置。
 */
void MotionPlanner::PositionInterpolator::CalcSoftGoal(int32_t _goalPosition)
{
    recordPositionLast = recordPosition;
    recordPosition = _goalPosition;

    estPositionIntegral += (((recordPosition - recordPositionLast) * context->CONTROL_FREQUENCY)
                            + ((estVelocity << 6) - estVelocity));
    estVelocity = estPositionIntegral >> 6;
    estPositionIntegral -= (estVelocity << 6);

    estPosition = recordPosition;

    goPosition = estPosition;
    goVelocity = estVelocity;
}


/**
 * @brief 设置轨迹跟踪的减速度。
 *
 * @param value 减速度值。
 */
void MotionPlanner::TrajectoryTracker::SetSlowDownVelocityAcc(int32_t value)
{
    velocityDownAcc = value;
}


/**
 * @brief 启动新的轨迹跟踪任务。
 *
 * @param real_location 当前实际位置。
 * @param real_speed 当前实际速度。
 */
void MotionPlanner::TrajectoryTracker::NewTask(int32_t real_location, int32_t real_speed)
{
    updateTime = 0;
    overtimeFlag = false;
    dynamicVelocityAccRemainder = 0;
    velocityNow = real_speed;
    velovityNowRemainder = 0;
    positionNow = real_location;
}


/**
 * @brief 计算轨迹跟踪的软目标。
 *
 * 根据动态目标（位置和速度），计算所需的加速度以跟踪轨迹，
 * 如果更新超时，则减速。
 *
 * @param _goalPosition 目标位置。
 * @param _goalVelocity 目标速度。
 */
void MotionPlanner::TrajectoryTracker::CalcSoftGoal(int32_t _goalPosition, int32_t _goalVelocity)
{
    if (_goalVelocity != recordVelocity || _goalPosition != recordPosition)
    {
        updateTime = 0;
        recordVelocity = _goalVelocity;
        recordPosition = _goalPosition;

        dynamicVelocityAcc = (int32_t) ((float) (_goalVelocity + velocityNow) *
                                        (float) (_goalVelocity - velocityNow) /
                                        (float) (2 * (_goalPosition - positionNow)));

        overtimeFlag = false;
    } else
    {
        if (updateTime >= (updateTimeout * 1000))
            overtimeFlag = true;
        else
            updateTime += context->CONTROL_PERIOD;
    }

    if (overtimeFlag)
    {
        if (velocityNow == 0)
        {
            dynamicVelocityAccRemainder = 0;
        } else if (velocityNow > 0)
        {
            CalcVelocityIntegral(-velocityDownAcc);
            if (velocityNow <= 0)
            {
                dynamicVelocityAccRemainder = 0;
                velocityNow = 0;
            }
        } else
        {
            CalcVelocityIntegral(velocityDownAcc);
            if (velocityNow >= 0)
            {
                dynamicVelocityAccRemainder = 0;
                velocityNow = 0;
            }
        }
    } else
    {
        CalcVelocityIntegral(dynamicVelocityAcc);
    }

    CalcPositionIntegral(velocityNow);

    goPosition = positionNow;
    goVelocity = velocityNow;
}


/**
 * @brief 计算速度积分。
 *
 * @param value 加速度值。
 */
void MotionPlanner::TrajectoryTracker::CalcVelocityIntegral(int32_t value)
{
    dynamicVelocityAccRemainder += value; // sum up last remainder
    velocityNow += dynamicVelocityAccRemainder / context->CONTROL_FREQUENCY;
    dynamicVelocityAccRemainder = dynamicVelocityAccRemainder % context->CONTROL_FREQUENCY; // calc remainder
}


/**
 * @brief 计算位置积分。
 *
 * @param value 速度值。
 */
void MotionPlanner::TrajectoryTracker::CalcPositionIntegral(int32_t value)
{
    velovityNowRemainder += value;
    positionNow += velovityNowRemainder / context->CONTROL_FREQUENCY;
    velovityNowRemainder = velovityNowRemainder % context->CONTROL_FREQUENCY;
}


/**
 * @brief 初始化轨迹跟踪器。
 *
 * @param _updateTimeout 超时时间（毫秒），超过此时间未收到新目标则减速。
 */
void MotionPlanner::TrajectoryTracker::Init(int32_t _updateTimeout)
{
    //SetSlowDownVelocityAcc(context->config->ratedVelocityAcc / 10);
    SetSlowDownVelocityAcc(context->config->ratedVelocityAcc);
    updateTimeout = _updateTimeout;
}


/**
 * @brief 绑定配置结构体，并初始化所有子跟踪器。
 *
 * @param _config 配置结构体指针。
 */
void MotionPlanner::AttachConfig(MotionPlanner::Config_t* _config)
{
    config = _config;

    currentTracker.Init();
    velocityTracker.Init();
    positionTracker.Init();
    positionInterpolator.Init();
    trajectoryTracker.Init(200);
}
