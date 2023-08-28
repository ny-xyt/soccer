/** --------------------------------------------------------
 * 
 *                      MOTION
 * 
 * This class is intended for use with various parameters,
 *   animation coordinates for example, to create smooth 
 *    movements based on easing mathematical functions.
 * 
 * The class is capable of modifying all numerical types,
 *   be it integer coordinates or floating point values.
 * 
 *    Easing visual examples at: https://easings.net/
 * 
-------------------------------------------------------- **/

#ifndef MOTION_CORE_H
#define MOTION_CORE_H

#include <vector>
#include <deque>
#include <fstream>

#include "EasingFunctions.h"

namespace Motion
{

using TimeType = uint32_t;

/** Singme motion parameters */
template<typename ValueType>
struct MotionParameters
{
    /** Type of easing */
    Type motion_type;

    /** Type of easing */
    Acceleration accel_type;

    /** Duration of the animation (fraction of the total) */
    double duration {0.5};

    /** Length of the animation (fraction of the total) */
    double length {0.5};

    /** Starting value of the function */
    ValueType start_value {0};

    /** Ending value of the function */
    ValueType end_value {1};

    /** Extra modifier for Bounce/Elastic/Pow/Exponential */
    double modifier {};

    /** Gravity modifier for Bounce/Elastic */
    double gravity {};

    /** Elapsed time */
    TimeType elapsed_time {};
};

/** Motion parameters queue type */
template<typename ValueType>
using MotionQueue = std::vector<MotionParameters<ValueType>>;

/** Easing class */
template<typename ValueType>
class MotionCore
{
    using MotionIdx = uint8_t;
    using Interpolated = std::deque<ValueType>;
    
private:
    
    /** Starting value */
    ValueType start_value {};
    
    /** Target value */
    ValueType end_value {};
    
    /** Total duration in frames */
    TimeType total_duration {};
    
    /** Queue of animations */
    MotionQueue<ValueType> motion_queue;
    
    /** Queue of interpolated values */
    Interpolated interpolated_values;
    
    /** Current value */
    ValueType current_value {};
    
    /** Current animation starting point */
    double current_start_value {};
    
    /** Current animation ending point */
    double current_end_value {};
    
    /** Runtime calculation flag */
    bool runtime_calculation {};
    
public:
        
    /** Constructor */
    explicit MotionCore( bool runtime_calculation = true )
        : runtime_calculation( runtime_calculation )
    {}
    
    /** Reset animation */
    inline void Reset() 
    {
        current_value = start_value;
        current_start_value = start_value;
        current_end_value = end_value;
    }
    
    /** Check if animation has finished */
    inline bool HasFinished() 
    {
        if( runtime_calculation )
        {
            return motion_queue.empty();
        }
        else
        {
            return interpolated_values.empty();
        }
    }
    
    /** Advance to next frame */
    void AdvanceToNext() 
    {
        if( runtime_calculation )
        {
            CalculateNext();
        }
        else
        {
            if( !interpolated_values.empty() )
            {
                current_value = interpolated_values.front();
                interpolated_values.pop_front();
            }
        }
    }
    
    /** Calculate next value */
    void CalculateNext()
    {
        if( motion_queue.empty() )
        {
            return;
        }

        motion_queue.back().elapsed_time++;

        if( CalculateCurrentEasingValue() )
        {
            motion_queue.pop_back();
            if( !motion_queue.empty() )
            {
                motion_queue.back().elapsed_time = 0;
                current_start_value = current_end_value;
                current_end_value += (end_value-start_value) * motion_queue.back().length;
            }
        }
    }
    
    /** Fill interpolation vector */
    void FillInterpolationVector()
    {
        interpolated_values.clear();

        // Calculate easing values
        for( decltype(total_duration) t = 0; t < total_duration; ++t )
        {
            CalculateNext();
            interpolated_values.emplace_back( current_value );
        }

        Reset();
    }


/** ACCESSORS */


    /** Set current value */
    inline void SetCurrentValue( ValueType currentValue )
    {
        current_value = currentValue;
    }

    /** Get current value */
    inline ValueType GetCurrentValue()
    {
        return current_value;
    }
    
    /** Set starting value */
    inline void SetStartingValue( ValueType startingValue )
    {
        start_value = startingValue;
    } 
    
    /** Get starting value */
    inline ValueType GetStartingValue()
    {
        return start_value;
    }
    
    /** Set ending value */
    inline void SetEndingValue( ValueType endingValue )
    {
        end_value = endingValue;
    } 
    
    /** Get starting value */
    inline double GetEndingValue() 
    {
        return end_value;
    }
    
    /** Set frame duration value */
    inline void SetFrameDuration( TimeType frameDuration )
    {
        total_duration = frameDuration;
    } 
    
    /** Get frame duration */
    inline TimeType GetFrameDuration()
    {
        return total_duration;
    }
    
    /** Set motion parameters queue */
    inline void SetMotionQueue( MotionQueue<ValueType>&& params )
    {
        motion_queue = MotionQueue<ValueType>( std::move(params) );
        // Calculate new ending value
        current_start_value = start_value;
        current_end_value = start_value + (end_value-start_value) * motion_queue.back().length;
        
        if( std::fabs(current_start_value - current_end_value) <= std::numeric_limits<decltype(current_start_value)>::epsilon() )
        {
            motion_queue.clear();
            interpolated_values.clear();
            Reset();
            return;
        }

        if( !runtime_calculation )
        {
            FillInterpolationVector();
        } 
    }

    /** Get motion parameters queue */
    inline MotionQueue<ValueType>& GetMotionQueue()
    {
        return motion_queue;
    }

    /** Get interpolated values queue */
    inline const Interpolated& GetInterpolatedValues()
    {
        return interpolated_values;
    }
    
    /** Set parameters */
    inline void SetParameters(
        ValueType start_value,
        ValueType end_value,
        TimeType frame_duration,
        MotionQueue<ValueType> params )
    {
        SetStartingValue( start_value );
        SetEndingValue( end_value );
        SetFrameDuration( frame_duration );
        SetCurrentValue( start_value );
        SetMotionQueue( std::move(params) );
    }
    
    /** Set linear parameters */
    inline void SetParameters(
        ValueType start_value,
        ValueType end_value,
        TimeType frame_duration,
        Type type = Type::SINE,
        double duration_split = 0.5,
        double modifier = 4,
        double gravity = 2 )
    {
        SetStartingValue( start_value );
        SetEndingValue( end_value );
        SetFrameDuration( frame_duration );
        SetCurrentValue( start_value );
        SetMotionQueue( 
            {
                {type, Acceleration::OUT, 1-duration_split, 0.5, 0, 1, modifier, gravity},
                {type, Acceleration::IN,    duration_split, 0.5, 0, 1, modifier, gravity},
            }
        );
    }
    

    /** Set runtime calculation flag */
    inline void SetRuntimeCalculation( bool runtime )
    {
        runtime_calculation = runtime;
    }

    /** Dump interpolated values to file for plotting */
    inline void DumpToFile( const std::string& path )
    {
        std::ofstream file( path == std::string() ? "motion_plot.xls" : path );
        for( const auto value : interpolated_values )
        {
            file << value << std::endl;
        }
        file.close();
    }

    /** Calculate current animation value */
    bool CalculateCurrentEasingValue()
    {
        const auto& element = motion_queue.back();

        // Calculate current motion progress
        auto progress = static_cast<double>(element.elapsed_time) / (total_duration * element.duration);

        // Check for progress completion
        if( std::fabs(1.0 - progress) <= 0.1 )
        {
            current_value = current_end_value;
            return true;
        }
        else if( progress < std::numeric_limits<decltype(progress)>::epsilon() )
        {
            progress = std::numeric_limits<decltype(progress)>::epsilon();
        }

        // Calculate current step
        double step = EasingFunctions::GetFunctionValue( progress,
                                                         static_cast<double>(element.start_value),
                                                         static_cast<double>(element.end_value),
                                                         element.motion_type,
                                                         element.accel_type,
                                                         element.modifier,
                                                         element.gravity );

        // Calculate new value
        auto length = std::fabs(current_end_value - current_start_value);
        step *= length;

        if( current_start_value > current_end_value )
        {
            current_value = (length - step) + current_end_value;
        }
        else
        {
            current_value = step + current_start_value;
        }

        return false;
    }
    
}; // class MotionCore

} // namespace egt

#endif /** MOTION_H */
