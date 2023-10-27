/*
 * gnd_rosutil.hpp
 *
 *  Created on: 2014/08/04
 *      Author: tyamada
 *  Updated by ryu, 2023/5/22
 *  .Use Qt v5.14 lib to update util functions
 */
#ifndef GND_ROSUTIL_H
#define GND_ROSUTIL_H

#include <QVector>
#include <QtGlobal>
#include <QtMath>

#include "std_msgs/Header.h"
#include "sensor_msgs/Imu.h"
#include "gnd_msgs/msg_velocity2d_with_covariance_stamped.h"
#include "gnd_msgs/msg_pose2d_stamped.h"

namespace gnd {

#define gnd_sign(x)				( ((x) < 0) ? -1 : 1 )
#define gnd_round(x)			( (int) ((x) + 0.5) )
#define gnd_square(x)				((x) * (x))

// ---> distance unit
#define gnd_unit_mm2cm_scale	(10.0)
#define gnd_unit_cm2mm_scale	(0.1)
#define gnd_mm2cm(x)			((x) / gnd_unit_mm2cm_scale)
#define gnd_cm2mm(x)			((x) * gnd_unit_mm2cm_scale)

#define gnd_unit_mm2m_scale		(0.001)
#define gnd_unit_m2mm_scale		(1000.0)
#define gnd_mm2m(x)				((x) / gnd_unit_m2mm_scale)
#define gnd_m2mm(x)				((x) * gnd_unit_m2mm_scale)

#define gnd_unit_cm2m_scale		(0.01)
#define gnd_unit_m2cm_scale		(100.0)
#define gnd_cm2m(x)				((x) * gnd_unit_cm2m_scale)
#define gnd_m2cm(x)				((x) * gnd_unit_m2cm_scale)

#define gnd_unit_dist2mm_scale	gnd_unit_m2mm_scale
#define gnd_unit_mm2dist_scale	gnd_unit_mm2m_scale
#define gnd_unit_dist2cm_scale	gnd_unit_m2cm_scale
#define gnd_unit_cm2dist_scale	gnd_unit_cm2m_scale
#define gnd_unit_dist2m_scale	(1.0)
#define gnd_unit_m2dist_scale	(1.0)

#define gnd_mm2dist(x)			( (x) * gnd_unit_mm2dist_scale )
#define gnd_dist2mm(x)			( (x) * gnd_unit_dist2mm_scale )
#define gnd_cm2dist(x)			( (x) * gnd_unit_cm2dist_scale )
#define gnd_dist2cm(x)			( (x) * gnd_unit_dist2cm_scale )
#define gnd_m2dist(x)			( (x) * gnd_unit_m2dist_scale )
#define gnd_dist2m(x)			( (x) * gnd_unit_dist2m_scale )
// <--- distance unit

// ---> angle unit
#define gnd_unit_rad2ang_scale	( 1.0 )
#define gnd_unit_ang2rad_scale	( 1.0 )
#define gnd_unit_deg2ang_scale	( M_PI / 180.0 )
#define gnd_unit_ang2deg_scale	( 180.0 / M_PI )
#define gnd_rad2ang(x)			( (x) * gnd_unit_rad2ang_scale )
#define gnd_ang2rad(x)			( (x) * gnd_unit_ang2rad_scale )
#define gnd_deg2ang(x)			( (x) * gnd_unit_deg2ang_scale )
#define gnd_ang2deg(x)			( (x) * gnd_unit_ang2deg_scale )

#define gnd_unit_rad2deg_scale	( 180.0 / M_PI )
#define gnd_unit_deg2rad_scale	( M_PI / 180.0 )
#define gnd_rad2deg(x)			( (x) * gnd_unit_rad2deg_scale )
#define gnd_deg2rad(x)			( (x) * gnd_unit_deg2rad_scale )
// <--- angle unit

// ---> time unit
#define gnd_unit_sec2time_scale			(1.0)
#define gnd_unit_time2sec_scale			(1.0)
#define gnd_unit_msec2time_scale		(1.0e-3)
#define gnd_unit_time2msec_scale		(1.0e+3)
#define gnd_unit_usec2time_scale		(1.0e-6)
#define gnd_unit_time2usec_scale		(1.0e+6)
#define gnd_unit_nsec2time_scale		(1.0e-9)
#define gnd_unit_time2nsec_scale		(1.0e+9)

#define gnd_sec2time(x)			((x) * gnd_unit_sec2time_scale )
#define gnd_time2sec(x)			((x) * gnd_unit_time2sec_scale )
#define gnd_msec2time(x)		((x) * gnd_unit_msec2time_scale )
#define gnd_time2msec(x)		((x) * gnd_unit_time2msec_scale )
#define gnd_usec2time(x)		((x) * gnd_unit_usec2time_scale )
#define gnd_time2usec(x)		((x) * gnd_unit_time2usec_scale )
#define gnd_nsec2time(x)		((x) * gnd_unit_nsec2time_scale )
#define gnd_time2nsec(x)		((x) * gnd_unit_time2nsec_scale )

#define gnd_unit_sec2msec_scale		(1.0e+3)
#define gnd_unit_sec2usec_scale		(1.0e+6)
#define gnd_unit_sec2nsec_scale		(1.0e+9)
#define gnd_unit_msec2sec_scale		(1.0e-3)
#define gnd_unit_msec2usec_scale	(1.0e+3)
#define gnd_unit_msec2nsec_scale	(1.0e+6)
#define gnd_unit_usec2sec_scale		(1.0e-6)
#define gnd_unit_usec2msec_scale	(1.0e-3)
#define gnd_unit_usec2nsec_scale	(1.0e+3)
#define gnd_unit_nsec2sec_scale		(1.0e-9)
#define gnd_unit_nsec2msec_scale	(1.0e-6)
#define gnd_unit_nsec2usec_scale	(1.0e-3)

#define gnd_sec2msec(x)			((x)*gnd_unit_sec2msec_scale)
#define gnd_sec2usec(x)			((x)*gnd_unit_sec2usec_scale)
#define gnd_sec2nsec(x)			((x)*gnd_unit_sec2nsec_scale)
#define gnd_msec2sec(x)			((x)*gnd_unit_msec2sec_scale)
#define gnd_msec2usec(x)		((x)*gnd_unit_msec2usec_scale)
#define gnd_msec2nsec(x)		((x)*gnd_unit_msec2nsec_scale)
#define gnd_usec2sec(x)			((x)*gnd_unit_usec2sec_scale)
#define gnd_usec2msec(x)		((x)*gnd_unit_usec2msec_scale)
#define gnd_usec2nsec(x)		((x)*gnd_unit_usec2nsec_scale)
#define gnd_nsec2sec(x)			((x)*gnd_unit_nsec2sec_scale)
#define gnd_nsec2msec(x)		((x)*gnd_unit_nsec2msec_scale)
#define gnd_nsec2usec(x)		((x)*gnd_unit_nsec2usec_scale)
// <--- time unit

#define gnd_loop_period(cur, st, cyc) (  floor( (cur - st) / cyc ) )
#define gnd_loop_next(cur, st, cyc)  (  st + (gnd_loop_period(cur, st, cyc) + 1) * cyc )

/**
 * @ingroup GNDUtil
 * @def
 * @brief normalization of angle
 * @return -pi <= ret < pi
 */
double rad_normalize(double x)
{
    return ( x - ( ( qFloor((x + M_PI ) / (2 * M_PI) ) ) * (2 * M_PI)));
}


template<typename T>
class  data_buff
{
private:
    QVector<T> msgs_;   //< buffer
    uint32_t length_;   //< length of buffer
    uint32_t header_;   //< header index of ring buffer(not ros header)
    uint32_t ndata_;    //< number of valid data
    uint64_t n_;        //< latest data's sequential number in this reader (not ros header)

public:
    data_buff();
    data_buff(uint32_t maxsize);
    ~data_buff();

public:
    int16_t push(const T &msg);
    int16_t copy_at_time(T *dest, double query);
    int nlatest();

};

/**
 * @brief construct class without init size
 * @param [in]  : -
 * @return     : -
 */
template<typename T>
data_buff<T>::data_buff()
{
    length_ = 0;
    header_ = 0;
    ndata_ = 0;
    n_ = 0;
}

/**
 * @brief construct class with size
 * @param [in] maxsize : buff size
 * @return     : -
 */
template<typename T>
data_buff<T>::data_buff(uint32_t maxsize)
{
    msgs_.resize(maxsize);
    length_ = maxsize;
    header_ = 0;
    ndata_ = 0;
    n_ = 0;
}

template<typename T>
data_buff<T>::~data_buff()
{

}

/**
 * @brief Save data to buffer
 * @param [in] msg : message for saving
 * @return    <0 : fail
 *           >=0 : success
 */
template<typename T>
int16_t data_buff<T>::push(const T &msg)
{
    int i;
    Q_ASSERT_X( msgs_.size() > 0, "valid" , "null buffer" );
    i = (header_ + 1) % length_;
    msgs_[i] = msg;
    header_ = i;
    ndata_ = (ndata_ < length_ - 1) ? (ndata_ + 1) : (length_ - 1);
    n_++;
    return 0;

}

/**
 * \brief get sequential serial number of latest
 */
template<typename T>
int data_buff<T>::nlatest()
{
    return n_;
}

/**
 * @brief get data from buffer by required time
 * @param [in] query : data message based time
 * @param [out] dest : searched message data
 * @return    <0 : fail
 *           >=0 : success
 */
template<typename T>
int16_t data_buff<T>::copy_at_time(T *dest, double query)
{
    uint32_t h = header_;

    double query_sec = query;
    double sec = msgs_[h].header.stamp.toSec();
    uint32_t i, j;

    // ---> seek scan of nearest query time
    if( sec < query_sec || ndata_ <= 1  )
    {
        // query time is after last scan timestamp
        i = h;
    }
    else
    {
        double diff1, diff2;
        i = h;
        for( j = 1; (j < length_ - 1) && (j < ndata_); j++ )
        {
            i = (h - j + length_) % length_;
            sec = msgs_[i].header.stamp.toSec();
            if( sec < query_sec )
            {
                break;
            }
        }
        diff1 = qFabs(query_sec - msgs_[i].header.stamp.toSec());
        i = (h - (j - 1) + length_) % length_;
        diff2 = qFabs(msgs_[i].header.stamp.toSec() - query_sec);
        i = (diff1 < diff2) ? ((h - j + length_ ) % length_) : ((h - (j - 1) + length_) % length_);
    }
    // copy
    *dest = msgs_[i];

    return 0;

}

} //End of namespace

#endif // GND_ROSUTIL_H
