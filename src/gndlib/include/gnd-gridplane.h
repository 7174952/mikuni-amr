/*
 * gnd_gridmap.hpp
 *
 *  Created on: 2011/08/09
 *      Author: tyamada
 *
 *  Updated by ryu, 2023/5/15
 *  .Use Qt v5.14 lib to update grid plane class
 */
#ifndef GND_GRIDPLANE_H
#define GND_GRIDPLANE_H

#include <QPoint>
#include <QVector>
#include <QDataStream>
#include <QFile>
#include <QGenericMatrix>
#include <QtMath>

#include "gnd-gridmap.h"

namespace gnd {

template <typename T>
class gridplane : public basic_gridmap<T>
{
public:
    // ---> constructor, destructor
      public:
    // constructor
    gridplane();
    // destructor
    ~gridplane();
    // <--- constructor, destructor

    // ---> variables
protected:
    /// @brief origin(row index 0, column index 0)
    QPointF _orgn;
    /// @brief resolution
    QPointF _rsl;
    // <--- variables

public:
    // set core position
    int pset_core(const double x, const double y);
    // set origin position
    int pset_origin(const double x, const double y);
    // get origin position
    int pget_origin(double *x, double *y);
    // set resolution
    int pset_rsl(double x, double y);

    // ---> allocate, deallocate
public:
    // allocate
    virtual int pallocate(const double xs, const double ys, const double xrsl, const double yrsl);
    // deallocate
    virtual int deallocate();
    // reallocate
    virtual int reallocate(const double xs, const double ys);
    // <--- allocate, deallocate


    // ---> setter, getter
public:
    // get pixel index
    int pindex(const double x, const double y, qint64 *r, qint64 *c);
    // get pixel pointer
    T* ppointer(const double x, const double y);
    // get pixel value
    T pvalue(const double x, const double y);
    // x lower bounds on current plane
    double xlower();
    // x upper bounds on current plane
    double xupper();
    // y lower bounds on current plane
    double ylower();
    // y upper bounds on current plane
    double yupper();
    // get a pixel value
    int pget(const double x, const double y, T* v);
    // set a pixel value
    int pset(const double x, const double y, const T &v);
    // get x component resolution
    double xrsl();
    // get y component resolution
    double yrsl();
    // get origin's x component value
    double xorg();
    // get origin's y component value
    double yorg();
    // get plane width
    double width();
    // get plane height
    double height();

    // get a pixel lower boundary position
    int pget_pos_lower(const quint64 r, const quint64 c, double *x, double *y);
    // get a pixel upper boundary position
    int pget_pos_upper(const quint64 r, const quint64 c, double *x, double *y);
    // get a pixel core position
    int pget_pos_core(const quint64 r, const quint64 c, double *x, double *y);
    // <--- setter, getter

public:
    // ---> read write
    public:
    // file write(binary)
    int fwrite(const QString fname);
    // file read(binary)
    int fread(const QString fname);

    // <--- read write

};

// ----------------------------------------> constructor, destructor
/**
 * @brief constructor
 */
template< typename T >
gridplane<T>::gridplane() : basic_gridmap<T>()
{

}

/**
 * @brief destructor
 */
template< typename T >
gridplane<T>::~gridplane()
{

}
// <---------------------------------------- constructor, destructor

// ----------------------------------------> allocator, deallocator
/**
 * @brief allocate
 * @param   xs : x component size
 * @param   ys : y component size
 * @param xrsl : x component resolution
 * @param yrsl : y component resolution
 */
template< typename T >
int gridplane<T>::pallocate(const double xs, const double ys, const double xrsl, const double yrsl)
{
    unsigned long r, c;
    int ret;

    if((xrsl <= 0) || (yrsl <= 0))	return -1;

    // compute size of row and column
    r = ::qCeil(ys / yrsl);
    c = ::qCeil(xs / xrsl);
    // allocate
    if( (ret = basic_gridmap<T>::allocate(r, c)) < 0 )	return ret;
    // set resolution
    _rsl.rx() = xrsl;
    _rsl.ry() = yrsl;
    return ret;
}


/**
 * @brief deallocate
 */
template< typename T >
int gridplane<T>::deallocate()
{
    int ret;

    if( (ret = basic_gridmap<T>::deallocate()) < 0 ) return ret;

    _orgn.rx() = 0;
    _orgn.ry() = 0;
    _rsl.rx() = 0;
    _rsl.ry() = 0;

    return ret;
}

/**
 * @brief deallocate
 * @param[in] xs : requiring x component size
 * @param[in] ys : requiring y component size
 */
template< typename T >
int gridplane<T>::reallocate(const double xs, const double ys)
{
    qint64 r = 0, c = 0;
    int ret;
    int tmp;

    // compute access index of row and column
    if( (tmp = pindex(xs, ys, &r, &c)) == 0 )	return 0;

    // reallocate
    if( (ret = basic_gridmap<T>::reallocate(r,c)) < 0 ) return ret;

    // adjust origin
    if( c < 0 ){
      _orgn.rx() -= _rsl.x() * ::qCeil( qAbs((double)c) / basic_gridmap<T>::_unit.x() ) * basic_gridmap<T>::_unit.x();
    }
    if( r < 0 ){
      _orgn.ry() -= _rsl.y() * ::qCeil( qAbs((double)r) / basic_gridmap<T>::_unit.y() ) * basic_gridmap<T>::_unit.y();
    }

    // <---------------------------------------- allocator, deallocator

    return ret;
}

/**
 * @brief get pixel index
 * @param[in]  x : x component value
 * @param[in]  y : y component value
 * @param[out] r : row index
 * @param[out] c : column index
 */
template< typename T >
int gridplane<T>::pindex(const double x, const double y, qint64 *r, qint64 *c)
{
    if( !basic_gridmap<T>::is_allocate() ) return -1;

    double xx = x - _orgn.x();
    double yy = y - _orgn.y();
    *r = qFloor( yy / _rsl.y() );
    *c = qFloor( xx / _rsl.x() );
//    *r = qCeil( yy / _rsl.y() );
//    *c = qCeil( xx / _rsl.x() );

    return ((*r >= 0) && (*r < (signed)basic_gridmap<T>::row()) &&
            (*c >= 0) && (*c < (signed)basic_gridmap<T>::column()) ) ? 0 : -1;
}

// ----------------------------------------> setter, getter
/**
 * @brief set core position
 * @param[in] x : x component value
 * @param[in] y : y component value
 */
template< typename T >
int gridplane<T>::pset_core(const double x, const double y)
{
    _orgn.rx() = x - (basic_gridmap<T>::column() * _rsl.x()) / 2.0;
    _orgn.ry() = y - (basic_gridmap<T>::row() * _rsl.y()) / 2.0;
    return 0;
}


/**
 * @brief set origin position
 * @param[in] x : x component value
 * @param[in] y : y component value
 */
template< typename T >
int gridplane<T>::pset_origin(const double x, const double y)
{
    _orgn.rx() = x;
    _orgn.ry() = y;
    return 0;
}


/**
 * @brief get core position
 * @param[out] x : x component value
 * @param[out] y : y component value
 */
template< typename T >
int gridplane<T>::pget_origin(double *x, double *y)
{
    *x = _orgn.x();
    *y = _orgn.y();
    return 0;
}

/**
 * @brief get pointer
 * @param[in]  x : x component value
 * @param[in]  y : y component value
 * @return 0 : not exit
 */
template< typename T >
T* gridplane<T>::ppointer(const double x, const double y)
{
    qint64 r, c;

    if( pindex(x, y, &r, &c) < 0 ) return 0;

    return basic_gridmap<T>::pointer(r, c);
}

/**
 * @brief get value
 * @param[in]  x : x component value
 * @param[in]  y : y component value
 * @return pixel's value
 */
template< typename T >
T gridplane<T>::pvalue(const double x, const double y)
{
    return *ppointer(x, y);
}

/**
 * @brief x lower bound
 */
template< typename T >
double gridplane<T>::xlower()
{
    return _orgn.x();
}

/**
 * @brief x upper bound
 */
template< typename T >
double gridplane<T>::xupper()
{
    return _orgn.x() + basic_gridmap<T>::column() * _rsl.x();
}

/**
 * @brief y lower bound
 */
template< typename T >
double gridplane<T>::ylower()
{
    return _orgn.y();
}

/**
 * @brief y upper bound
 */
template< typename T >
double gridplane<T>::yupper()
{
    return _orgn.y() + basic_gridmap<T>::row() * _rsl.y();
}

/**
 * @brief get value
 * @param[in]  x : x component value
 * @param[in]  y : y component value
 * @param[out] v : pixel's value
 */
template< typename T >
int gridplane<T>::pget(const double x, const double y, T* v)
{
    qint64 r, c;

    if(!v)	return -1;
    if( pindex(x, y, &r, &c) < 0 ) return -1;
    *v = *basic_gridmap<T>::pointer(r, c);
    return 0;
}

/**
 * @brief set value
 * @param[in] x : x component value
 * @param[in] y : y component value
 * @param[in] v : pixel's value
 */
template< typename T >
int gridplane<T>::pset(const double x, const double y, const T &v)
{
    long r = 0, c = 0;
    if(!v)	return -1;
    for( int ret = pindex(x, y, &r, &c); ret < 0; ret = pindex(x, y, &r, &c))
    {
        int ret_realloc;
        if( (ret_realloc = reallocate(x,y)) < 0) return -1;
    }
    *(basic_gridmap<T>::pointer(r, c)) = v;
    return 0;
}

/**
 * @brief x resolution
 */
template< typename T >
double gridplane<T>::xrsl()
{
    return _rsl.x();
}

/**
 * @brief y resolution
 */
template< typename T >
double gridplane<T>::yrsl()
{
    return _rsl.y();
}


/**
 * @brief origin's x component vlaue
 */
template< typename T >
double gridplane<T>::xorg()
{
    return _orgn.x();
}


/**
 * @brief origin's y component vlaue
 */
template< typename T >
double gridplane<T>::yorg()
{
    return _orgn.y();
}


/**
 * @brief plane widith
 */
template< typename T >
double gridplane<T>::width()
{
    return _rsl.x() * basic_gridmap<T>::column();
}


/**
 * @brief plane height
 */
template< typename T >
double gridplane<T>::height()
{
    return _rsl.y() * basic_gridmap<T>::row();
}


/**
 * @brief set resolution
 */
template< typename T >
int gridplane<T>::pset_rsl(double x, double y)
{
    _rsl.rx() = x;
    _rsl.ry() = y;
    return 0;
}


/**
 * @brief get a pixel lower bounds
 */
template< typename T >
int gridplane<T>::pget_pos_lower(const quint64 r, const quint64 c, double *x, double *y)
{
    *x = _orgn.x() + c * _rsl.x();
    *y = _orgn.y() + r * _rsl.y();
    return 0;
}


/**
 * @brief get a pixel upper bounds
 */
template< typename T >
int gridplane<T>::pget_pos_upper(const quint64 r, const quint64 c, double *x, double *y)
{
    pget_pos_lower(r, c, x, y);
    *x += _rsl.x();
    *y += _rsl.y();
    return 0;
}


/**
 * @brief get a pixel core position
 */
template< typename T >
int gridplane<T>::pget_pos_core(const quint64 r, const quint64 c, double *x, double *y)
{
    pget_pos_lower(r, c, x, y);
    *x += _rsl.x() / 2.0;
    *y += _rsl.y() / 2.0;
    return 0;
}

// <---------------------------------------- setter, getter

// ---> read write
/**
 * @brief file write
 * @param[in] fname : file name
 */
template< typename T >
int gridplane<T>::fwrite(const QString fname)
{
    QFile file(fname);
    if(file.exists())
    {
        file.open(QIODevice::WriteOnly);
    }
    else
    {
        file.open(QIODevice::NewOnly);
    }

    QDataStream out(&file);
    // save file tag
    out.writeRawData(&__GridPlaneFileTag__[0],__GridPlaneTagSize);
    // file header
    out << basic_gridmap<T>::_unit.y() << basic_gridmap<T>::_unit.x() << basic_gridmap<T>::_plane.y() << basic_gridmap<T>::_plane.x();
    out << _orgn.y() << _orgn.x() << _rsl.y() << _rsl.x();

    uint64_t r, c;
    for(r = 0; r < basic_gridmap<T>::_map_data.size(); r++)
    {
        for(c = 0; c < basic_gridmap<T>::_map_data[r].size(); c++)
        {
            basic_gridmap<T>::_fwrite(out,basic_gridmap<T>::_map_data[r][c]);
        }
    }

    file.close();

    return 0;
}

/**
 * @brief file read
 * @param[in] fname : file name
 */
template< typename T >
int gridplane<T>::fread(const QString fname)
{
    QFile file(fname);
    if(!file.exists())
    {
        qDebug() << "File:" << fname << "not exist!";
        return -1;
    }

    file.open(QIODevice::ReadOnly);
    QDataStream in(&file);

    char _fileFlag[__GridPlaneTagSize] = "";

    //Check file tag
    in.readRawData(_fileFlag, __GridPlaneTagSize);
    if(QString(_fileFlag) != QString(__GridPlaneFileTag__))
    {
        qDebug() << "File tag marker is error!";
        file.close();
        return -1;
    }

    //Read map size
    in >> basic_gridmap<T>::_unit.ry() >> basic_gridmap<T>::_unit.rx() >> basic_gridmap<T>::_plane.ry() >> basic_gridmap<T>::_plane.rx();
    in >> _orgn.ry() >> _orgn.rx() >> _rsl.ry() >> _rsl.rx();

    //Create data buffer
    if(basic_gridmap<T>::allocate(basic_gridmap<T>::_plane.y()*basic_gridmap<T>::_unit.y(), basic_gridmap<T>::_plane.x()*basic_gridmap<T>::_unit.x()) < 0)
    {
        qDebug() << "Allocate memory failer!";
        file.close();
        return -1;
    }

    T pixel;

    for(uint64_t r = 0; r < basic_gridmap<T>::_plane.y()*basic_gridmap<T>::_unit.y();r++)
    {
        for(uint64_t c = 0; c < basic_gridmap<T>::_plane.x()*basic_gridmap<T>::_unit.x();c++)
        {
          //read pixel data from file
          if(basic_gridmap<T>::_fread(in,&pixel) < 0)
          {
              qDebug() << "Read:" << fname << "file failed!";
              deallocate();
              file.close();
              return -1;
          }
          basic_gridmap<T>::_map_data[r][c] = pixel;
        }
    }

    file.close();
    return 0;

}



} //end namespace

#endif // GND_GRIDPLANE_H
