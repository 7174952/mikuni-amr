/*
 * gnd_gridmap.hpp
 *
 *  Created on: 2011/08/09
 *      Author: tyamada
 *
 *  Updated by ryu, 2023/5/9
 *  .Use Qt v5.14 lib to update bitmap class
 */

#ifndef GND_GRIDMAP_H
#define GND_GRIDMAP_H

#include <QPoint>
#include <QVector>
#include <QDataStream>
#include <QFile>
#include <QGenericMatrix>
#include <QtMath>

namespace gnd {

static const char __GridMapFileTag__[] = "GGRD";
static const char __GridPlaneFileTag__[] = "GPLN";
static uint __GridMapTagSize__ = sizeof(__GridMapFileTag__);
static uint __GridPlaneTagSize = sizeof(__GridPlaneFileTag__);

/**
 * @brief statistics counting map
 */
struct counting_map_pixel
{
    /// @brief sum of reflection point position value
    QPointF pos_sum;
    /// @brief sum of covariance
    QGenericMatrix<2,2,double> cov_sum;
    /// @brief count of reflection point
    quint64 cnt;
    /// @brief constructor
    counting_map_pixel() : cnt(0) {}

};

typedef struct counting_map_pixel cmap_pixel_t;

/**
 * @brief pixel of laser scan statistics map
 */
struct lssmap_pixel
{
    /// @brief mean
    QPointF mean;
    /// @brief inverse matrix of covariance
    QGenericMatrix<2,2,double> inv_cov;
    /// @brief weight ( N / sqrt(|Sigma|)  ) * eta
    double k;
    /// @brief constructor
    lssmap_pixel() : k(0.0) {}
};

typedef struct lssmap_pixel lssmap_pixel_t;

template <typename T>
class basic_gridmap
{
private:

public:
    basic_gridmap();
    ~basic_gridmap();

protected:
    /// @brief map data storage
    QVector<QVector<T> > _map_data;
    /// @brief number of memory unit
    QPoint _unit;
    /// @brief number of plane
    QPoint _plane;

public:
    virtual int allocate(const uint64_t r, const uint64_t c);
    virtual int deallocate();
    virtual bool is_allocate();
    virtual int reallocate(const int64_t sr, const int64_t sc);

    uint64_t row() const;
    uint64_t column() const;

    T* pointer(const ulong r, const ulong c);
    uint32_t _unit_row_();
    uint32_t _unit_column_();

    uint32_t _plane_row_();
    uint32_t _plane_column_();
    int set_uniform(const T v);
    int set_uniform(const T *v);
    int get(const ulong r, const ulong c, T* v);
    int set(const ulong r, const ulong c, const T v);
    int set(const ulong r, const ulong c, const T* v);


private:
    uint32_t memory_unit_row(ulong r);
    uint32_t memory_unit_column(ulong c);

protected:
    int _fwrite(QDataStream& out, const lssmap_pixel_t &pixel);
    int _fwrite(QDataStream& out, const uint8_t &pixel);
    int _fwrite(QDataStream& out, const uint32_t &pixel);
    int _fwrite(QDataStream& out, const cmap_pixel_t &pixel);
    int _fwrite(QDataStream& out, const double &pixel);

    int _fread(QDataStream& in, lssmap_pixel_t *pixel);
    int _fread(QDataStream& in, uint8_t *pixel);
    int _fread(QDataStream& in, uint32_t *pixel);
    int _fread(QDataStream& in, cmap_pixel_t *pixel);
    int _fread(QDataStream& in, double *pixel);


    // ---> read write
public:
    virtual int fwrite(const QString fname);
    virtual int fread(const QString fname);


    // <--- read write

};

// ----------------------------------------> constructor, destructor
template <typename T>
basic_gridmap<T>::basic_gridmap()
{
    _unit.setX(0);
    _unit.setY(0);
    _plane.setX(0);
    _plane.setY(0);
    _map_data.clear();
}

template <typename T>
basic_gridmap<T>::~basic_gridmap()
{
    if(is_allocate()) deallocate();
}

/**
 * @brief allocate memory
 * @param[in] r : row size
 * @param[in] c : column size
 */
template <typename T>
int basic_gridmap<T>::allocate(const uint64_t r, const uint64_t c)
{
    if(is_allocate())	return -1;

    _map_data.resize(r);
    if(_map_data.size() != r)	return -1;
    _plane.setY(r);

    for(uint i = 0; i < _map_data.size(); i++)
    {
        _map_data[i].resize(c);
        if(_map_data[i].size() != c)
        {
            deallocate();
            return -1;
        }
    }
    _plane.setX(c);

    _unit.setY(1);
    _unit.setX(1);

    return 0;
}

/**
 * @brief return memory allocated or not
 */
template <typename T>
bool basic_gridmap<T>::is_allocate()
{
    return (_map_data.size() && _unit.x() && _unit.y());
}

/**
 * @brief deallocate memory
 */
template< typename T >
int basic_gridmap<T>::deallocate()
{
    if(!is_allocate())	return -1;

    _map_data.clear();
    _unit.setY(0);
    _unit.setX(0);
    _plane.setY(0);
    _plane.setX(0);

    return 0;
}

/**
 * @brief number of pixel (row)
 */
template< typename T >
uint64_t basic_gridmap<T>::row() const
{
    return _plane.y() * _unit.y();
}

/**
 * @brief number of pixel (column)
 */
template< typename T >
uint64_t basic_gridmap<T>::column() const
{
    return (uint64_t)_plane.x() * _unit.x();
}

/**
 * @brief reallocate memory
 * @param[in] sr : required index of grid (row)
 * @param[in] sc : required index of grid (column)
 */
template< typename T >
int basic_gridmap<T>::reallocate(const int64_t sr, const int64_t sc)
{
    if(((sr >= 0) && (sr < row())) && ((sc >= 0) && (sc < column()))) return -1; //no need more memorys

    int64_t npr = 0, npc = 0;
    uint i,j;

    //need new more rows
    if(sr < 0)
    {
        npr = qAbs(sr);
        for(i = 0; i < npr; i++)
        {
            _map_data.prepend(QVector<T>(column()));
        }
    }
    else if(sr >= row())
    {
        npr = sr - row() + 1;
        for(i = 0; i < npr; i++)
        {
            _map_data.append(QVector<T>(column()));
        }
    }

    //need new more columns
    if(sc < 0)
    {
        npc = qAbs(sc);
        for(i = 0; i < _map_data.size(); i++)
        {
            for(j = 0; j < npc;j++)
            {
                _map_data[i].prepend(T());
            }
        }
    }
    else if(sc >= column())
    {
        npc = sc - column() + 1;
        for(i = 0; i < _map_data.size(); i++)
        {
            _map_data[i].append(QVector<T>(npc));
        }
    }

    //set new size
    _plane.setY(npr + row());
    _plane.setX(npc + column());

    return 0;
}

/**
 * @brief pointer of a pixel
 * @param[in] r : pixel index (row)
 * @param[in] c : pixel index (column)
 */
template< typename T >
T* basic_gridmap<T>::pointer(const ulong r, const ulong c)
{
    if(!is_allocate())				return 0;
    if(r > row() || c > column())	return 0;

    {
      uint32_t pr, pc;
      pr = memory_unit_row(r);
      pc = memory_unit_column(c);

      return (&_map_data[pr][pc]);
    }
}

/**
 * @brief row index of memory unit
 * @param[in] r : pixel row index
 */
template< typename T >
uint32_t basic_gridmap<T>::memory_unit_row(ulong r)
{
    return r / _unit.y();
}

/**
 * @brief column index of memory unit
 * @param[in] c : pixel column index
 */
template< typename T >
uint32_t basic_gridmap<T>::memory_unit_column(ulong c)
{
    return c / _unit.x();
}

/**
 * @brief memory unit size (row)
 */
template< typename T >
uint32_t basic_gridmap<T>::_unit_row_()
{
    return _unit.y();
}

/**
 * @brief memory unit size (row)
 */
template< typename T >
uint32_t basic_gridmap<T>::_unit_column_()
{
    return _unit.x();
}

/**
 * @brief number of  memory unit (row)
 */
template< typename T >
uint32_t basic_gridmap<T>::_plane_row_()
{
    return _plane.y();
}

/**
 * @brief number of  memory unit (column)
 */
template< typename T >
uint32_t basic_gridmap<T>::_plane_column_()
{
    return _plane.x();
}

/**
 * @brief set all pixel value uniformly
 * @param[in] v : value
 */
template< typename T >
int basic_gridmap<T>::set_uniform(const T v)
{
    return set_uniform(&v);
}

/**
 * @brief set all pixel value uniformly
 * @param[in] v : value
 */
template< typename T >
int basic_gridmap<T>::set_uniform(const T *v)
{
    if(!v)	return -1;
    if(!is_allocate())	return -1;

    ulong i, j;

    // ---> plane
    for(i=0;i<_map_data.size();i++)
    {
        for(j=0; j < _map_data[i].size(); j++)
        {
            _map_data[i][j] = *v;
        }
    }


    return 0;
}

/**
 * @brief get value in a pixel
 * @param[in]  r : pixel's index (row)
 * @param[in]  c : pixel's index (column)
 * @param[out] v : pixel's value
 */
template< typename T >
int basic_gridmap<T>::get(const ulong r, const ulong c, T* v)
{
    if(!v)	return -1;
    *v = *pointer(r,c);
    return 0;
}

/**
 * @brief set value in a pixel
 * @param[in]  r : pixel's index (row)
 * @param[in]  c : pixel's index (column)
 * @param[in] v : pixel's value
 */
template< typename T >
int basic_gridmap<T>::set(const ulong r, const ulong c, const T v)
{
    return set(r,c,&v);
}

/**
 * @brief set value in a pixel
 * @param[in]  r : pixel's index (row)
 * @param[in]  c : pixel's index (column)
 * @param[in] v : pixel's value
 */
template< typename T >
int basic_gridmap<T>::set(const ulong r, const ulong c, const T* v)
{
    if(!v)	return -1;
    *pointer(r,c) = *v;
    return 0;
}

/**
 * @brief file write (binary)
 * @param[in] fname : file path
 */
template< typename T >
int basic_gridmap<T>::fwrite(const QString fname)
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
    out.writeRawData(&__GridMapFileTag__[0],__GridMapTagSize__);
    // file header
    out << _unit.y() << _unit.x() << _plane.y() << _plane.x();

    uint64_t i, j;
    for(i = 0; i < _map_data.size(); i++)
    {
        for(j = 0; j < _map_data[i].size(); j++)
        {
            _fwrite(out,_map_data[i][j]);
        }
    }

    file.close();

    return 0;
}

template< typename T >
int basic_gridmap<T>::_fwrite(QDataStream& out, const lssmap_pixel_t &pixel)
{
    out << pixel.mean.y() << pixel.mean.x()
        << pixel.inv_cov(0,0) << pixel.inv_cov(0,1) << pixel.inv_cov(1,0) << pixel.inv_cov(1,1)
        << pixel.k;
    return 0;
}

template< typename T >
int basic_gridmap<T>::_fwrite(QDataStream& out, const uint8_t &pixel)
{
    out << pixel;
    return 0;
}

template< typename T >
int basic_gridmap<T>::_fwrite(QDataStream& out, const uint32_t &pixel)
{
    out << pixel;
    return 0;
}

template< typename T >
int basic_gridmap<T>::_fwrite(QDataStream& out, const double &pixel)
{
    out << pixel;
    return 0;
}


template< typename T >
int basic_gridmap<T>::_fwrite(QDataStream& out, const cmap_pixel_t &pixel)
{
    out << pixel.pos_sum.y() << pixel.pos_sum.x()
        << pixel.cov_sum(0,0) << pixel.cov_sum(0,1) << pixel.cov_sum(1,0) << pixel.cov_sum(1,1)
        << pixel.cnt ;

    return 0;
}

/**
 * @brief file read (binary)
 * @param[in] fname : file path
 */
template< typename T >
int basic_gridmap<T>::fread(const QString fname)
{
    QFile file(fname);
    if(!file.exists())
    {
        qDebug() << "File:" << fname << "not exist!";
        return -1;
    }

    file.open(QIODevice::ReadOnly);
    QDataStream in(&file);

    char _fileFlag[__GridMapTagSize__] = "";

    //Check file tag
    in.readRawData(_fileFlag, __GridMapTagSize__);
    if(QString(_fileFlag) != QString(__GridMapFileTag__))
    {
        qDebug() << "File tag marker is error!";
        return -1;
    }

    //Read map size
    in >> _unit.ry() >> _unit.rx() >> _plane.ry() >> _plane.rx();

    //Create data buffer
    if(allocate(_plane.y()*_unit.y(), _plane.x()*_unit.x()) < 0)
    {
        qDebug() << "Allocate memory failer!";
        return -1;
    }

    T pixel;

    for(uint64_t i = 0; i < _plane.y()*_unit.y();i++)
    {
        for(uint64_t j = 0; j < _plane.x()*_unit.x();j++)
        {
          //read pixel data from file
          if(_fread(in,&pixel) < 0)
          {
              qDebug() << "Read:" << fname << "file failed!";
              deallocate();
              file.close();
              return -1;
          }
          _map_data[i][j] = pixel;
        }
    }

    file.close();

    return 0;
}

template< typename T >
int basic_gridmap<T>::_fread(QDataStream& in, lssmap_pixel_t *pixel)
{
    if(!in.atEnd()) in >> pixel->mean.ry();
    else return -1;
    if(!in.atEnd()) in >> pixel->mean.rx();
    else return -1;
    if(!in.atEnd()) in >> pixel->inv_cov(0,0);
    else return -1;
    if(!in.atEnd()) in >> pixel->inv_cov(0,1);
    else return -1;
    if(!in.atEnd()) in >> pixel->inv_cov(1,0);
    else return -1;
    if(!in.atEnd()) in >> pixel->inv_cov(1,1);
    else return -1;
    if(!in.atEnd()) in >> pixel->k;
    else return -1;

    return 0;
}

template< typename T >
int basic_gridmap<T>::_fread(QDataStream& in, uint8_t *pixel)
{
    if(!in.atEnd()) in >> *pixel;
    else return -1;

    return 0;
}

template< typename T >
int basic_gridmap<T>::_fread(QDataStream& in, uint32_t *pixel)
{
    if(!in.atEnd()) in >> *pixel;
    else return -1;

    return 0;
}

template< typename T >
int basic_gridmap<T>::_fread(QDataStream& in, double *pixel)
{
    if(!in.atEnd()) in >> *pixel;
    else return -1;

    return 0;
}

template< typename T >
int basic_gridmap<T>::_fread(QDataStream& in, cmap_pixel_t *pixel)
{
    if(!in.atEnd()) in >> pixel->pos_sum.ry();
    else return -1;

    if(!in.atEnd()) in >> pixel->pos_sum.rx();
    else return -1;

    if(!in.atEnd()) in >> pixel->cov_sum(0,0);
    else return -1;

    if(!in.atEnd()) in >> pixel->cov_sum(0,1);
    else return -1;

    if(!in.atEnd()) in >> pixel->cov_sum(1,0);
    else return -1;

    if(!in.atEnd()) in >> pixel->cov_sum(1,1);
    else return -1;

    if(!in.atEnd()) in >> pixel->cnt;
    else return -1;

    return 0;
}

} //end namespace

#endif // GND_GRIDMAP_H
