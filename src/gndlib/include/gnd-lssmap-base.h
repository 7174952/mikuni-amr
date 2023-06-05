/*
 * gnd-lssmap-base.hpp (GND's Laser Scan Statistics MAP BASE)
 *
 *  Created on: 2014/09/09
 *      Author: Taichi Yamada
 *  Updated by ryu, 2023/5/16
 *  .Use Qt v5.14 lib to update map functions
 */

#ifndef GND_LSSMAP_BASE_H
#define GND_LSSMAP_BASE_H

#include <QVector>
#include <QtMath>
#include <QTransform>
#include <QString>
#include <QGenericMatrix>
#include "gnd-gridmap.h"
#include "gnd-gridplane.h"
#include "float.h"

namespace gnd
{

/**
 * @brief number of map data plane
 */
static const size_t PLANE_NUM = 4;

static const double SqrtDBL_EPSILON = qSqrt(DBL_EPSILON);
static const double ErrorMargin     = SqrtDBL_EPSILON;
/**
 * @brief default file out directory
 */
static const QString CMapDirectoryDefault(".");

/**
 * @brief counting map file name format
 */

/**
 * @brief default counting map file name
 */
static const QString CMapFileNameDefault("lssmap");

/**
 * @brief default counting map file extension
 */
static const QString CMapFileExtension("cmap");

/**
 * @brief default map data cell size
 */
static const double DefaultMapCellSize = 5;

/**
 * @brief additional smoothing parameter
 */
static const uint32_t AddSmooth_SampleNum = 3;

const double AddSmooth_SumCovXX = 2.0/3.0;
const double AddSmooth_SumCovXY = 0.0;
const double AddSmooth_SumCovYX = 0.0;
const double AddSmooth_SumCovYY = 2.0/3.0;
const double AddSmooth_Eta      = 0.831220;

/**
 * @ingroup GNDLSSMAP
 * @brief counting map
 */
struct counting_map_
{
    /// @brief four planes
    gridplane<cmap_pixel_t> plane[PLANE_NUM];
};

/**
 * @ingroup GNDLSSMAP
 * @typedef cmap_t
 * @see counting_map
 */
typedef struct counting_map_ cmap_t;

/**
 * @ingroup GNDLSSMAP
 * @brief map
 */
struct lssmap_
{
    /// @brief four planes
    gridplane<lssmap_pixel_t> plane[PLANE_NUM];
};

/**
 * @typedef lssmap_t
 * @see lssmap
 */
typedef struct lssmap_ lssmap_t;

/**
 * @typedef bmp_gray_t
 * @brief bit map data type
 */
typedef gridplane<uint8_t> bmp8_t;

/**
 * @typedef bmp_t
 * @brief bit map data type
 */
typedef gridplane<uint32_t> bmp32_t;

int init_counting_map(cmap_t *m, double p, double u = DefaultMapCellSize);
int clear_counting_map(cmap_t *m);
int destroy_counting_map(cmap_t *m);
int counting_map(cmap_t *m, double x, double y);

int build_map(lssmap_t *map, cmap_t *cnt, double sr = 20, uint32_t alpha = 100, double blur = ErrorMargin, uint64_t np = 10 );
int inverse2x2(QGenericMatrix<2,2,double> &m1, QGenericMatrix<2,2,double> *m2);
int destroy_map(lssmap_t *m);
int likelihood(lssmap_t *map, double x, double y, double *l);
int likelihood(bmp32_t *m, double sx, double sy, double stheta, double r, double maxr, double *l);
int likelihood(bmp8_t *m, double sx, double sy, double stheta, double r, double maxr, double *l);
int read_counting_map(cmap_t *c,  const QString &d = CMapDirectoryDefault, const QString &f = CMapFileNameDefault, const QString &e = CMapFileExtension);
int write_counting_map(cmap_t *c,  const QString &d = CMapDirectoryDefault, const QString &f = CMapFileNameDefault, const QString &e = CMapFileExtension);


int build_bmp(bmp8_t *bmp, lssmap_t *map, double ps = 0.1, double sr = 0.0, double cp = 4.0);
int build_bmp(bmp32_t *bmp, lssmap_t *map, double ps = 0.1, double sr = 0.0, double cp = 4.0);

} //End of namespace

namespace gnd
{

/**
 * @ingroup GNDLSSMAP
 * @brief initialize counting map
 * @param[out] m : counting map
 * @param[in]  p : pixel size
 * @param[in]  u : grid map allocate unit size
 */
int init_counting_map(cmap_t *m, double p, double u)
{
    Q_ASSERT_X(m, "invalid", "invalid null pointer");
    Q_ASSERT_X(p > 0, "invalid","invalid argument. grid size must be greater than 0.");
    Q_ASSERT_X(u >= p, "invalid", "invalid argument. cell size must be greater than 0.");

    // ---> plane scan loop
    for( size_t i = 0; i < PLANE_NUM; i++)
    {
        // check error
        if( m->plane[i].is_allocate() )
        {
            qDebug() << "this map is buzy";
            qDebug() << "invalid argument, this map is buzy";
            exit(EXIT_FAILURE);
        }

        // allocate memory cell
        if( m->plane[i].pallocate(u, u, p, p) < 0)
        {
            qDebug() << "fail to allocate";
            qDebug() << "fail to allocate";
            exit(EXIT_FAILURE);
        }
        else
        {
            qDebug() << "map[" << i << "] allocated.";
        }
        // set origin
        m->plane[i].pset_core( (i % 2) * (p / 2.0), ((i / 2) % 2) * (p / 2.0) );
    } // <--- plane scan loop

    qDebug() << "End   - init_counting_map(" << "m," << p << "," << u << ")";

    return 0;
}

/**
 * @ingroup GNDLSSMAP
 * @brief clear counting map
 * @param[out] m :
 */
int clear_counting_map(cmap_t *m)
{
    Q_ASSERT_X(m, "invalid", "invalid null pointer");
    Q_ASSERT_X(!(m->plane+0) || !(m->plane + 1) || !(m->plane + 2) || !(m->plane + 3), "invalid", "invalid null pointer");

    cmap_pixel_t ini;

    for( size_t i = 0; i < PLANE_NUM; i++)
    {
        m->plane[i].set_uniform(&ini);
    }

    return 0;
}

/**
 * @ingroup GNDLSSMAP
 * @brief release counting map
 * @param[out] m : counting map
 */
int destroy_counting_map(cmap_t *m)
{
    Q_ASSERT_X(m, "invalid", "invalid null pointer");

    qDebug() << "Begin - destroy_counting_map(" << m << ")";

    for( size_t i = 0; i < PLANE_NUM; i++)
    {
        m->plane[i].deallocate();
    }

    qDebug() << " End - destroy_counting_map(" << m << ")";

    return 0;
}

/**
 * @ingroup GNDLSSMAP
 * @brief map counting function
 * @param[out] m : counting map
 * @param[in] x : laser scanner reflection point x
 * @param[in] y : laser scanner reflection point y
 */
int counting_map(cmap_t *m, double x, double y)
{
    Q_ASSERT_X(m, "invalid", "invalid null argument");

    QPointF xx(x,y);

    // ---> plene scanning loop
    for(size_t i = 0; i < PLANE_NUM; i++)
    {
        cmap_pixel_t *pp = 0;		                 // reference of pixel data
        QPointF core;
        QPointF pos;
        QGenericMatrix<2,2,double> cov;          // reflection point covariance in pixel
        qint64 r = 0, c = 0;

        // if memory is lacking, map data reallocate
        for( pp = m->plane[i].ppointer( x, y );
             pp == 0;
             pp = m->plane[i].ppointer( x, y ) )
        {
            m->plane[i].reallocate(x, y);
        }

        // get row and column number of reflection point pixel
        m->plane[i].pindex(x, y, &r, &c);
        // get core position of reflection point pixel
        m->plane[i].pget_pos_core(r, c, &core.rx(), &core.ry());

        // compute reflection point on pixel
        pos = xx - core;

        // compute covariance
        cov(0,0) = pos.x()*pos.x();
        cov(0,1) = pos.x()*pos.y();
        cov(1,0) = pos.y()*pos.x();
        cov(1,1) = pos.y()*pos.y();

        // add date
        pp->pos_sum += pos;
        pp->cov_sum += cov;
        pp->cnt++;
    } // <--- plene scanning loop

    return 0;
}

int inverse2x2(QGenericMatrix<2,2,double> &m1, QGenericMatrix<2,2,double> *m2)
{
    double det = m1(0,0)*m1(1,1) - m1(0,1)*m1(1,0);

    if(det == 0) return -1;

    (*m2)(0,0) = m1(1,1)  / det;
    (*m2)(0,1) = -m1(0,1) / det;
    (*m2)(1,0) = -m1(1,0) / det;
    (*m2)(1,1) = m1(0,0)  / det;

    return 0;
}

/**
 * @ingroup GNDLSSMAP
 * @brief map building function
 * @param[out]    map : builded map
 * @param[in]     cnt : laser scanner reflection point counting data
 * @param[in]      sr : sensor range
 * @param[in]   alpha : additional smoothing parameter
 * @param[in]    blur :
 * @param[in]      np :
 */
int build_map(lssmap_t *map, cmap_t *cnt, double sr, uint32_t alpha, double blur, uint64_t np )
{
    Q_ASSERT_X(cnt, "invalid", "invalid null pointer");
    Q_ASSERT_X(map, "invalid", "map is null");

    QGenericMatrix<2,2,double> cov;

    // ---> for each plane
    for(size_t i = 0; i < PLANE_NUM; i++)
    {
        if( map->plane[i].is_allocate() )
        {
            if( (map->plane[i].xrsl() != cnt->plane[i].xrsl()) || (map->plane[i].yrsl() != cnt->plane[i].yrsl()) )
            {
                qDebug() << "Fail - ini build_map";
                return -1;
            }
        }
        else
        {
            map->plane[i].allocate( cnt->plane[i]._plane_row_()*cnt->plane[i]._unit_row_(), cnt->plane[i]._plane_column_()*cnt->plane[i]._unit_column_());
            map->plane[i].pset_origin( cnt->plane[i].xlower(), cnt->plane[i].ylower());
            map->plane[i].pset_rsl( cnt->plane[i].xrsl(), cnt->plane[i].yrsl());
        }

        // ---> for each row
        for( quint64 r = 0; r < cnt->plane[i].row(); r++)
        {
            // ---> for each column
            for( quint64 c = 0; c < cnt->plane[i].column(); c++)
            {
                cmap_pixel_t cp;
                lssmap_pixel_t *pp;
                double x, y;
                quint64 cnt_cell = 0;
                quint64 sum;

                // get ndt data
                cnt->plane[i].get(r, c, &cp);
                cnt->plane[i].pget_pos_core(r, c, &x, &y);
                for( pp = map->plane[i].ppointer( x, y );
                     pp == 0;
                     pp = map->plane[i].ppointer( x, y ) )
                {
                    map->plane[i].reallocate(x, y);
                }


                // ---> obtain mean and inverse matrix of co-variance
                {
                    QGenericMatrix<2,2,double> ws2x2;

                    ws2x2(0,0) = AddSmooth_SumCovXX * alpha / qPow(AddSmooth_SampleNum,2) * qPow(cnt->plane[i].xrsl(),2);
                    ws2x2(0,1) = AddSmooth_SumCovXY * alpha / qPow(AddSmooth_SampleNum,2) * cnt->plane[i].xrsl() * cnt->plane[i].yrsl();
                    ws2x2(1,0) = AddSmooth_SumCovYX * alpha / qPow(AddSmooth_SampleNum,2) * cnt->plane[i].xrsl() * cnt->plane[i].yrsl();
                    ws2x2(1,1) = AddSmooth_SumCovYY * alpha / qPow(AddSmooth_SampleNum,2) * qPow(cnt->plane[i].yrsl(),2);

                    cp.cnt += alpha;
                    cp.cov_sum += ws2x2;

                    // compute mean
                    pp->mean = cp.pos_sum / (double)cp.cnt;

                    // compute covariance
                    ws2x2(0,0) = pp->mean.x()*cp.pos_sum.x();
                    ws2x2(0,1) = pp->mean.x()*cp.pos_sum.y();
                    ws2x2(1,0) = pp->mean.y()*cp.pos_sum.x();
                    ws2x2(1,1) = pp->mean.y()*cp.pos_sum.y();

                    ws2x2 = cp.cov_sum - ws2x2;
                    cov = ws2x2 / (double)cp.cnt;

                    // add minimal diagonal matrix
                    ws2x2.setToIdentity();
                    ws2x2 *= qPow(blur,2);
                    cov += ws2x2;

                    // compute inverse covariance
                    inverse2x2(cov,&pp->inv_cov);

                }
                // --->obtain mean and inverse matrix of co-variance

                {   // ---> compute evaluation gain
                    double det = 0;
                    double inv_normal = 0;

                    // obtain determinant of co-variance matrix
                    det = cov(0,0)*cov(1,1) - cov(0,1)*cov(1,0);
                    if( det < 0)
                    {
                        pp->k = 0;
                        continue;
                    }
                    // ---> obtain the sum of laser points in the sensor range for each plane
                    if( sr > 0 )
                    {
                        size_t f = qFloor( sr / map->plane[0].xrsl());
                        cmap_pixel_t *tmp_cpp;
                        quint64  lowerr;	// search range lower row
                        quint64  upperr;	// search range upper row
                        quint64  lowerc;	// search range lower column
                        quint64  upperc;	// search range upper column

                        lowerr = (r < f) ? 0 : r - f;
                        upperr = ((r + f) >= map->plane[i].row()) ? map->plane[i].row() : r + f;
                        lowerc = (c < f) ? 0 : c - f;
                        upperc = ((c + f) >= map->plane[i].column()) ? map->plane[i].column() : c + f;
                        sum = 0;

                        // compute average of local area number of observed point
                        for( quint64 rr = lowerr; rr < upperr; rr++)
                        {
                            for( quint64 cc = lowerc; cc < upperc; cc++)
                            {
                                tmp_cpp = cnt->plane[i].pointer( rr, cc );
                                sum += tmp_cpp->cnt;
                                cnt_cell++;
                            }
                        }
                    } // <--- obtain the sum of laser points in the sensor range for each plane
                    else
                    {
                        sum = 1;
                    }

                    if( np > 0 )
                    {   // --->  normalize factor
                        // ---> in a case that the cell have enough data
                        if(cp.cnt >= alpha + 3)
                        {
                            double k = 1.0 / (2.0 * M_PI * ( qSqrt(det) ));
                            uint32_t pcnt = 0;
                            uint32_t ix, iy;

                            QGenericMatrix<1,2,double> xx;    //row=2,col=1
                            QGenericMatrix<2,1,double> ws1x2;
                            QGenericMatrix<1,1,double> ws1x1;

                            for(ix = 0; (-(map->plane[i].xrsl() * ix / np) + pp->mean.x()) >= (- map->plane[i].xrsl() / 2); ix++)
                            {
                                for(iy = 0; (-(map->plane[i].yrsl() * iy / np) + pp->mean.y()) >= (- map->plane[i].yrsl() / 2); iy++)
                                {
                                    xx(0,0) = -(map->plane[i].xrsl() * ix / np) + pp->mean.x();
                                    xx(1,0) = -(map->plane[i].yrsl() * iy / np) + pp->mean.y();
                                    // difference from mean
                                    xx(0,0) -= pp->mean.x();
                                    xx(1,0) -= pp->mean.y();

                                    // compute likelihood
                                    ws1x2 = xx.transposed() * pp->inv_cov;
                                    ws1x1 = ws1x2 * xx;
                                    ws1x1(0,0) = k * ::qExp( - ws1x1(0,0) / 2.0) * (map->plane[i].xrsl() * map->plane[i].yrsl());
                                    inv_normal += ws1x1(0,0);
                                    pcnt++;
                                }
                            }

                            for(ix = 1; (-(map->plane[i].xrsl() * ix / np) + pp->mean.x()) >= (- map->plane[i].xrsl() / 2); ix++)
                            {
                                for(iy = 1; ((map->plane[i].yrsl() * iy / np) + pp->mean.y()) < (map->plane[i].yrsl() / 2); iy++)
                                {
                                    xx(0,0) = -(map->plane[i].xrsl() * ix / np) + pp->mean.x();
                                    xx(1,0) = (map->plane[i].yrsl() * iy / np) + pp->mean.y();

                                    // difference from mean
                                    xx(0,0) -= pp->mean.x();
                                    xx(1,0) -= pp->mean.y();

                                    // compute likelihood
                                    ws1x2 = xx.transposed() * pp->inv_cov;
                                    ws1x1 = ws1x2 * xx;
                                    ws1x1(0,0) = k * qExp( - ws1x1(0,0) / 2.0) * (map->plane[i].xrsl() * map->plane[i].yrsl());
                                    inv_normal += ws1x1(0,0);
                                    pcnt++;
                                }
                            }

                            for(ix = 1; ((map->plane[i].xrsl() * ix / np) + pp->mean.x()) < (map->plane[i].xrsl() / 2); ix++)
                            {
                                for(iy = 1; (-(map->plane[i].yrsl() * iy / np) + pp->mean.y()) >= (- map->plane[i].yrsl() / 2); iy++)
                                {
                                    xx(0,0) = (map->plane[i].xrsl() * ix / np) + pp->mean.x();
                                    xx(1,0) = -(map->plane[i].yrsl() * iy / np) + pp->mean.y();

                                    // difference from mean
                                    xx(0,0) -= pp->mean.x();
                                    xx(1,0) -= pp->mean.y();
                                    // compute likelihood
                                    ws1x2 = xx.transposed() * pp->inv_cov;
                                    ws1x1 = ws1x2 * xx;
                                    ws1x1(0,0) = k * qExp( - ws1x1(0,0) / 2.0) * (map->plane[i].xrsl() * map->plane[i].yrsl());
                                    inv_normal += ws1x1(0,0);
                                    pcnt++;
                                }
                            }

                            for(ix = 1; ((map->plane[i].xrsl() * ix / np) + pp->mean.x()) < (map->plane[i].xrsl() / 2); ix++)
                            {
                                for(iy = 1; (map->plane[i].yrsl() * iy / np) + pp->mean.y() < map->plane[i].yrsl() / 2; iy++)
                                {
                                    xx(0,0) = (map->plane[i].xrsl() * ix / np) + pp->mean.x();
                                    xx(1,0) = (map->plane[i].yrsl() * iy / np) + pp->mean.y();
                                    // difference from mean
                                    xx(0,0) -= pp->mean.x();
                                    xx(1,0) -= pp->mean.y();
                                    // compute likelihood
                                    ws1x2 = xx.transposed() * pp->inv_cov;
                                    ws1x1 = ws1x2 * xx;
                                    ws1x1(0,0) = k * ::exp( - ws1x1(0,0) / 2.0) * (map->plane[i].xrsl() * map->plane[i].yrsl());
                                    inv_normal += ws1x1(0,0);
                                    pcnt++;
                                }
                            }

                            if( pcnt > 0)
                            {
                                inv_normal /= pcnt;
                            }
                            else
                            {
                                return -1;
                            }

                        }	// <--- in a case that the cell have enough data
                        else if(alpha > 0)
                        {
                            inv_normal = AddSmooth_Eta;
                        }
                        else
                        {
                            inv_normal = 0;
                        }
                    } // <---  normalize factor

                    // obtain determinant of co-variance matrix
                    if( inv_normal <= 0)
                    {
                        pp->k = 0;
                        continue;
                    }

                    pp->k = ( ((double) (cp.cnt)) / ((double)sum + cnt_cell * alpha)) / ( qSqrt(det) );

                } // <--- compute evaluation gain
            } // <-- for each column
      } // <--- for each row

    } // <--- for each plane

    return 0;
}

/**
 * @ingroup GNDPSM
 * @brief destory map
 * @param[out] m :  map
 */
int destroy_map(lssmap_t *m)
{
    Q_ASSERT_X(m, "invalid", "invalid null pointer");

    for( size_t i = 0; i < PLANE_NUM; i++)
    {
        m->plane[i].deallocate();
    }

    return 0;
}

/**
 * @ingroup GNDPSM
 * @brief compute likelihood
 * @param[in] map : map data
 * @param[in]   x : laser scanner reflection point x
 * @param[in]   y : laser scanner reflection point y
 * @param[in]  pg : position gain
 * @param[out]  l : likelihood
 */
int likelihood(lssmap_t *map, double x, double y, double *l)
{
    Q_ASSERT_X(l, "invalid", "invalid null pointer");
    Q_ASSERT_X(map, "invalid", "map is null");

    QGenericMatrix<1,2,double> pos, q;
    QGenericMatrix<2,1,double> ws1x2;  // workspace 2x2 matrix
    QGenericMatrix<1,1,double> ws1x1;  // workspace 1x1 matrix

    //initiate
    pos(0,0) = x;
    pos(1,0) = y;
    *l = 0;

    // ---> for
    for( size_t i = 0; i < PLANE_NUM; i++)
    {
        lssmap_pixel_t *pp = 0;
        int ret;
        qint64 pr, pc;

        // get index of pixel
        ret = map->plane[i].pindex( pos(0,0), pos(1,0), &pr, &pc );
        // no data
        if( ret < 0 )			continue;

        // get pixel data
        pp = map->plane[i].pointer( pr, pc );
        // zero weight
        if( pp->k <= 0.0 )	continue;

        // get pixel core pos on ndt data pixel
        map->plane[i].pget_pos_core(pr, pc, &q(0,0), &q(1,0));
        q = pos - q;

        // difference from mean
        q(0,0) -= pp->mean.x();
        q(1,0) -= pp->mean.y();
        // compute likelihood
        ws1x2 = q.transposed() * pp->inv_cov;
        ws1x1 = ws1x2 * q;

        *l += (pp->k) * qExp( -ws1x1(0,0) / 2.0);
    } // ---> for

    // normalization
    *l /= PLANE_NUM;
    return 0;
}

/**
 * @ingroup GNDPSM
 * @brief compute likelihood
 * @param[in] map : map data
 * @param[in]     sx : sensor position x
 * @param[in]     sy : sensor position y
 * @param[in] stheta : sensor direction
 * @param[in]      r : sensor reading (range)
 * @param[in]   maxr : sensor range
 * @param[out]     l : likelihood
 */
int likelihood(bmp32_t *m, double sx, double sy, double stheta, double r, double maxr, double *l)
{
    Q_ASSERT_X(l, "invalid", "invalid null pointer");
    Q_ASSERT_X(m, "invalid", "map is null");

    double sinv, cosv;
    double ur = m->xrsl() < m->yrsl() ? m->xrsl() : m->yrsl();

    sinv = qSin(stheta);
    cosv = qCos(stheta);
    *l = 0;

    { // ---> compute likelihood
        double ieta = 0;
        double xx, yy;
        double rr;
        uint32_t v;

        if( m->pget(sx + r * cosv, sy + r * sinv, &v) < 0 )
        {
            *l = 0;
            return -1;
        }

        // normalize factor
        for( rr = 0; rr < maxr; rr += ur )
        {
            uint32_t vv;
            xx = sx + rr * cosv;
            yy = sy + rr * sinv;
            if( m->pget(xx, yy, &vv) < 0 )
            {
                vv = 0;
            }
            ieta += vv;
        }
        *l = (double) v / ieta;

    } // <--- compute likelihood
    return 0;
}

/**
 * @ingroup GNDPSM
 * @brief compute likelihood
 * @param[in] map : map data
 * @param[in]     sx : sensor position x
 * @param[in]     sy : sensor position y
 * @param[in] stheta : sensor direction
 * @param[in]      r : sensor reading (range)
 * @param[in]   maxr : sensor range
 * @param[out]     l : likelihood
 */
int likelihood(bmp8_t *m, double sx, double sy, double stheta, double r, double maxr, double *l)
{
    Q_ASSERT_X(l, "invalid", "invalid null pointer");
    Q_ASSERT_X(m, "invalid", "map is null");

    double sinv, cosv;
    double ur = m->xrsl() / 2;

    sinv = qSin(stheta);
    cosv = qCos(stheta);
    *l = 0;

    { // ---> compute likelihood
        double ieta = 0;
        double xx, yy;
        double rr;
        uint8_t v;

        if( m->pget(sx + r * cosv, sy + r * sinv, &v) < 0 )
        {
            *l = 0;
            return -1;
        }

        // normalize factor
        for( rr = 0; rr < maxr; rr += ur )
        {
            uint8_t vv;
            xx = sx + rr * cosv;
            yy = sy + rr * sinv;
            if( m->pget(xx, yy, &vv) < 0 )
            {
              vv = 0;
            }
            ieta += vv;
        }

        *l = (double) v / ieta;
    } // <--- compute likelihood
    return 0;
}

/**
 * @ingroup GNDPSM
 * @brief counting map file read
 * @param[out] c : counting map
 * @param[in] d : directory path
 * @param[in] f : file name template
 * @param[in] e : extention
 */
//int read_counting_map(cmap_t *c,  const char* d, const char* f, const char* e)
int read_counting_map(cmap_t *c,  const QString &d, const QString &f, const QString &e)
{
    Q_ASSERT_X(c, "invalid", "invalid null argument");
    Q_ASSERT_X(d.size() > 0, "invalid", "invalid null argument");
    Q_ASSERT_X(f.size() > 0, "invalid", "invalid null argument");
    Q_ASSERT_X(e.size() > 0, "invalid", "invalid null argument");

    QString path;
    // ---> map plane data scanning loop
    for( uint i = 0; i < PLANE_NUM; i++)
    {
        path = d + "/" + f + "." + QString::number(i) + "." + e;
        if( c->plane[i].fread(path) < 0 )
        {
            return -1;
        }
    } // <--- map plane data scanning loop

  return 0;
}

/**
 * @ingroup GNDPSM
 * @brief counting map file out
 * @param[in] c : counting map
 * @param[in] d : directory path
 * @param[in] f : file name template
 * @param[in] e : extention
 */
//int write_counting_map(cmap_t *c,  const char* d, const char* f, const char* e)
int write_counting_map(cmap_t *c,  const QString &d, const QString &f, const QString &e)
{
    Q_ASSERT_X(c, "invalid", "invalid null argument");
    Q_ASSERT_X(d.size() > 0, "invalid", "invalid null argument");
    Q_ASSERT_X(f.size() > 0, "invalid", "invalid null argument");
    Q_ASSERT_X(e.size() > 0, "invalid", "invalid null argument");

    QString path;
    // ---> map plane data scanning loop
    for( uint8_t i = 0; i < PLANE_NUM; i++){
        path = d + "/" + f + "." + QString::number(i) + "." + e;

        if( c->plane[i].fwrite(path) < 0 )
        {
            return -1;
        }
    } // <--- map plane data scanning loop

    return 0;
}

/**
 * @brief build bitmap data (gray)
 * @param[out] bmp : gray scale bit map
 * @param[in] map  : map data
 * @param[in] ps   : pixel size
 * @param[in] sr   : sensor range (for smoothing)
 * @param[in] cp   : contrast parameter
 */
int build_bmp(bmp8_t *bmp, lssmap_t *map, double ps, double sr, double cp)
{
    Q_ASSERT_X(bmp, "invalid", "invalid null argument");
    Q_ASSERT_X(map, "invalid", "invalid null argument");
    Q_ASSERT_X(ps > 0, "invalid", "invalid argument. pixel size must be greater than 0.");

    gridplane<double> ws;	// workspace

    // allocate bmp data
    if(bmp->is_allocate())
    {
        bmp->deallocate();
    }

    bmp->pallocate(map->plane[0].xupper() - map->plane[3].xlower(), map->plane[0].yupper() - map->plane[3].ylower(), ps, ps);
    bmp->pset_origin(map->plane[3].xlower(), map->plane[3].ylower());

    ws.pallocate(map->plane[0].xupper() - map->plane[3].xlower(), map->plane[0].yupper() - map->plane[3].ylower(), ps, ps);
    ws.pset_origin(map->plane[3].xlower(), map->plane[3].ylower());

    double lkh = 0;		// likelihood
    double x, y;		  // bitmap pixel core position
    double mu;
    double max = 0;
    double sigma;
    quint64 cnt = 0;

    { // ---> compute likelihood and average
        mu = 0;

        for( quint64 r = 0; r < ws.row(); r++)
        {
            for( quint64 c = 0; c < ws.column(); c++)
            {
                // get pixel core position
                ws.pget_pos_core(r, c, &x, &y);

                // compute likelihood
                likelihood(map, x, y, &lkh);

                lkh *= (bmp->xrsl() * bmp->yrsl());

                ws.set(r, c, &lkh);

                max = lkh > max ? lkh : max;

                if( lkh != 0 )
                {
                    mu += lkh;
                    cnt++;
                }
            }
        }
        if( cnt == 0 )
        {
            return -1;
        }
        mu /= cnt;

        sigma = 0;
        for( quint64 r = 0; r < ws.row(); r++)
        {
            for( quint64 c = 0; c < ws.column(); c++)
            {
                ws.get(r, c, &lkh);
                if( lkh == 0 ) continue;
                sigma += qPow(lkh - mu, 2);
            }
        }
        sigma /= cnt;
        sigma = qSqrt(sigma);
    } // ---> compute likelihood and average

    { // ---> set bitmap
        uint8_t bpv;	// bitmap pixel value

        for( quint64 r = 0; r < bmp->row(); r++)
        {
            // ---> grid map scanning loop column
            for( quint64 c = 0; c < bmp->column(); c++)
            {
                ws.get(r, c, &lkh);

                // normalize for bitmap
                if( lkh <= 0 )
                {
                    bpv = 0;
                }
                else
                {
                    lkh = (lkh) / (max);
                    lkh *= 0xff;
                    bpv = lkh > 0xff ? 0xff : (uint8_t)lkh;
                }

                bmp->set( r, c, &bpv);
            } // <--- grid map scanning loop column
        } // <--- grid map scanning loop
    } // <--- set bitmap

    ws.deallocate();

    return 0;
}

/**
 * @brief build bitmap data (gray)
 * @param[out] bmp : gray scale bit map
 * @param[in] map  : map data
 * @param[in] ps   : pixel size
 * @param[in] sr   : sensor range (for smoothing)
 * @param[in] cp   : contrast parameter
 */
int build_bmp(bmp32_t *bmp, lssmap_t *map, double ps, double sr, double cp)
{

    Q_ASSERT_X(bmp, "invalid", "invalid null argument");
    Q_ASSERT_X(map, "invalid", "invalid null argument");
    Q_ASSERT_X(ps > 0, "invalid", "invalid argument. pixel size must be greater than 0.");

    gridplane<double> ws;	// workspace

    // allocate bmp data
    if(bmp->is_allocate())
    {
        bmp->deallocate();
    }

    bmp->pallocate(map->plane[0].xupper() - map->plane[3].xlower(), map->plane[0].yupper() - map->plane[3].ylower(), ps, ps);
    bmp->pset_origin(map->plane[3].xlower(), map->plane[3].ylower());

    ws.pallocate(map->plane[0].xupper() - map->plane[3].xlower(), map->plane[0].yupper() - map->plane[3].ylower(), ps, ps);
    ws.pset_origin(map->plane[3].xlower(), map->plane[3].ylower());

    double lkh = 0;		// likelihood
    double x, y;		  // bitmap pixel core position
    double mu;
    double sigma;
    double lkh_max;
    quint64 cnt = 0;

    {   // ---> compute likelihood and average
        mu = 0;
        lkh_max = 0;

        for( quint64 r = 0; r < ws.row(); r++)
        {
            for( quint64 c = 0; c < ws.column(); c++)
            {
                // get pixel core position
                ws.pget_pos_core(r, c, &x, &y);

                // compute likelihood
                likelihood(map, x, y, &lkh);
                lkh *= (bmp->xrsl() * bmp->yrsl());

                ws.set(r, c, &lkh);

                if( lkh != 0 )
                {
                    mu += lkh;
                    cnt++;
                }
                lkh_max = lkh_max < lkh ? lkh : lkh_max;
            }
        }
        if( cnt == 0 )
        {
            return -1;
        }
        mu /= cnt;

        sigma = 0;
        for( quint64 r = 0; r < ws.row(); r++)
        {
            for( quint64 c = 0; c < ws.column(); c++)
            {
                ws.get(r, c, &lkh);
                if( lkh == 0 ) continue;
                sigma += qPow(lkh - mu, 2);
            }
        }
        sigma /= cnt;
        sigma = qSqrt(sigma);
    } // ---> compute likelihood and average

    { // ---> set bitmap
        uint32_t bpv;	// bitmap pixel value
        double max = lkh_max;

        for( quint64 r = 0; r < bmp->row(); r++)
        {
            // ---> grid map scanning loop column
            for( quint64 c = 0; c < bmp->column(); c++)
            {
                ws.get(r, c, &lkh);

                // normalize for bitmap
                if( lkh <= 0 )
                {
                    bpv = 0;
                }
                else
                {
                    lkh = (lkh) / (max);
                    lkh *= 0xffffff;
                    bpv = lkh > (0xffffff) ? (0xffffff) : (uint32_t)lkh;
                }

                bmp->set( r, c, &bpv);
            } // <--- grid map scanning loop column
        } // <--- grid map scanning loop
    } // <--- set bitmap

    ws.deallocate();

    return 0;
}

} //End of namespace

#endif // GND_LSSMAP_BASE_H
