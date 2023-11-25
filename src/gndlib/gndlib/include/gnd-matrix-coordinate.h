/*
 * matrix-coordinate-convert.hpp
 *
 *  Created on: 2011/07/03
 *      Author: tyamada
 *  Updated by ryu, 2023/5/24
 *  .Use Qt v5.14 lib to update
 */
#ifndef GND_MATRIX_COORDINATE_H
#define GND_MATRIX_COORDINATE_H

#include <QGenericMatrix>
#include <QVector>
#include <QtMath>
#include <QtGlobal>

namespace gnd
{
namespace matrix
{

inline
double __coordinate_matrix_inner_product__(
    const double x1, const double y1, const double z1,
    const double x2, const double y2, const double z2 )
{
    return x1*x2 + y1*y2 + z1*z2;
}

/**
 * @brief compute coordinate convert matrix
 * @param[out]  mt : cooridnate convert matix
 * @param[in]    x : origin x position
 * @param[in]    y : origin y position
 * @param[in]    z : origin z position
 * @param[in] xaxx : x element of x-axis unit vector
 * @param[in] xaxy : y element of x-axis unit vector
 * @param[in] xaxz : z element of x-axis unit vector
 * @param[in] zaxx : x element of z-axis unit vector
 * @param[in] zaxy : y element of z-axis unit vector
 * @param[in] zaxz : z element of z-axis unit vector
 */
inline
int coordinate_converter( QGenericMatrix<4,4,double> *mt,
    const double x,    const double y,    const double z,
    const double xaxx, const double xaxy, const double xaxz,
    const double zaxx, const double zaxy, const double zaxz)
{
    static const double error_margin = 1.0e-8;
    Q_ASSERT_X(mt, "invalid", "null pointer");

    { // ---> operation
        double xx, xy, xz;
        double yx, yy, yz;
        double zx, zy, zz;

        { // ---> x-axis
            double norm =
                __coordinate_matrix_inner_product__( xaxx, xaxy, xaxz, xaxx, xaxy, xaxz );

            // normalization
            Q_ASSERT_X(norm != 0, "invalid", "x-axis is 0 vector");
            if(1.0 - norm < 1.0e-8)
            {
                xx = xaxx;
                xy = xaxy;
                xz = xaxz;
            }
            else
            {
                norm = qSqrt(norm);
                xx = xaxx / norm;
                xy = xaxy / norm;
                xz = xaxz / norm;
            }
        } // <--- x-axis


        { // ---> z-axis
            double inner_prod;
            double norm;

            inner_prod =
                __coordinate_matrix_inner_product__( xaxx, xaxy, xaxz, zaxx, zaxy, zaxz );

            if(qFabs(inner_prod) > error_margin)
            {
                qWarning("input z-axis not a vertical axis of x");
                zx = zaxx;
                zy = zaxy;
                zz = zaxz;
            }
            else
            {
                zx = zaxx - xaxx * inner_prod;
                zy = zaxy - xaxy * inner_prod;
                zz = zaxz - xaxz * inner_prod;
            }

            // normalization
            norm = __coordinate_matrix_inner_product__( zx, zy, zz, zx, zy, zz );
            Q_ASSERT_X(norm != 0, "invalid", "z-axis is 0 vector");
            if(1.0 - norm >= 1.0e-8)
            {
                norm = qSqrt(norm);
                zx = zaxx / norm;
                zy = zaxy / norm;
                zz = zaxz / norm;
            }
        } // <--- z-axis


        { // ---> y-axis
            // external product ZxX
            yx = zy * xz - zz * xy;
            yy = zz * xx - zx * xz;
            yz = zx * xy - zy * xx;
        } // <--- y-axis

        { // ---> integrate
            (*mt)(0,0) = xx;
            (*mt)(0,1) = xy;
            (*mt)(0,2) = xz;
            (*mt)(0,3) = 0;

            (*mt)(1,0) = yx;
            (*mt)(1,1) = yy;
            (*mt)(1,2) = yz;
            (*mt)(1,3) = 0;

            (*mt)(2,0) = zx;
            (*mt)(2,1) = zy;
            (*mt)(2,2) = zz;
            (*mt)(2,3) = 0;

            (*mt)(3,0) = x;
            (*mt)(3,1) = y;
            (*mt)(3,2) = z;
            (*mt)(3,3) = 1;

        } // <--- integrate
        *mt = mt->transposed();

    } // <--- operation
    return 0;
}

/**
 * @brief Compute Determinant (matrix 4x4)
 * @param [in]  mt  : Matrix
 * @param [in]  ws  : Work Space
 * @param [out] v 	: Determinant
 * @return ==0 : success
 * @return  <0 : fail
 */
inline
int _det_4x4_( QGenericMatrix<4,4,double> *mt, double *v)
{
  *v =  (*mt)(0,0) *
      (     (*mt)(1,1) * ((*mt)(2,2) * (*mt)(3,3) - (*mt)(2,3) * (*mt)(3,2))
          + (*mt)(1,2) * ((*mt)(2,3) * (*mt)(3,1) - (*mt)(2,1) * (*mt)(3,3))
          + (*mt)(1,3) * ((*mt)(2,1) * (*mt)(3,2) - (*mt)(2,2) * (*mt)(3,1))
      )
      + (*mt)(0,1) *
      (     (*mt)(1,0) * ((*mt)(2,3) * (*mt)(3,2) - (*mt)(2,2) * (*mt)(3,3))
          + (*mt)(1,2) * ((*mt)(2,0) * (*mt)(3,3) - (*mt)(2,3) * (*mt)(3,0))
          + (*mt)(1,3) * ((*mt)(2,2) * (*mt)(3,0) - (*mt)(2,0) * (*mt)(3,2))
      )
      + (*mt)(0,2) *
      (     (*mt)(1,0) * ((*mt)(2,1) * (*mt)(3,3) - (*mt)(2,3) * (*mt)(3,1))
          + (*mt)(1,1) * ((*mt)(2,3) * (*mt)(3,0) - (*mt)(2,0) * (*mt)(3,3))
          + (*mt)(1,3) * ((*mt)(2,0) * (*mt)(3,1) - (*mt)(2,1) * (*mt)(3,0))
      )
      + (*mt)(0,3) *
      (     (*mt)(1,0) * ((*mt)(2,2) * (*mt)(3,1) - (*mt)(2,1) * (*mt)(3,2))
          + (*mt)(1,1) * ((*mt)(2,0) * (*mt)(3,2) - (*mt)(2,2) * (*mt)(3,0))
          + (*mt)(1,2) * ((*mt)(2,1) * (*mt)(3,0) - (*mt)(2,0) * (*mt)(3,1))
      );

    return 0;
}

/**
 * @brief Compute Invert Matrix (4x4)
 * @param [in]  mt  : Matrix (4x4)
 * @param [out] inv : Inverse Matrix
 * @return ==0 : success
 * @return ==1 :
 * @return  <0 : fail
 */
inline
int _inverse_4x4_( QGenericMatrix<4,4,double> *mt, QGenericMatrix<4,4,double> *inv)
{
    double det;

    _det_4x4_(mt, &det);

    (*inv)(0,0) = (  (*mt)(1,1) * ((*mt)(2,2) * (*mt)(3,3) - (*mt)(2,3) * (*mt)(3,2) )
                   + (*mt)(1,2) * ((*mt)(2,3) * (*mt)(3,1) - (*mt)(2,1) * (*mt)(3,3) )
                   + (*mt)(1,3) * ((*mt)(2,1) * (*mt)(3,2) - (*mt)(2,2) * (*mt)(3,1) )
                  ) / det;

    (*inv)(0,1) = (  (*mt)(0,1) * ((*mt)(2,3) * (*mt)(3,2) - (*mt)(2,2) * (*mt)(3,3) )
                   + (*mt)(0,2) * ((*mt)(2,1) * (*mt)(3,3) - (*mt)(2,3) * (*mt)(3,1) )
                   + (*mt)(0,3) * ((*mt)(2,2) * (*mt)(3,1) - (*mt)(2,1) * (*mt)(3,2) )
                  ) / det;

    (*inv)(0,2) = (  (*mt)(0,1) * ((*mt)(1,2) * (*mt)(3,3) - (*mt)(1,3) * (*mt)(3,2) )
                   + (*mt)(0,2) * ((*mt)(1,3) * (*mt)(3,1) - (*mt)(1,1) * (*mt)(3,3) )
                   + (*mt)(0,3) * ((*mt)(1,1) * (*mt)(3,2) - (*mt)(1,2) * (*mt)(3,1) )
                  ) / det;

    (*inv)(0,3) = (  (*mt)(0,1) * ((*mt)(1,3) * (*mt)(2,2) - (*mt)(1,2) * (*mt)(2,3) )
                   + (*mt)(0,2) * ((*mt)(1,1) * (*mt)(2,3) - (*mt)(1,3) * (*mt)(2,1) )
                   + (*mt)(0,3) * ((*mt)(1,2) * (*mt)(2,1) - (*mt)(1,1) * (*mt)(2,2) )
                  ) / det;
  //---------------------------------------------------------------------------------------------//

    (*inv)(1,0) = (  (*mt)(1,0) * ((*mt)(2,3) * (*mt)(3,2) - (*mt)(2,2) * (*mt)(3,3) )
                   + (*mt)(1,2) * ((*mt)(2,0) * (*mt)(3,3) - (*mt)(2,3) * (*mt)(3,0) )
                   + (*mt)(1,3) * ((*mt)(2,2) * (*mt)(3,0) - (*mt)(2,0) * (*mt)(3,2) )
                  ) / det;

    (*inv)(1,1) = (  (*mt)(0,0) * ((*mt)(2,2) * (*mt)(3,3) - (*mt)(2,3) * (*mt)(3,2) )
                   + (*mt)(0,2) * ((*mt)(2,3) * (*mt)(3,0) - (*mt)(2,0) * (*mt)(3,3) )
                   + (*mt)(0,3) * ((*mt)(2,0) * (*mt)(3,2) - (*mt)(2,2) * (*mt)(3,0) )
                  ) / det;

    (*inv)(1,2) = (  (*mt)(0,0) * ((*mt)(1,3) * (*mt)(3,2) - (*mt)(1,2) * (*mt)(3,3) )
                   + (*mt)(0,2) * ((*mt)(1,0) * (*mt)(3,3) - (*mt)(1,3) * (*mt)(3,0) )
                   + (*mt)(0,3) * ((*mt)(1,2) * (*mt)(3,0) - (*mt)(1,0) * (*mt)(3,2) )
                  ) / det;

    (*inv)(1,3) = (  (*mt)(0,0) * ((*mt)(1,2) * (*mt)(2,3) - (*mt)(1,3) * (*mt)(2,2) )
                   + (*mt)(0,2) * ((*mt)(1,3) * (*mt)(2,0) - (*mt)(1,0) * (*mt)(2,3) )
                   + (*mt)(0,3) * ((*mt)(1,0) * (*mt)(2,2) - (*mt)(1,2) * (*mt)(2,0) )
                  ) / det;
  //---------------------------------------------------------------------------------------------//

    (*inv)(2,0) = (  (*mt)(1,0) * ((*mt)(2,1) * (*mt)(3,3) - (*mt)(2,3) * (*mt)(3,1) )
                   + (*mt)(1,1) * ((*mt)(2,3) * (*mt)(3,0) - (*mt)(2,0) * (*mt)(3,3) )
                   + (*mt)(1,3) * ((*mt)(2,0) * (*mt)(3,1) - (*mt)(2,1) * (*mt)(3,0) )
                  ) / det;

    (*inv)(2,1) = (  (*mt)(0,0) * ((*mt)(2,3) * (*mt)(3,1) - (*mt)(2,1) * (*mt)(3,3) )
                   + (*mt)(0,1) * ((*mt)(2,0) * (*mt)(3,3) - (*mt)(2,3) * (*mt)(3,0) )
                   + (*mt)(0,3) * ((*mt)(2,1) * (*mt)(3,0) - (*mt)(2,0) * (*mt)(3,1) )
                  ) / det;

    (*inv)(2,2) = (  (*mt)(0,0) * ((*mt)(1,1) * (*mt)(3,3) - (*mt)(1,3) * (*mt)(3,1) )
                   + (*mt)(0,1) * ((*mt)(1,3) * (*mt)(3,0) - (*mt)(1,0) * (*mt)(3,3) )
                   + (*mt)(0,3) * ((*mt)(1,0) * (*mt)(3,1) - (*mt)(1,1) * (*mt)(3,0) )
                  ) / det;

    (*inv)(2,3) = (  (*mt)(0,0) * ((*mt)(1,3) * (*mt)(2,1) - (*mt)(1,1) * (*mt)(2,3) )
                   + (*mt)(0,1) * ((*mt)(1,0) * (*mt)(2,3) - (*mt)(1,3) * (*mt)(2,0) )
                   + (*mt)(0,3) * ((*mt)(1,1) * (*mt)(2,0) - (*mt)(1,0) * (*mt)(2,1) )
                  ) / det;
  //---------------------------------------------------------------------------------------------//

    (*inv)(3,0) = (  (*mt)(1,0) * ((*mt)(2,2) * (*mt)(3,1) - (*mt)(2,1) * (*mt)(3,2) )
                   + (*mt)(1,1) * ((*mt)(2,0) * (*mt)(3,2) - (*mt)(2,2) * (*mt)(3,0) )
                   + (*mt)(1,2) * ((*mt)(2,1) * (*mt)(3,0) - (*mt)(2,0) * (*mt)(3,1) )
                  ) / det;

    (*inv)(3,1) = (  (*mt)(0,0) * ((*mt)(2,1) * (*mt)(3,2) - (*mt)(2,2) * (*mt)(3,1) )
                   + (*mt)(0,1) * ((*mt)(2,2) * (*mt)(3,0) - (*mt)(2,0) * (*mt)(3,2) )
                   + (*mt)(0,2) * ((*mt)(2,0) * (*mt)(3,1) - (*mt)(2,1) * (*mt)(3,0) )
                  ) / det;

    (*inv)(3,2) = (  (*mt)(0,0) * ((*mt)(1,2) * (*mt)(3,1) - (*mt)(1,1) * (*mt)(3,2) )
                   + (*mt)(0,1) * ((*mt)(1,0) * (*mt)(3,2) - (*mt)(1,2) * (*mt)(3,0) )
                   + (*mt)(0,2) * ((*mt)(1,1) * (*mt)(3,0) - (*mt)(1,0) * (*mt)(3,1) )
                  ) / det;

    (*inv)(3,3) = (  (*mt)(0,0) * ((*mt)(1,1) * (*mt)(2,2) - (*mt)(1,2) * (*mt)(2,1) )
                   + (*mt)(0,1) * ((*mt)(1,2) * (*mt)(2,0) - (*mt)(1,0) * (*mt)(2,2) )
                   + (*mt)(0,2) * ((*mt)(1,0) * (*mt)(2,1) - (*mt)(1,1) * (*mt)(2,0) )
                  ) / det;

    return 0;
}


} //End of namespace

} //End of namespace

#endif // GND_MATRIX_COORDINATE_H
