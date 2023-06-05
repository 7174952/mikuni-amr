/****************************************
 * gnd-coord-tree.h
 *
 *  Created on: 2011/06/23
 *      Author: tyamada
 *
 *  Updated by ryu, 2023/04/25
 *  .Use Qt v5.14 lib to generate random data
 *
 ***************************************/

#ifndef GND_RANDOM_H
#define GND_RANDOM_H

#include <QQueue>
#include <QGenericMatrix>
#include <QtMath>
#include <QtGlobal>
#include <QRandomGenerator>
#include <QVector>

// ---> function declaration
namespace gnd {

template < typename VCT >
inline
int assign_to_as_vector ( QVector<double> *dest, VCT *src, const size_t r, const size_t c);

inline
int sqnorm(const QVector<double> *vec,  double *v);

inline
int inner_prod(const QVector<double> *m1, const QVector<double> *m2, double *sum);
/*
 * @brief generate random value following a gaussian distribution (box-muller method)
 */
inline
double random_gaussian(const double sigma);

/*
 * @brief generate random value following a gaussian distribution (multi-dimension)
 */
template < typename MTRX1 >
inline
int random_gaussian_mult(MTRX1 *cov, QVector<double> *ws, QVector<double> *out);

/*
 * @brief generate random value following a gaussian distribution (multi-dimension)
 */
template < uint32_t D >
inline
int random_gaussian_mult(QGenericMatrix<D,D,double> *cov, QVector<double> *ws, QVector<double> *out);

} // <--- function declaration


namespace gnd {// ---> namespace gnd

/**
 * @brief Compute Norm (sub-vector row)
 * @param [in]  vec : vector
 * @param [in]  size : vector size
 * @param [out]  v : result
 * @return ==0 : success
 * @return  <0 : fail
 */
inline
int sqnorm(const QVector<double> *vec,  double *v)
{
    Q_ASSERT_X(vec->size() > 0, "empty", "vector empty");
    *v = 0;

    for (uint i = 0; i < vec->size(); i++)
    {
        *v += vec->at(i) * vec->at(i);
    }

    return 0;
}

inline int inner_prod(const QVector<double> *m1, const QVector<double> *m2, double *sum)
{
    Q_ASSERT_X(m1 && m2, "null", "null pointer");
    Q_ASSERT_X((m1->size() == m2->size()), "invalid", "vector size invalid");

    // ---> compute inner product
    uint32_t i;
    *sum = 0;
    for(i = 0; i < m1->size(); i++)
    {
        *sum += m1->at(i) * m2->at(i);
    }

    return 0;
}

/**
 * @brief assign mt with buffer
 * @param [out] dest : buffer assigned matrix
 * @param  [in]  src : buffer source matrix
 * @param  [in]    r : row index
 * @param  [in]    c : column index
 * @param  [in]    s : size
 */
template < typename VCT >
inline
int assign_to_as_vector ( QVector<double> *dest, VCT *src, const size_t r, const size_t c)
{
    Q_ASSERT_X(dest && src, "null", "null pointer");

    for(uint32_t i = 0; i < dest->size(); i++)
    {
        (*dest)[i] = (*src)(r,c+i);
    }

    return 0;
}

/**
 * @brief generate random value following a gaussian distribution (box-muller method)
 * @param [in] sigma : standard deviation
 */
inline
double random_gaussian(const double sigma)
{
    double x, y, r;

    do
    {
      x = - 1 + 2 * QRandomGenerator::global()->generateDouble();
      y = - 1 + 2 * QRandomGenerator::global()->generateDouble();

      r = x * x + y * y;
    } while( (r > 1.0) || (r == 0) );

    return sigma * y * qSqrt(-2.0 * qLn(r) / r);
}

/**
 * @brief generate random value following a gaussian distribution (multi-dimension)
 * @param [in]  cov : matrix
 * @param [in]  n : vector size
 * @param [in]  ws: temp work
 * @param [out] out: result
 * @return ==0 : success
 * @return  <0 : fail
 */
template < typename MTRX1>
inline
int random_gaussian_mult(MTRX1 *cov, QVector<double> *ws, QVector<double> *out)
{
    // ---> initial
    uint32_t i, j;
    for(i = 0; i < ws->size(); i++)
        (*ws)[i] = random_gaussian(1.0);
    // ---> operation
    double l00;

    Q_ASSERT_X((*cov)(0,0) > 0, "sqrt", "sqrt minus failure.");
    l00 = qSqrt((*cov)(0,0));
    (*cov)(0,0) = l00;
    (*out)[0] = ws->at(0) * l00;
    if(out->size() > 1)
    {
        double l10 = (*cov)(1,0) / l00;
        double diag = (*cov)(1,1) - l10 * l10;
        double l11;

        Q_ASSERT_X(diag >= 0, "sqrt", "sqrt minus failure.");
        l11 = qSqrt(diag);

        (*cov)(1,0) = l10;
        (*cov)(1,1) = l11;
        (*out)[1] = ws->at(0) * l10 +  ws->at(1) * l11;
    }

    // cholesky decomposition
    for(j = 2; j < out->size(); j++)
    {
        (*out)[j] = 0;

        for(i = 0; i < j; i++)
        {
            double sum = 0;
            double aji = (*cov)(j,i);
            double aii = (*cov)(i,i);
            double lji;

            if(i != 0)
            {
                QVector<double> di;
                QVector<double> dj;

                di.resize(i);
                dj.resize(i);
                assign_to_as_vector(&di,cov,i,0);
                assign_to_as_vector(&dj,cov,j,0);

                inner_prod(&di,&dj,&sum);
            }
            Q_ASSERT_X(aii != 0, "devide", "devide 0 failure.");
            lji = (aji - sum) / aii;
            (*cov)(j,i) = lji;
            (*out)[j] += lji * ws->at(i);
        } // for(i)

        // ---> diagonal
        QVector<double> dj;
        double sum;
        double diag;
        double ljj;

        dj.resize(j);
        assign_to_as_vector(&dj, cov, j, 0);
        sqnorm(&dj, &sum);

        diag = (*cov)(j,j) - sum;
        Q_ASSERT_X(diag >= 0, "sqrt", "sqrt minus failure.");
        ljj = qSqrt(diag);
        (*cov)(j,j) = ljj;
        (*out)[j] += ljj * ws->at(j);

    }// for(j)

  return 0;
}


/*
 * @brief generate random value following a gaussian distribution (multi-dimension)
 */
template < uint32_t D >
inline
int random_gaussian_mult(QGenericMatrix<D,D,double> *cov, QVector<double> *ws, QVector<double> *out) {

    Q_ASSERT_X(cov && out , "null", "null pointer");

    if(ws->size() < D) ws->resize(D);
    if(out->size() < D) out->resize(D);

    return random_gaussian_mult<QGenericMatrix<D,D,double> >(cov, ws, out);
}

} // <--- namespace gnd

#endif // GND_RANDOM_H
