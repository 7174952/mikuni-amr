/*
 * gnd_bmp.hpp
 *
 *  Created on: 2011/08/10
 *      Author: tyamada
 *  Updated by ryu, 2023/5/17
 *  .Use Qt v5.14 lib to update map functions
 */
#ifndef GND_BMP_H
#define GND_BMP_H

#include <QString>
#include "gnd-gridmap.h"
#include "gnd-lssmap-base.h"

namespace gnd {

static const uint8_t FILETYPE[] = "BM";			      ///< bit map file type tag
static const uint8_t FILEHEADER_SIZE = 14;			  ///< bit map file header byte size
static const uint8_t INFOHEADER_SIZE = 40;			  ///< bit map info header byte size
static const uint8_t INFOHEADER_COMMOM_SIZE = 16;	///< bit map info common header byte size
static const uint8_t RGBQUAD_SIZE = 4;				    ///< rgb quad byte size

/**
 * @privatesection
 * @ingroup GNDBmp
 * @brief bit map file header
 */
struct FILEHEADER_
{
    uint16_t	  bfType;			      ///< file type
    uint32_t		bfSize;			      ///< file size
    uint16_t	  bfReserved1;	    ///< reserve (0)
    uint16_t  	bfReserved2;	    ///< reserve (0)
    uint32_t		bfOffBits;		    ///< offset byte size at image data
};
typedef struct FILEHEADER_ FILEHEADER;


/**
 * @privatesection
 * @ingroup GNDBmp
 * @brief bit map info header
 */
struct INFOHEADER_
{
    uint32_t		biSize;			      ///< info header size
    int32_t			biWidth;		      ///< width (pixel)
    int32_t			biHeight;		      ///< height (pixel)
    uint16_t	  biPlanes;		      ///< number of plane (1)
    uint16_t	  biBitCount;		    ///< data size per pixel
    uint32_t  	biCompression;	  ///< compression method
    uint32_t		biSizeImage;	    ///< image size
    int32_t			biXPixPerMeter;	  ///< pixel per meter on x-axis
    int32_t			biYPixPerMeter;	  ///< pixel per meter on y-axis
    uint32_t		biClrUsed;		    ///< number of color index
    uint32_t		biClrImporant;	  ///< number of important color index
};
typedef struct INFOHEADER_  INFOHEADER;

/**
 * @privatesection
 * @ingroup GNDBmp
 * @brief RGB quad
 */
struct RGBQUAD_
{
    uint8_t   rgbBlue;			///< blue
    uint8_t   rgbGreen;			///< green
    uint8_t   rgbRed;			  ///< red
    uint8_t   rgbReserved;  ///< reserve
};
typedef struct RGBQUAD_  RGBQUAD;

int write( const QString &fname, bmp8_t *gm );
int read( const QString &fname, bmp8_t *gm );

int write( const QString &fname, bmp32_t *gm );
int read( const QString &fname, bmp32_t *gm );

} //End of namespace


namespace gnd {
/**
 * @ingroup GNDBmp
 * @brief write gray bit map file
 * @param[in] fname : file name
 * @param[in]    gm : grid map data
 * @return  ==0 : success
 *          < 0 : failure
 */
int write( const QString &fname, bmp8_t *gm)
{
    FILEHEADER  fheader;
    INFOHEADER  iheader;
    const uint32_t pixlsize = 1;
    int32_t linesize, imagesize;

    struct
    {
        double x;
        double y;
    } org;
    size_t pad;

    linesize = pixlsize * gm->column();
    pad = ((linesize % 4) == 0) ? 0 : (4 - (linesize % 4));

    linesize += pad;

    imagesize = linesize * gm->row();
    gm->pget_origin(&org.x, &org.y);

    // ---> file header
    ::memset(&fheader, 0, sizeof(fheader));
    ::memcpy(&fheader.bfType, FILETYPE, sizeof(fheader.bfType));
    fheader.bfSize = FILEHEADER_SIZE + INFOHEADER_SIZE + imagesize + RGBQUAD_SIZE * 256;
    fheader.bfReserved1 = 0;
    fheader.bfReserved2 = 0;
    fheader.bfOffBits = FILEHEADER_SIZE + INFOHEADER_SIZE + RGBQUAD_SIZE * 256;

    // ---> info header
    // add origin data
    iheader.biSize         = INFOHEADER_SIZE;
    iheader.biWidth        = gm->column() + pad; //2023.6.1 bug fix - must be divide by 4
    iheader.biHeight       = gm->row();
    iheader.biPlanes       = 1;
    iheader.biBitCount     = 8 * pixlsize;
    iheader.biCompression  = 0;
    iheader.biSizeImage    = imagesize;
    iheader.biXPixPerMeter = qRound( 1.0 / gm->xrsl() );
    iheader.biYPixPerMeter = qRound( 1.0 / gm->yrsl() );
    iheader.biClrUsed      = 0;
    iheader.biClrImporant  = 0;

    { // ---> file out
        QFile file(fname);
        if(!file.open( QIODevice::WriteOnly))
        {
            qDebug() << "New bmp8 file failed!";
            return -1;
        }

        QDataStream out(&file);
        out.setByteOrder(QDataStream::LittleEndian);

        const uint8_t d = 0x00;	// dummy
        RGBQUAD	rgb;

        // ---> write file header
        out << fheader.bfType
            << fheader.bfSize
            << fheader.bfReserved1
            << fheader.bfReserved2
            << fheader.bfOffBits;

        // ---> write info header
        out << iheader.biSize
            << iheader.biWidth
            << iheader.biHeight
            << iheader.biPlanes
            << iheader.biBitCount
            << iheader.biCompression
            << iheader.biSizeImage
            << iheader.biXPixPerMeter
            << iheader.biYPixPerMeter
            << iheader.biClrUsed
            << iheader.biClrImporant;

        // ---> write colar pallete
        for(uint32_t i = 0; i < (1<<8); i++)
        {
            rgb.rgbBlue = rgb.rgbGreen = rgb.rgbRed = i;
            out << rgb.rgbBlue
                << rgb.rgbGreen
                << rgb.rgbRed
                << rgb.rgbReserved;
        }

        // write image data
        for( quint64 h = 0; h < (unsigned)iheader.biHeight; h++ )
        {
            for( quint64 w = 0; w < (unsigned)iheader.biWidth - pad; w += gm->_unit_column_() )
            {
                out << *gm->pointer(h,w);
            }
            for( size_t i = 0; i < pad; i++ )
            {
                out << d;
            }
        }

        file.close();
    } // <--- file out

  return 0;
}

/**
 * @ingroup GNDBmp
 * @brief read gray bit map file
 * @param[in]  fname : file name
 * @param[out]    gm : grid map data
 * @return  ==0 : success
 *          < 0 : failure
 */
int read( const QString &fname, bmp8_t *gm)
{
    QFile file(fname);
    if(!file.open(QIODevice::ReadOnly))
    {
        qDebug() << "Open bmp8 file failed!";
        return -1;
    }

    QDataStream in(&file);
    FILEHEADER fheader;
    INFOHEADER iheader;
    const uint32_t pixlsize = 1;
//    size_t pad;

    // ---> file header
    ::memset(&fheader, 0, sizeof(fheader));
    in >> fheader.bfType
       >> fheader.bfSize
       >> fheader.bfReserved1
       >> fheader.bfReserved2
       >> fheader.bfOffBits;

    // ---> info header
    ::memset(&iheader, 0, sizeof(iheader));
    in >> iheader.biSize
       >> iheader.biWidth
       >> iheader.biHeight
       >> iheader.biPlanes
       >> iheader.biBitCount
       >> iheader.biCompression
       >> iheader.biSizeImage
       >> iheader.biXPixPerMeter
       >> iheader.biYPixPerMeter
       >> iheader.biClrUsed
       >> iheader.biClrImporant;

    { // ---> set data
        uint8_t d;	// dummy
        gm->allocate(iheader.biHeight, iheader.biWidth);
        gm->pset_rsl( 1.0 / iheader.biXPixPerMeter, 1.0 / iheader.biYPixPerMeter);

//        pad = (pixlsize * iheader.biWidth) % 4;

        errno = 0;

        if(!file.seek(fheader.bfOffBits))
        {
            return -2;
        }

        // read image data
        for( quint64 h = 0; h < (unsigned)iheader.biHeight; h++ )
        {
            for(quint64 w = 0; w < (unsigned)iheader.biWidth; w++)
            {
                if(in.atEnd())
                {
                    return -2;
                }
                in >> *gm->pointer(h, w);
            }

//            for( size_t i = 0; i < pad; i++ )
//            {
//                if(in.atEnd())
//                {
//                    return -2;
//                }
//                in >> d;
//            }
        }
    } // <--- set data

    file.close();

  return 0;
}

/**
 * @ingroup GNDBmp
 * @brief write gray bit map file
 * @param[in] fname : file name
 * @param[in]    gm : grid map data
 * @return  ==0 : success
 *          < 0 : failure
 */
int write( const QString &fname, bmp32_t *gm)
{
    FILEHEADER  fheader;
    INFOHEADER  iheader;
    const size_t  pixlsize = 4;
    quint64  linesize, imagesize;
    struct
    {
        double x;
        double y;
    } org;
    size_t pad;

    // initialize
    linesize = pixlsize * gm->column();
    pad = ((linesize % 4) == 0) ? 0 : (4 - (linesize % 4));

    linesize += pad;

    imagesize = linesize * gm->row();
    gm->pget_origin(&org.x, &org.y);


    // file header
    ::memset(&fheader, 0, sizeof(fheader));
    ::memcpy(&fheader.bfType, FILETYPE, sizeof(fheader.bfType));
    fheader.bfSize = FILEHEADER_SIZE + INFOHEADER_SIZE + imagesize;
    fheader.bfReserved1 = 0;
    fheader.bfReserved2 = 0;
    fheader.bfOffBits = FILEHEADER_SIZE + INFOHEADER_SIZE;

    // info header
    // add origin data
    iheader.biSize = INFOHEADER_SIZE;
    iheader.biWidth = gm->column() + pad; //2023.6.1 bug fix - must be divide by 4
    iheader.biHeight = gm->row();
    iheader.biPlanes = 1;
    iheader.biBitCount = 8 * pixlsize;
    iheader.biCompression = 0;
    iheader.biSizeImage = imagesize;
    iheader.biXPixPerMeter = qRound( 1.0 / gm->xrsl() );
    iheader.biYPixPerMeter = qRound( 1.0 / gm->yrsl() );
    iheader.biClrUsed      = 0;
    iheader.biClrImporant  = 0;

    { // ---> file out
        QFile file(fname);
        if(!file.open(QIODevice::WriteOnly))
        {
            qDebug() << "New bmp8 file failed!";
            return -1;
        }

        QDataStream out(&file);
        const uint32_t d = 0x00;	// dummy
        out.setByteOrder(QDataStream::LittleEndian);

        // write file header
        out << fheader.bfType
            << fheader.bfSize
            << fheader.bfReserved1
            << fheader.bfReserved2
            << fheader.bfOffBits;

        // write info header
        out << iheader.biSize
            << iheader.biWidth
            << iheader.biHeight
            << iheader.biPlanes
            << iheader.biBitCount
            << iheader.biCompression
            << iheader.biSizeImage
            << iheader.biXPixPerMeter
            << iheader.biYPixPerMeter
            << iheader.biClrUsed
            << iheader.biClrImporant;

        // write image data
        for( quint64 h = 0; h < (unsigned)iheader.biHeight; h++ )
        {
            for( quint64 w = 0; w < (unsigned)iheader.biWidth - pad; w += gm->_unit_column_() )
            {
                out << *gm->pointer(h,w);
            }
            for( size_t i = 0; i < pad; i++ )
            {
                out << d;
            }
        }

        file.close();
    } // <--- file out

    return 0;
}

/**
 * @ingroup GNDBmp
 * @brief read gray bit map file
 * @param[in]  fname : file name
 * @param[out]    gm : grid map data
 * @return  ==0 : success
 *          < 0 : failure
 */
int read( const QString &fname, bmp32_t *gm)
{
    QFile file(fname);
    if(!file.open(QIODevice::ReadOnly))
    {
        qDebug() << "Open bmp8 file failed!";
        return -1;
    }

    QDataStream in(&file);
    FILEHEADER fheader;
    INFOHEADER iheader;
    const size_t pixlsize = 4;
//    size_t pad;

    // file header
    ::memset(&fheader, 0, sizeof(fheader));
    in >> fheader.bfType
       >> fheader.bfSize
       >> fheader.bfReserved1
       >> fheader.bfReserved2
       >> fheader.bfOffBits;

    // ---> info header
    ::memset(&iheader, 0, sizeof(iheader));
    in >> iheader.biSize
       >> iheader.biWidth
       >> iheader.biHeight
       >> iheader.biPlanes
       >> iheader.biBitCount
       >> iheader.biCompression
       >> iheader.biSizeImage
       >> iheader.biXPixPerMeter
       >> iheader.biYPixPerMeter
       >> iheader.biClrUsed
       >> iheader.biClrImporant;

    { // ---> set data
        uint8_t d;	// dummy
        gm->allocate(iheader.biHeight, iheader.biWidth);
        gm->pset_rsl( 1.0 / iheader.biXPixPerMeter, 1.0 / iheader.biYPixPerMeter);

//        pad = (pixlsize * iheader.biWidth) % 4;

        errno = 0;
        if(!file.seek(fheader.bfOffBits))
        {
            return -2;
        }

        // read image data
        for( quint64 h = 0; h < (unsigned)iheader.biHeight; h++ )
        {
            for(quint64 w = 0; h < (unsigned)iheader.biWidth; w++)
            {
                if(in.atEnd()) return -2;
                in >> *gm->pointer(h, w);
            }

//            for( size_t i = 0; i < pad; i++ )
//            {
//                if(in.atEnd()) return -2;
//                in >> d;
//            }
        }
    } // <--- set data

    file.close();

    return 0;
}

} //End of namespace

#endif // GND_BMP_H
