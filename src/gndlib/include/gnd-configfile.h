/*
 * gnd-config-file.hpp
 *
 *  Created on: 2011/09/16
 *      Author: tyamada
 *
 *  Updated by ryu, 2023/04/29
 *  .Use Qt v5.14 lib to read/write config file
 */
#ifndef GND_CONFIGFILE_H
#define GND_CONFIGFILE_H

#include <QString>
#include <QFile>
#include <QtCore>
#include <QTextStream>
#include <QVector>
#include <QQueue>
#include <QtGlobal>

namespace gnd {

template<typename T>
struct Param
{
    QString item;
    QVector<T> value;
    QString comment;
};

enum BoolType
{
    TRUE_FALSE = 0,
    YES_NO,
    ON_OFF
};

/**
 * @brief config item saved
 */
class ConfigItem
{
private:
    QString _item;
    QString _value;
    QString _comment;

public:
    // ---> constructor
    ConfigItem();
    ConfigItem(const QString &name, const QString &value, const QString &comment = "");
    ~ConfigItem();

    int set(const QString &name, const QString &value, const QString &comment);
    int get(QString *pname, QString *pvalue, QString *pcomment) const;
    QString name() const;
    QString value() const;
    QString comment() const;
};

/**
 * @brief save all config items
 */
class ConfigFile
{
private:
    QFile file;
    ConfigItem itemSave;
    QQueue<ConfigItem> qConfigItems;

public:
    // ---> constructor
    ConfigFile();
    ~ConfigFile();

    int read_from_file(const QString &fileName);
    int32_t get_param(Param<double> *dest);
    int32_t get_param(Param<QString> *dest);
    int32_t get_param(Param<bool> *dest);
    //update parameters
    int32_t set_param(const Param<double> &src);
    int32_t set_param(const Param<QString> &src);
    int32_t set_param(const Param<bool> &src,uint type);

    //save new parameters to config file
    int32_t write_to_file(QString fileName);

};

} //namespace gnd


namespace gnd {


ConfigItem::ConfigItem()
{
    _item = "";
    _value = "";
    _comment = "";
}

/**
 * @brief Init 1 item data
 * @param [in] name : item name
 * @param [in] value : value of item
 * @param [in] comment : comment of item
 */
ConfigItem::ConfigItem(const QString &name, const QString &value, const QString &comment)
{
    _item = name;
    _value = value;
    _comment = comment;
}

ConfigItem::~ConfigItem()
{

}

/**
 * @brief get 1 item data from file
 * @param [out] pname : item name
 * @param [out] pvalue : value of item
 * @param [out] pcomment : comment of item
 * @return    <0 : fail
 *           >=0 : success
 */
int ConfigItem::get(QString *pname, QString *pvalue, QString *pcomment) const
{
    *pname = _item;
    *pvalue = _value;
    *pcomment = _comment;
    return 0;
}

/**
 * @brief set 1 item data
 * @param [in] name : item name
 * @param [in] value : value of item
 * @param [in] comment : comment of item
 * @return    <0 : fail
 *           >=0 : success
 */
int ConfigItem::set(const QString &name, const QString &value, const QString &comment)
{
    _item = name;
    _value = value;
    _comment = comment;
    return 0;
}

QString ConfigItem::name() const
{
    return _item;
}

QString ConfigItem::value() const
{
    return _value;
}

QString ConfigItem::comment() const
{
    return _comment;
}


/**
 * @brief init file handle
 * @param [in] fileName : config file name
 * @return   : -
 */
ConfigFile::ConfigFile()
{

}

ConfigFile::~ConfigFile()
{
    if(file.exists())
    {
        file.close();
    }
}

/**
 * @brief read all items to buff from config file
 * @param [in] fileName : config file name
 * @return    <0 : fail
 *           >=0 : success
 */
int ConfigFile::read_from_file(const QString &fileName)
{
    file.setFileName(fileName);
    if(file.exists())
    {
        file.open(QIODevice::ReadWrite | QIODevice::Text);
    }
    else
    {
        qDebug() << "Config file is not exist!";
        return -1;
    }

    QTextStream itemIn(&file);
    QStringList strList;
    QString commentTmp;
    QString nameTmp;
    QString valueTmp;
    QString str;

    while(!itemIn.atEnd())
    {
        if((str = itemIn.readLine().trimmed()) == "") continue;

        if(str.at(0)=='#') //find comment line
        {
            commentTmp = str.right(str.size() - 1).trimmed();
        }
        else
        {
            //seperate item and value by <=,#>
            strList.clear();
            strList = str.split(QRegularExpression("(=|#)"));

            if(str.contains('='))
            {
                nameTmp = strList.at(0);
                if(str.contains('{')) //matrix value
                {
                    if(itemIn.atEnd()) return -1;
                    QString strTmp;
                    valueTmp.clear();

                    do
                    {
                        strTmp = itemIn.readLine().trimmed();
                        if(strTmp.at(0) == '#') continue;
                        if(strTmp.contains('}')) break; //matrix value over
                        QStringList tmp = strTmp.split(QRegularExpression("(,|;| )"));
                        for(const QString &s:tmp)
                        {
                            if((s.size() > 0) && (s.at(0) != '#'))
                            {
                                valueTmp.append(s + ";");
                            }
                        }
                        //valueTmp.append(strTmp.split(QRegularExpression("(,|;|#| )"))[0] + ";"); //val0;val1;val2;...

                    } while(!itemIn.atEnd() );

                }
                else //single value
                {
                    valueTmp = strList.at(1).trimmed() + ";";
                }
           }
           //save file item to queue
           itemSave.set(nameTmp,valueTmp,commentTmp);
           qConfigItems.push_back(itemSave);
        }
    }

    file.close();

    return 0;
}

/**
 * @brief read 1 parameter infomation from buffer
 * @param [in/out] dest : get target parameter and save
 * @return    <0 : fail
 *           >=0 : success
 */
int32_t ConfigFile::get_param(Param<double> *dest)
{
    for(const ConfigItem &item_tmp : qConfigItems)
    {
        //save param info to dest
        if(item_tmp.name() == dest->item)
        {
            bool ok;

            dest->comment = item_tmp.comment();
            dest->item = item_tmp.name();
            //matrix value
            QStringList tmpList = item_tmp.value().split(';');

            dest->value.resize(tmpList.size()-1);

            for(uint i = 0; i < dest->value.size();i++)
            {
                dest->value[i] = tmpList.at(i).toDouble(&ok);
                if(ok == false) //not number
                {
                    return -2;
                }
            }
            return 0;
        }
    }
    return -1; //not exist
}

/**
 * @brief read 1 parameter infomation from buffer
 * @param [in/out] dest : get target parameter and save
 * @return    <0 : fail
 *           >=0 : success
 */
int32_t ConfigFile::get_param(Param<QString> *dest)
{
    for(const ConfigItem &item_tmp : qConfigItems)
    {
        //save param info to dest
        if(item_tmp.name() == dest->item)
        {
            dest->comment = item_tmp.comment();
            dest->item = item_tmp.name();
            //matrix value
            QStringList tmpList = item_tmp.value().split(';');
            dest->value.resize(tmpList.size()-1);

            for(uint i = 0; i < dest->value.size();i++)
            {
                dest->value[i] = tmpList.at(i);
            }
            return 0;
        }
    }
    return -1; //not exist
}

/**
 * @brief read 1 parameter infomation from buffer
 * @param [in/out] dest : get target parameter and save
 * @return    <0 : fail
 *           >=0 : success
 */
int32_t ConfigFile::get_param(Param<bool> *dest)
{
    for(const ConfigItem &item_tmp : qConfigItems)
    {
        //save param info to dest
        if(item_tmp.name() == dest->item)
        {
            dest->comment = item_tmp.comment();
            dest->item = item_tmp.name();
            //matrix value
            QStringList tmpList = item_tmp.value().split(';');
            dest->value.resize(tmpList.size()-1);

            for(uint i = 0; i < dest->value.size();i++)
            {
                if(tmpList.at(i).toLower()=="true" || tmpList.at(i).toLower()=="yes" || tmpList.at(i).toLower()=="on")
                {
                    dest->value[i] = true;
                }
                else if(tmpList.at(i).toLower()=="false" || tmpList.at(i).toLower()=="no" || tmpList.at(i).toLower()=="off")
                {
                    dest->value[i] = false;
                }
                else
                {
                    return -2;
                }
            }
            return 0;
        }
    }
    return -1; //not exist
}

/**
 * @brief set 1 parameter infomation to buffer
 * @param [in] src : update new parameter value to buff
 * @return    <0 : fail
 *           >=0 : success
 */
int32_t ConfigFile::set_param(const Param<QString> &src)
{
    bool exist = false;
    QString str_val = "";

    for(uint i = 0; i < src.value.size();i++)
    {
        str_val += src.value[i] + ";";
    }

    itemSave.set(src.item,str_val,src.comment);

    for(uint i = 0; i < qConfigItems.size(); i++)
    {
        //search parameter
        if(qConfigItems.at(i).name() == src.item)
        {
            qConfigItems[i] = itemSave;
            exist = true;
        }
    }
    //add new parameter item to back
    if(exist == false)
    {
        qConfigItems.append(itemSave);
    }
    return 0;
}

/**
 * @brief set 1 parameter infomation to buffer
 * @param [in] src : update new parameter value to buff
 * @return    <0 : fail
 *           >=0 : success
 */
int32_t ConfigFile::set_param(const Param<double> &src)
{
    bool exist = false;
    QString str_val = "";

    for(uint i = 0; i < src.value.size();i++)
    {
        str_val += QString::number(src.value[i]) + ";";
    }

    itemSave.set(src.item,str_val,src.comment);

    for(uint i = 0; i < qConfigItems.size(); i++)
    {
        //search parameter
        if(qConfigItems.at(i).name() == src.item)
        {
            qConfigItems[i] = itemSave;
            exist = true;
        }
    }
    //add new parameter item to back
    if(exist == false)
    {
        qConfigItems.append(itemSave);
    }
    return 0;
}

/**
 * @brief set 1 parameter infomation to buffer
 * @param [in] src : update new parameter value to buff
 * @param [in] type : 0-true/false, 1-yes/no, 2-on/off
 * @return    <0 : fail
 *           >=0 : success
 */
int32_t ConfigFile::set_param(const Param<bool> &src,uint type)
{
    bool exist = false;
    QString str_val = "";
    QStringList boolTrue;
    QStringList boolFalse;

    boolTrue << "true" << "yes" << "on";
    boolFalse << "false" << "no" << "off";


    for(uint i = 0; i < src.value.size();i++)
    {
        if(src.value[i] == true)
        {
            str_val += boolTrue.at(type) + ";";
        }
        else
        {
            str_val += boolFalse.at(type) + ";";
        }
    }

    itemSave.set(src.item,str_val,src.comment);

    for(uint i = 0; i < qConfigItems.size(); i++)
    {
        //search parameter
        if(qConfigItems.at(i).name() == src.item)
        {
            qConfigItems[i] = itemSave;
            exist = true;
        }
    }
    //add new parameter item to back
    if(exist == false)
    {
        qConfigItems.append(itemSave);
    }
    return 0;
}

/**
 * @brief write buff to config file
 * @param [in] fileName: save file of name
 * @return    <0 : fail
 *           >=0 : success
 */
int32_t ConfigFile::write_to_file(QString fileName)
{
    //check old file status
    file.setFileName(fileName);

    //Open or create new file
    if(file.exists())
    {
        file.open(QIODevice::WriteOnly  | QIODevice::Text);
    }
    else
    {
        file.open(QIODevice::NewOnly | QIODevice::Text);
    }

    QTextStream outToFile(&file);

    //Save all buff context to file
    for(uint i = 0; i < qConfigItems.size(); i++)
    {
        outToFile << "# " << qConfigItems[i].comment() << "\r\n";
        QStringList strList = qConfigItems[i].value().split(';');
        if(strList.size() > 0)
        {
            strList.removeLast();
        }

        //----matrix value format--------//
        //----a00 a01 a02----------------//
        //----a10 a11 a12----------------//
        //----a20 a21 a22----------------//
        if(strList.size() > 1)
        {
            outToFile << qConfigItems[i].name() << "={ \r\n" ;
            for(uint j = 0; j < strList.size(); j++)
            {
                outToFile << "        " << strList[j] << ",";
                if(strList.size() > 3)
                {
                    if((j+1) % 3 == 0)
                        outToFile << "\r\n";
                }
                else
                {
                    outToFile << "\r\n";
                }
            }
            outToFile << "} \r\n";
        }
        else
        {
            outToFile << qConfigItems[i].name() << "=" << strList.first() << "\r\n";
        }

        outToFile << "\r\n";

    }

    file.close();

    return 0;
}


}
#endif // GND_CONFIGFILE_H
