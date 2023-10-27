/*
 * gnd-path-io.h
 *
 *  Created on: 2011/08/09
 *      Author: tyamada
 *
 *  Updated by ryu, 2023/10/11
 *  .Use Qt v5.14 lib to update path information
 */

#ifndef GND_PATH_IO_H
#define GND_PATH_IO_H

#include <QString>
#include <QStringList>
#include <QFile>
#include <QTextStream>
#include <QStack>

#include "gnd-path.h"
#include "gnd-configfile.h"
#include "gnd_rosutil.h"

// <--- function declaration
namespace gnd {
	namespace path {
      static const QString Default_Item_Waypoint         = "waypoints";
      static const QString Default_Item_Path_List        = "path";
      static const QString Default_Item_Path_Start       = "start";
      static const QString Default_Item_Path_Destination = "destination";
      static const QString Format_Item_Path              = "from_%s_to_%s";

      static const QString Default_Item_Path_Limit_Speed_Translate = "speed_translate";
      static const QString Default_Item_Path_Limit_Speed_Rotate    = "speed_rotate";
      static const QString Default_Item_Path_Start_Extend          = "start_extend";
      static const QString Default_Item_Path_End_Extend            = "end_extend";
      static const QString Default_Item_Path_Left_Width            = "left_width";
      static const QString Default_Item_Path_Right_Width           = "right_width";

      int fread(const char* fname, path_net_speed_limited *path);
      int fwrite(const char* fname, path_net_speed_limited *path);

      int fread(const char* fname, path_net_area_and_speed_limited *path);
      int fwrite(const char* fname, path_net_area_and_speed_limited *path);

	}
} // <--- function declaration



// ---> function definition
namespace gnd {
	namespace path {

		inline
    int fread(const char* fname, path_net_speed_limited *net)
    {
        const int GET_DEFAULT   = 0;
        const int GET_WAYPOINTS = 1;
        const int GET_LINEPATH  = 2;

        int data_type = GET_DEFAULT;
        QStack<QString> path_tmp;
        QString strLine;
        QStringList strList;
        waypoint_named_t wp_ws;
        uint data_cnt = 0;

        pathprop_speed_t lp_ws;
        QString start;
        QString dest;

        QFile file(fname);

        if(file.exists())
        {
            file.open(QIODevice::ReadOnly | QIODevice::Text);
        }
        else
        {
            return -1;
        }

        QTextStream itemStreamIn(&file);

        net->clear();

        while(!itemStreamIn.atEnd())
        {
            strLine = itemStreamIn.readLine().trimmed();
            if(strLine.isEmpty()) continue;

            //check data type
            if(strLine.contains(Default_Item_Waypoint))
            {
                if(path_tmp.isEmpty())
                {
                    //start to get waypoint
                    data_type = GET_WAYPOINTS;
                    if(strLine.contains("{"))
                    {
                        path_tmp.push("{");
                    }
                    continue;
                }
                else
                {
                    return -1;
                }
            }
            else if(strLine.contains(Default_Item_Path_List))
            {
                if(path_tmp.isEmpty())
                {
                    //start to get path
                    data_type = GET_LINEPATH;
                    if(strLine.contains("{"))
                    {
                        path_tmp.push("{");
                    }
                    continue;
                }
                else
                {
                    return -1;
                }
            }

            //save waypoints to buffer
            if(data_type == GET_WAYPOINTS)
            {
                if(strLine.contains("{"))
                {
                    path_tmp.push("{");
                    strList = strLine.split('=');
                    strcpy(wp_ws.name,strList.at(0).toStdString().c_str());
                }
                else if(strLine.contains("}"))
                {
                    if(path_tmp.pop().contains("{"))
                    {
                        //set to buffer
                        if(path_tmp.size() > 0)
                        {
                            net->add_waypoint(wp_ws.name, wp_ws.x, wp_ws.y);

                        }
                        else
                        {
                           //Do nothing, get waypoints complete
                        }
                    }
                    else
                    {
                        //path file format error
                        return -1;
                    }
                }
                else
                {
                    if(data_cnt == 0)
                    {
                        wp_ws.x = strLine.toDouble();
                        data_cnt++;
                    }
                    else
                    {
                        wp_ws.y = strLine.toDouble();
                        data_cnt = 0;
                    }
                }
            }

            //save linepath to buffer
            if(data_type == GET_LINEPATH)
            {
                if(strLine.contains("{"))
                {
                    path_tmp.push("{");
                }
                else if(strLine.contains("}"))
                {
                    if(path_tmp.pop().contains("{"))
                    {
                        //set to buffer
                        if(path_tmp.size() > 0)
                        {
                            net->add_linepath(start.toStdString().c_str(),dest.toStdString().c_str(),&lp_ws);
                        }
                        else
                        {
                            //Do nothing, get linepath complete
                        }
                    }
                    else
                    {
                        //path file format error
                        return -1;
                    }
                }
                else
                {
                    //set items value: item = value
                    strList = strLine.split('=');
                    if(strLine.contains(Default_Item_Path_Start))
                    {
                        start = strList.at(1);
                    }
                    if(strLine.contains(Default_Item_Path_Destination))
                    {
                        dest = strList.at(1);
                    }
                    if(strLine.contains(Default_Item_Path_Limit_Speed_Translate))
                    {
                        lp_ws.limit_translate = strList.at(1).toDouble();
                    }
                    if(strLine.contains(Default_Item_Path_Limit_Speed_Rotate))
                    {
                        lp_ws.limit_rotate = strList.at(1).toDouble();
                    }
                }
            }

        }

        return 0;
		}


		inline
    int fwrite(const char* fname, path_net_speed_limited *net)
    {
        QFile file(fname);
        file.open(QIODevice::NewOnly | QIODevice::WriteOnly | QIODevice::Text);

        QTextStream outStreamItem(&file);

        // write waypoint
        outStreamItem << Default_Item_Waypoint << "={" << "\n";
        for(int i = 0; i < net->n_waypoints(); i++)
        {
            outStreamItem << "\t" << (*net)[i].waypoint.name << "={" << "\n";
            outStreamItem << "\t\t" << (*net)[i].waypoint.x << "\n";
            outStreamItem << "\t\t" << (*net)[i].waypoint.y << "\n";
            outStreamItem << "\t" << "}" << "\n";
        }
        outStreamItem << "}" << "\n";

        // write path list
        QString start, dest, path_name;
        int j, k;

        outStreamItem << Default_Item_Path_List << "={" << "\n";
        for(j = 0; j < net->n_waypoints(); j++)
        {
            start = QString((*net)[j].waypoint.name);
            for(k = 0; k < (*net)[j].list.size(); k++)
            {
                dest = QString((*net)[j].list[k].end.name);
                path_name = "from_"+start + "_to_" + dest;
                outStreamItem << "\t" << path_name << "={" << "\n";
                outStreamItem << "\t\t" << Default_Item_Path_Start << "=" << start << "\n";
                outStreamItem << "\t\t" << Default_Item_Path_Destination << "=" << dest << "\n";
                outStreamItem << "\t\t" << Default_Item_Path_Limit_Speed_Translate << "=" << (*net)[j].list[k].prop.limit_translate;
                outStreamItem << "\t\t" << Default_Item_Path_Limit_Speed_Rotate << "=" << (*net)[j].list[k].prop.limit_rotate;
                outStreamItem << "\t" << "}" << "\n";
            }
        }
        outStreamItem << "}" << "\n";

        file.close();

        return 0;
		}
	}
} // <--- function definition


// ---> function definition
namespace gnd {
	namespace path {

		inline
    int fread(const char* fname, path_net_area_and_speed_limited *net)
    {
        const int GET_DEFAULT   = 0;
        const int GET_WAYPOINTS = 1;
        const int GET_LINEPATH  = 2;

        int data_type = GET_DEFAULT;
        QStack<QString> path_tmp;
        QString strLine;
        QStringList strList;
        waypoint_named_t wp_ws;
        uint data_cnt = 0;

        pathprop_area_and_speed_t lp_ws;
        QString start;
        QString dest;

        QFile file(fname);

        if(file.exists())
        {
            file.open(QIODevice::ReadOnly | QIODevice::Text);
        }
        else
        {
            return -1;
        }

        QTextStream itemStreamIn(&file);

        net->clear();

        while(!itemStreamIn.atEnd())
        {
            strLine = itemStreamIn.readLine().trimmed();
            if(strLine.isEmpty()) continue;

            //check data type
            if(strLine.contains(Default_Item_Waypoint))
            {
                if(path_tmp.isEmpty())
                {
                    //start to get waypoint
                    data_type = GET_WAYPOINTS;
                    if(strLine.contains("{"))
                    {
                        path_tmp.push("{");
                    }
                    continue;
                }
                else
                {
                    return -1;
                }
            }
            else if(strLine.contains(Default_Item_Path_List))
            {
                if(path_tmp.isEmpty())
                {
                    //start to get path
                    data_type = GET_LINEPATH;
                    if(strLine.contains("{"))
                    {
                        path_tmp.push("{");
                    }
                    continue;
                }
                else
                {
                    return -1;
                }
            }

            //save waypoints to buffer
            if(data_type == GET_WAYPOINTS)
            {
                if(strLine.contains("{"))
                {
                    path_tmp.push("{");
                    strList = strLine.split('=');
                    strcpy(wp_ws.name, strList.at(0).toStdString().c_str());
                }
                else if(strLine.contains("}"))
                {
                    if(path_tmp.pop().contains("{"))
                    {
                        //set to buffer
                        if(path_tmp.size() > 0)
                        {
                            net->add_waypoint(wp_ws.name, wp_ws.x, wp_ws.y);
                        }
                        else
                        {
                           //Do nothing, get waypoints complete
                        }
                    }
                    else
                    {
                        //path file format error
                        return -1;
                    }
                }
                else
                {
                    if(data_cnt == 0)
                    {
                        wp_ws.x = strLine.toDouble();
                        data_cnt++;
                    }
                    else
                    {
                        wp_ws.y = strLine.toDouble();
                        data_cnt = 0;
                    }
                }
            }

            //save linepath to buffer
            if(data_type == GET_LINEPATH)
            {
                if(strLine.contains("{"))
                {
                    path_tmp.push("{");
                    strList = strLine.split("=").at(0).split("_");
                    start = strList.at(1);
                    dest = strList.at(3);
                }
                else if(strLine.contains("}"))
                {
                    if(path_tmp.pop().contains("{"))
                    {
                        //set to buffer
                        if(path_tmp.size() > 0)
                        {
                            net->add_linepath(start.toStdString().c_str(),dest.toStdString().c_str(),&lp_ws);
                        }
                        else
                        {
                            //Do nothing, get linepath complete
                        }
                    }
                    else
                    {
                        //path file format error
                        return -1;
                    }
                }
                else
                {
                    //set items value: item = value
                    strList = strLine.split('=');
                    if(strLine.contains(Default_Item_Path_Limit_Speed_Translate))
                    {
                        lp_ws.limit_translate = strList.at(1).toDouble();
                    }
                    if(strLine.contains(Default_Item_Path_Limit_Speed_Rotate))
                    {
                        lp_ws.limit_rotate = strList.at(1).toDouble();
                    }
                    if(strLine.contains(Default_Item_Path_Start_Extend))
                    {
                        lp_ws.start_extend = strList.at(1).toDouble();
                    }
                    if(strLine.contains(Default_Item_Path_End_Extend))
                    {
                        lp_ws.end_extend = strList.at(1).toDouble();
                    }
                    if(strLine.contains(Default_Item_Path_Left_Width))
                    {
                        lp_ws.left_width = strList.at(1).toDouble();
                    }
                    if(strLine.contains(Default_Item_Path_Right_Width))
                    {
                        lp_ws.right_width = strList.at(1).toDouble();
                    }
                }
            }

        }

        return 0;
		}


		inline
    int fwrite(const char* fname, path_net_area_and_speed_limited *net)
    {
        QFile file(fname);
        file.open(QIODevice::NewOnly | QIODevice::WriteOnly | QIODevice::Text);

        QTextStream outStreamItem(&file);

        // write waypoint
        outStreamItem << Default_Item_Waypoint << "={" << "\n";
        for(int i = 0; i < net->n_waypoints(); i++)
        {
            outStreamItem << "\t" << (*net)[i].waypoint.name << "={" << "\n";
            outStreamItem << "\t\t" << (*net)[i].waypoint.x << "\n";
            outStreamItem << "\t\t" << (*net)[i].waypoint.y << "\n";
            outStreamItem << "\t" << "}" << "\n";
        }
        outStreamItem << "}" << "\n";

        // write path list
        QString start, dest, path_name;
        int j, k;

        outStreamItem << Default_Item_Path_List << "={" << "\n";
        for(j = 0; j < net->n_waypoints(); j++)
        {
            start = QString((*net)[j].waypoint.name);
            for(k = 0; k < (*net)[j].list.size(); k++)
            {
                dest = QString((*net)[j].list[k].end.name);
                path_name = "from_"+start + "_to_" + dest;
                outStreamItem << "\t" << path_name                                 << "={" << "\n";
                outStreamItem << "\t\t" << Default_Item_Path_Start                 << "="  << start << "\n";
                outStreamItem << "\t\t" << Default_Item_Path_Destination           << "="  << dest  << "\n";
                outStreamItem << "\t\t" << Default_Item_Path_Limit_Speed_Translate << "="  << (*net)[j].list[k].prop.limit_translate;
                outStreamItem << "\t\t" << Default_Item_Path_Limit_Speed_Rotate    << "="  << (*net)[j].list[k].prop.limit_rotate;
                outStreamItem << "\t\t" << Default_Item_Path_Start_Extend          << "="  << (*net)[j].list[k].prop.start_extend;
                outStreamItem << "\t\t" << Default_Item_Path_End_Extend            << "="  << (*net)[j].list[k].prop.end_extend;
                outStreamItem << "\t\t" << Default_Item_Path_Left_Width            << "="  << (*net)[j].list[k].prop.left_width;
                outStreamItem << "\t\t" << Default_Item_Path_Right_Width           << "="  << (*net)[j].list[k].prop.right_width;
                outStreamItem << "\t" << "}" << "\n";
            }
        }
        outStreamItem << "}" << "\n";

        file.close();

        return 0;
		}
	}
} // <--- function definition

#endif /* GND_PATH_IO_H_ */
