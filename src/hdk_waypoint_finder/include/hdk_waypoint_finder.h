/*
 * @file hdk_waypoint_finder.hpp
 * @author Hidekazu TAKAHASHI
 * @date 2017/11/14
 *  *  *  Updated by ryu, 2023/10/18
 *  .Use Qt v5.14 lib to update config file
 */

#ifndef HDK_WAYPOINT_FINDER_H
#define HDK_WAYPOINT_FINDER_H

#include <QString>
#include <QtMath>
#include <QtGlobal>
#include <QGenericMatrix>

#include "hdk_waypoint_finder/srv_find_waypoint.h"
#include "hdk_waypoint_finder/srv_is_in_travelable_area.h"

#include "gnd-matrix-coordinate.h"
#include "gnd-coord-tree.h"
#include "hdk_waypoint_finder_config.h"
#include "gnd-path-io.h"

#include "gnd_msgs/msg_path_area_and_speed_limited.h"
#include "gnd_msgs/msg_pose2d_stamped.h"
#include "gnd_msgs/msg_velocity2d_with_covariance_stamped.h"
#include "gnd_msgs/msg_path_area_and_speed_limited.h"
#include "gnd_msgs/msg_vehicle_status.h"
#include "gnd_msgs/msg_trajectory.h"

namespace hdk {
	namespace waypoint_finder {

    typedef struct
    {
        double x;
        double y;
		} point2d_t;

    typedef struct
    {
        double x;
        double y;
        double theta;
		} pose2d_t;


		/**
		 * @brief 地図座標系の点を指定した経路座標系に変換する関数．
		 *        始点と終点で経路を指定するが，存在しないときは失敗し，dstは不定．
		 * @param[in] path_net パスネット
		 * @param[in] waypoint_name_start 経路の始点
		 * @param[in] waypoint_name_end 経路の終点
		 * @param[in] src 点（地図座標系）
		 * @param[out] dst 点（経路座標系）
		 * @return true 成功, false 失敗
		 */
		bool coordtf_map_to_path( gnd::path::path_net_area_and_speed_limited &path_net,
				const char *waypoint_name_start, const char *waypoint_name_end,
        const point2d_t &src, point2d_t &dst);

		/**
		 * @brief 指定した点が経路の中にあるか判定する関数．
		 * @param[in] path_net パスネット
		 * @param[in] waypoint_name_start 経路の始点
		 * @param[in] waypoint_name_end 経路の終点
		 * @param[in] x 位置（地図座標系）
		 * @param[in] y 位置（地図座標系）
		 * @param[in] expansion Configuration-spaceで膨張させる量
		 *                      ロボットの大きさ（ロボット座標系中心から最も離れている部分までの長さ）を設定するとよい
		 * @return true 存在する, flase それ以外
		 * @note パス自体が存在しない場合はfalseを返す．
		 */
		bool is_in_path( gnd::path::path_net_area_and_speed_limited &path_net,
				const char *waypoint_name_start, const char *waypoint_name_end,
				double x, double y, double expansion );


		/**
		 * @brief 指定した点が経路の中にあるか判定する関数．
		 * @param[in] path_net パスネット
		 * @param[in] x 位置（地図座標系）
		 * @param[in] y 位置（地図座標系）
		 * @param[in] expansion Configuration-spaceで膨張させる量
		 *                      ロボットの大きさ（ロボット座標系中心から最も離れている部分までの長さ）を設定するとよい
		 * @return true 存在する, flase それ以外
		 */
		bool is_in_path_net( gnd::path::path_net_area_and_speed_limited &path_net, double x, double y, double expansion );


		/**
		 * @brief 指定した位置に最も近い経路点(waypoint)を返す関数．
		 *        指定した位置が走行可能領域でないときは，失敗し，waypoint_nameは不定．
		 * @param[in] path_net パスネット
		 * @param[in] x 位置（地図座標系）
		 * @param[in] y 位置（地図座標系）
		 * @param[in] expansion Configuration-spaceで膨張させる量
		 *                      ロボットの大きさ（ロボット座標系中心から最も離れている部分までの長さ）を設定するとよい
		 * @param[in] waypoint_name_destination 目的地の名前
		 * @param[out] waypoint_name 最も近い経路点の名前
		 * @return true 成功, false 失敗
		 */
		bool find_waypoint( gnd::path::path_net_area_and_speed_limited &path_net,
				double x, double y, double expansion,
				const char *waypoint_name_destination, char *waypoint_name );

		/**
		 * @brief パスの長さ（道のり）を返す関数．
		 * @param[in] path
		 * @return パスの長さ
		 */
		double get_path_length(gnd::path::path_net_area_and_speed_limited::path_t &path);

	}
}

namespace hdk {
	namespace waypoint_finder {

		bool coordtf_map_to_path( gnd::path::path_net_area_and_speed_limited &path_net,
				const char *waypoint_name_start, const char *waypoint_name_end,
        const point2d_t &src, point2d_t &dst)
    {
        gnd::path::path_net_area_and_speed_limited::property_t prop;
        QGenericMatrix<4,1,double> point_on_map;
        QGenericMatrix<4,1,double> point_on_path;

        /* check if specified path exists */
        if (path_net.get_path_property( waypoint_name_start, waypoint_name_end, &prop ) < 0 )
        {
            /* there is not specified path */
            return false;
        }

        { // ---> coordinate transiform
            QGenericMatrix<4,4,double> mat_coordtf_map_to_path; // 変換行列
            point2d_t waypoint_start;         // 経路点（地図座標系）
            point2d_t waypoint_end;           // 経路点（地図座標系）
            point2d_t offset;                 // 経路座標系原点から見た地図座標系原点の位置
            double theta;                     // 経路座標系の角度（地図座標系）

            path_net.get_waypoint( waypoint_name_start, &waypoint_start.x, &waypoint_start.y );
            path_net.get_waypoint( waypoint_name_end, &waypoint_end.x, &waypoint_end.y );
            theta = std::atan2(waypoint_end.y - waypoint_start.y, waypoint_end.x - waypoint_start.x);

            { // ---> find offset
                offset.x = -( waypoint_end.x * std::cos(theta) + waypoint_end.y * std::sin(theta) );
                offset.y = -( -waypoint_end.x * std::sin(theta) + waypoint_end.y * std::cos(theta) );
            } // <--- find offset

            gnd::matrix::coordinate_converter( &mat_coordtf_map_to_path,
                offset.x, offset.y, 0.0,
                std::cos(-theta), std::sin(-theta), 0.0,
                0.0, 0.0, 1.0);

            { // ---> transform
                /* set source */
                point_on_map(0,0) = src.x;
                point_on_map(0,1) = src.y;
                point_on_map(0,2) = 0.0;
                point_on_map(0,3) = 1.0;
                /* transform  */
                point_on_path = point_on_map * mat_coordtf_map_to_path;
                /* set return value */
                dst.x = point_on_path(0,0);
                dst.y = point_on_path(0,1);
            } // <--- transform
        }
        return true;
		}

		bool is_in_path( gnd::path::path_net_area_and_speed_limited &path_net,
				const char *waypoint_name_start, const char *waypoint_name_end,
        double x, double y, double expansion )
    {
        gnd::path::path_net_area_and_speed_limited::property_t prop;

        /* check specified path exist */
        if (path_net.get_path_property( waypoint_name_start, waypoint_name_end, &prop ) < 0 )
        {
            /* there is not specified path */
            return false;
        }


        { // ---> coordinate transiform
            point2d_t waypoint_start;         // 経路点（地図座標系）
            point2d_t waypoint_start_on_path; // 経路点（経路座標系）
            point2d_t target;                 // 調べたい点（地図座標系）
            point2d_t target_on_path;         // 調べたい点（経路座標系）

            path_net.get_waypoint( waypoint_name_start, &waypoint_start.x, &waypoint_start.y );
            target.x = x;
            target.y = y;

            // transform
            coordtf_map_to_path( path_net, waypoint_name_start, waypoint_name_end, waypoint_start, waypoint_start_on_path );
            coordtf_map_to_path( path_net, waypoint_name_start, waypoint_name_end, target, target_on_path );

            // evaluate
            if ( target_on_path.x > prop.end_extend )
            {
                return false;
            }
            else if ( target_on_path.x < waypoint_start_on_path.x - prop.start_extend )
            {
                return false;
            }
            else if ( target_on_path.y > prop.left_width - expansion )
            {
                return false;
            }
            else if ( target_on_path.y < -prop.right_width + expansion )
            {
                return false;
            }
            else
            {
                return true;
            }
        } // <--- coordinate transiform
		}

    bool is_in_path_net( gnd::path::path_net_area_and_speed_limited &path_net, double x, double y, double expansion )
    {
        int i, j;
        char name_i[128];
        char name_j[128];

        // ---> scanning path
        for ( i = 0; i < path_net.n_waypoints(); i++ )
        {
            for ( j = i+1; j < path_net.n_waypoints(); j++ )
            {
                /* get name */
                path_net.name_waypoint(i, name_i);
                path_net.name_waypoint(j, name_j);
                /* check */
                /* 経路は方向別にプロパティを設定可能なため，双方向調べる */
                if ( is_in_path( path_net, name_i, name_j, x, y, expansion ) )
                {
                    return true;
                }
                if ( is_in_path( path_net, name_j, name_i, x, y, expansion ) )
                {
                    return true;
                }
            }
        } // <--- scanning path

        return false;
		}


		bool find_waypoint( gnd::path::path_net_area_and_speed_limited &path_net,
				double x, double y, double expansion,
        const char *waypoint_name_destination, char *waypoint_name )
    {
        int i, j;
        char name_i[128];
        char name_j[128];

        /* check destination */
        if ( path_net.index_waypoint(waypoint_name_destination) == -1 )
        {
            /* there is no specified waypoint */
            return false;
        }

        /* check area */
        if ( !is_in_path_net( path_net, x, y, expansion ) )
        {
            /* not in movable area */
            return false;
        }

        /* find */
        // ---> scanning path
        for ( i=0; i<path_net.n_waypoints(); i++ )
        {
            for ( j=i+1; j<path_net.n_waypoints(); j++ )
            {
                /* get name */
                path_net.name_waypoint(i, name_i);
                path_net.name_waypoint(j, name_j);
                /* check */
                /* 経路は方向別にプロパティを設定可能なため，双方向調べる */
                if ( is_in_path( path_net, name_i, name_j, x, y, expansion )
                    && is_in_path( path_net, name_i, name_j, x, y, expansion ) )
                {
                    /* i-> j経路, j->i経路の両方が有効 */
                    // compare
                    gnd::path::path_net_area_and_speed_limited::path_t ws_i;
                    gnd::path::path_net_area_and_speed_limited::path_t ws_j;
                    int ret_i;
                    int ret_j;
                    ret_i = path_net.find_path_dijkstra(&ws_i, name_i, waypoint_name_destination);
                    ret_j = path_net.find_path_dijkstra(&ws_j, name_j, waypoint_name_destination);

                    if ( (ret_i < 0) || (ret_j < 0) )
                    {
                        return false;
                    }
                    if ( get_path_length(ws_i) > get_path_length(ws_j) )
                    {
                        std::strncpy(waypoint_name, name_i, 128);
                        return true;
                    }
                    else
                    {
                        std::strncpy(waypoint_name, name_j, 128);
                        return true;
                    }
                }
                else if ( is_in_path( path_net, name_i, name_j, x, y, expansion ) )
                {
                    /* i->j経路のみが有効 */
                    std::strncpy(waypoint_name, name_i, 128);
                    return true;
                }
                else if ( is_in_path( path_net, name_j, name_i, x, y, expansion ) )
                {
                    /* j->i経路のみが有効 */
                    std::strncpy(waypoint_name, name_j, 128);
                    return true;
                }
            }
        } // <--- scanning path

        return true; // 到達しない．
		}

    double get_path_length(gnd::path::path_net_area_and_speed_limited::path_t &path)
    {
        double length = 0.0;
        if ( path.path.size() > 0 )
        {
            double dx;
            double dy;
            dx = path.path[0].end.x - path.start.x;
            dy = path.path[0].end.y - path.start.y;
            length += std::sqrt(dx*dx + dy*dy);

            for ( std::size_t i=1; i<path.path.size(); i++ )
            {
                double dx;
                double dy;
                dx = path.path[i].end.x - path.path[i-1].end.x;
                dy = path.path[i].end.y - path.path[i-1].end.y;
                length += std::sqrt(dx*dx + dy*dy);
            }
        }

        return length;
		}

	}
}

#endif //HDK_WAYPOINT_FINDER_H
