/*
 * gnd-path.h
 *
 *  Created on: 2011/08/09
 *      Author: tyamada
 *
 *  Updated by ryu, 2023/10/11
 *  .Use Qt v5.14 lib to update path information
 */
#ifndef GND_PATH_H_
#define GND_PATH_H_

#include <QString>
#include <QQueue>

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <float.h>
#include <cmath>
#include "gnd_rosutil.h"

// ---> type definition
namespace gnd
{
  namespace path
  {
      typedef int32_t seq_id_t;

      // type declaration
      struct waypoint_t;
      struct waypoint_directional_t;
      struct waypoint_named_t;
      struct waypoint_directional_named_t;

      struct trajectory_unit_t;
      struct trajectory_t;

      struct path_unit_t;
      template <class PathProp>
      struct path_unit_with_property_t;
      template <class PathProp>
      struct path_unit_list_with_property_t;
      template <class PathProp>
      struct path_with_property_t;
      // ---> definition of waypoint's data structure

      /*
       * \brief waypoint
       */
      typedef struct waypoint_t
      {
          double x;
          double y;
          waypoint_t& operator=(const waypoint_t& a);
          waypoint_t& operator=(const waypoint_directional_t& a);
          waypoint_t& operator=(const waypoint_named_t& a);
          waypoint_t& operator=(const waypoint_directional_named_t& a);
      } waypoint_t;

      typedef struct waypoint_directional_t : virtual public waypoint_t
      {
          double theta;
          waypoint_directional_t& operator=(const waypoint_directional_t& a);
          waypoint_directional_t& operator=(const waypoint_directional_named_t& a);
      } waypoint_directional_t;

      typedef struct waypoint_named_t : virtual public waypoint_t
      {
          char name[128];
          waypoint_named_t& operator=(const waypoint_named_t& a);
          waypoint_named_t& operator=(const waypoint_directional_named_t& a);
      } waypoint_named_t;

      typedef struct waypoint_directional_named_t :
          public waypoint_named_t, public waypoint_directional_t
      {
          waypoint_directional_named_t& operator=(const waypoint_directional_named_t& a);
      } waypoint_directional_named_t;
      // <--- definition of waypoint's data structure
      // ---> operator of waypoint
      inline
      waypoint_t& waypoint_t::operator=(const waypoint_t& a)
      {
          x = a.x;
          y = a.y;
          return *this;
      }
      inline
      waypoint_t& waypoint_t::operator=(const waypoint_directional_t& a)
      {
          x = a.x;
          y = a.y;
          return *this;
      }
      inline
      waypoint_t& waypoint_t::operator=(const waypoint_named_t& a)
      {
          x = a.x;
          y = a.y;
          return *this;
      }
      inline
      waypoint_t& waypoint_t::operator=(const waypoint_directional_named_t& a)
      {
          x = a.x;
          y = a.y;
          return *this;
      }
      inline
      waypoint_directional_t& waypoint_directional_t::operator=(const waypoint_directional_t& a)
      {
          x = a.x;
          y = a.y;
          theta = a.theta;
          return *this;
      }
      inline
      waypoint_directional_t& waypoint_directional_t::operator=(const waypoint_directional_named_t& a)
      {
          x = a.x;
          y = a.y;
          theta = a.theta;
          return *this;
      }

      inline
      waypoint_named_t& waypoint_named_t::operator=(const waypoint_named_t& a)
      {
          x = a.x;
          y = a.y;
          memcpy(name, a.name, sizeof(name));
          return *this;
      }

      inline
      waypoint_named_t& waypoint_named_t::operator=(const waypoint_directional_named_t& a)
      {
          x = a.x;
          y = a.y;
          memcpy(name, a.name, sizeof(name));
          return *this;
      }

      inline
      waypoint_directional_named_t& waypoint_directional_named_t::operator=(const waypoint_directional_named_t& a)
      {
          x = a.x;
          y = a.y;
          theta = a.theta;
          memcpy(name, a.name, sizeof(name));
          return *this;
      }
      // ---> operator of waypoint

      // ---> definition of trajectory's data structure
      struct trajectory_unit_t
      {
          waypoint_directional_t end;
          double curvature;
          trajectory_unit_t& operator=(const trajectory_unit_t& a);
          trajectory_unit_t& operator=(const path_unit_t& a);
          template <class PathProp>
          trajectory_unit_t& operator=(const path_unit_with_property_t<PathProp>& a);
      };
      struct trajectory_t
      {
          QQueue< trajectory_unit_t > lines;
          trajectory_t& operator=(const trajectory_t& a);
          template <class PathProp>
          trajectory_t& operator=(const path_with_property_t<PathProp>& a);
      };
      // <--- definition of trajectory's data structure


      // ---> definition of path's data structure
      typedef struct path_unit_t
      {
          waypoint_directional_named_t end;
          double curvature;
          path_unit_t& operator=(const path_unit_t& a);
          template <class PathProp>
          path_unit_t& operator=(const path_unit_with_property_t<PathProp>& a);
      } path_unit_t;

      template <class PathProp>
      struct path_unit_with_property_t : public path_unit_t
      {
          PathProp prop;
      };
      template <class PathProp>
      struct path_unit_list_with_property_t
      {
          waypoint_named_t waypoint;
          QQueue< path_unit_with_property_t<PathProp> > list;
          path_unit_list_with_property_t<PathProp>& operator=( const path_unit_list_with_property_t<PathProp>& a);
      };
      template <class PathProp>
      struct path_with_property_t
      {
          waypoint_named_t start;
          QQueue< path_unit_with_property_t<PathProp> > path;
          path_with_property_t<PathProp>& operator=( const path_with_property_t<PathProp>& a);
      };
      // <--- definition of path's data structure

      // ---> operator of path and trajectory
      inline
      trajectory_unit_t& trajectory_unit_t::operator=(const trajectory_unit_t& a)
      {
          end = a.end;
          curvature = a.curvature;
          return *this;
      }
      inline
      trajectory_unit_t& trajectory_unit_t::operator=(const path_unit_t& a)
      {
          end = a.end;
          curvature = a.curvature;
          return *this;
      }
      template <class PathProp>
      inline
      trajectory_unit_t& trajectory_unit_t::operator=(const path_unit_with_property_t<PathProp>& a)
      {
          end = a.end;
          curvature = a.curvature;
          return *this;
      }

      inline
      trajectory_t& trajectory_t::operator=(const trajectory_t& a)
      {
          lines.clear();
          for(uint i= 0; i < a.lines.size(); i++)
          {
              lines.push_back(a.lines.at(i));
          }
          return *this;
      }

      template <class PathProp>
      inline
      trajectory_t& trajectory_t::operator=(const path_with_property_t<PathProp>& a)
      {
          lines.clear();
          if(a.path.size() > 0 )
          {
              for(int i = 0; i < a.path.size(); i++ )
              {
                  lines.append(a.path[i]);
              }
          }
          return *this;
      }


      inline
      path_unit_t& path_unit_t::operator=(const path_unit_t& a)
      {
          end = a.end;
          curvature = a.curvature;
          return *this;
      }
      template <class PathProp>
      inline
      path_unit_t& path_unit_t::operator=(const path_unit_with_property_t<PathProp>& a)
      {
          end = a.end;
          curvature = a.curvature;
          return *this;
      }



      template <class PathProp>
      inline
      path_unit_list_with_property_t<PathProp>& path_unit_list_with_property_t<PathProp>::operator=( const path_unit_list_with_property_t<PathProp>& a)
      {
          memcpy(&waypoint, &a.waypoint, sizeof(waypoint));
          list.clear();

          for( int i = 0; i < (signed)a.list.size(); i++ )
          {
              list.append(a.list.at(i));
          }
          return *this;
      }
      template <class PathProp>
      inline
      path_with_property_t<PathProp>& path_with_property_t<PathProp>::operator=(const path_with_property_t<PathProp>& a)
      {
          memcpy(&start, &a.start, sizeof(start));
          path.clear();

          for( int i = 0; i < (signed)a.path.size(); i++)
          {
              path.append(a.path.at(i));
          }
          return *this;
      }
      // <--- operator of path and trajectory


      template <class PathProp>
      class path_net_with_property
      {
      public:
          typedef PathProp									                property_t;
          typedef path_unit_with_property_t<PathProp> 		  path_unit_t;
          typedef path_unit_t* 								              path_unit_pt;
          typedef const path_unit_t* 							          const_path_unit_pt;
          typedef path_unit_list_with_property_t<PathProp>	path_unit_list_t;
          typedef path_with_property_t<PathProp>				    path_t;
          typedef waypoint_named_t							            waypoint_t;
      private:
          QQueue< path_unit_list_t >                        net_;

      public:
          path_net_with_property(){}

      public:
          int clear();

      public:
          int n_waypoints();

      public:
          int add_waypoint(const char *name, double x, double y);
          int erase_waypoint(const char *name);
          int set_waypoint(const char *name, double x, double y);
          int get_waypoint(const char *name, double *x, double *y);
          int rename_waypoint(const char *name, const char *new_name);
      public:
          int add_linepath(const char *from, const char *to, PathProp *prop);
          int add_linepath_bidirectional(const char *a, const char *b, PathProp *prop);
          int erase_path(const char *from, const char *to);
          int erase_path_bidirectional(const char *a, const char *b);
          int set_path_property(const char *from, const char *to, PathProp *prop);
          int get_path_property(const char *from, const char *to, PathProp *prop);

      public:
          int name_waypoint(int index, char *name) const;
          int index_waypoint( const char *name ) const;

      public:
          int copy_path_unit(path_unit_with_property_t<PathProp> *dest, const char *from, const char *to);
          int copy_path_list(path_with_property_t<PathProp> *dest, const char *waypoint);
          const path_unit_with_property_t<PathProp>* const_pointer_path_unit(const char *from, const char *to);
          const path_with_property_t<PathProp>* cosnt_pointer_path_list( const char *waypoint ) const;
          path_unit_with_property_t<PathProp>* pointer_path_unit(const char *from, const char *to);
          path_unit_list_with_property_t<PathProp>* pointer_path_list( const char *waypoint );

      public:
          int find_path_dijkstra( path_with_property_t<PathProp>* p, const char *from, const char *to);

      public:
          path_unit_list_with_property_t<PathProp>& operator[](int i);
      };


      typedef struct
      {
          double limit_translate;
          double limit_rotate;
      } pathprop_speed_t;

      typedef struct
      {
          double limit_translate;
          double limit_rotate;
          double left_width;
          double right_width;
          double start_extend;
          double end_extend;
      } pathprop_area_and_speed_t;

      typedef path_net_with_property< pathprop_speed_t > 				    path_net_speed_limited;
      typedef path_net_speed_limited::path_unit_t 					        path_unit_speed_limited;
      typedef path_net_speed_limited::path_t 							          path_speed_limited;

      typedef path_net_with_property< pathprop_area_and_speed_t >		path_net_area_and_speed_limited;
      typedef path_net_area_and_speed_limited::path_unit_t	 		    path_unit_area_and_speed_limited;
      typedef path_net_area_and_speed_limited::path_t					      path_area_and_speed_limited;
    }
} // <--- type definition

namespace gnd
{
  namespace path
  {

		// ---> route class member function
    template <class PathProp>
    inline
    int path_net_with_property<PathProp>::clear()
    {
        net_.clear();
        return 0;
    }

    template <class PathProp>
    inline
    int path_net_with_property<PathProp>::n_waypoints()
    {
        return net_.size();
    }

    template <class PathProp>
    inline
    int path_net_with_property<PathProp>::add_waypoint(const char *name, double x, double y)
    {
        path_unit_list_with_property_t<PathProp> wap;

        // exception: already existed
        if( index_waypoint(name) >= 0 )  return -1;

        { // ---> set
            strcpy(wap.waypoint.name, name);
            wap.waypoint.x = x;
            wap.waypoint.y = y;
            wap.list.clear();
            net_.push_back(wap);
        } // <--- set

        return n_waypoints();
    }


    template <class PathProp>
    inline
    int path_net_with_property<PathProp>::erase_waypoint(const char *name)
    {
        int i, j;
        // exception: not existing
        if( (i = index_waypoint(name)) < 0 )  return -1;

        // erase waypoint
        net_.erase(i);
        // ease path to the waypoint
        for( i = 0; i < (signed)net_.size(); i++ )
        {
            for( j = 0; j < (signed)net_[i].list.size(); j++)
            {
                if( strcmp(net_[i].list[j].end.name, name) == 0 )
                {
                    net_[i].list.erase(j);
                }
            }
        }

        return n_waypoints();
    }


    template <class PathProp>
    inline
    int path_net_with_property<PathProp>::set_waypoint(const char *name, double x, double y)
    {
        int i, j;
        path_unit_pt p;
        // exception: not existing
        if( (i = index_waypoint(name)) < 0 )  return -1;

        net_[i].waypoint.x = x;
        net_[i].waypoint.y = y;

        // set path direction
        for( j = 0; j < (signed)net_[i].list.size(); j++ )
        {
            net_[i].list[j].end.theta = atan2( net_[i].list[j].end.y - net_[i].waypoint.y, net_[i].list[j].end.x - net_[i].waypoint.x );
        }

        // set path end
        for( j = 0; j < (signed)net_.size(); j++ )
        {
            if( (p = pointer_path_unit( net_[j].waypoint.name, name )) )
            {
                p->end.x = x;
                p->end.y = y;
                p->end.theta = atan2( p->end.y - net_[j].waypoint.y, p->end.x - net_[j].waypoint.x );
            }
        }

        return i;
    }

    template <class PathProp>
    inline
    int path_net_with_property<PathProp>::get_waypoint(const char *name, double *x, double *y)
    {
        int i;
        // exception: not existing
        if( (i = index_waypoint(name)) < 0 )  return -1;

        if(x) *x = net_[i].waypoint.x;
        if(y) *y = net_[i].waypoint.y;
        return i;
    }

    template <class PathProp>
    inline
    int path_net_with_property<PathProp>::rename_waypoint(const char *name, const char *new_name)
    {
        int i, j;
        // exception: not existing
        if( (i = index_waypoint(name)) < 0 )  return -1;

        strcpy(net_[i].waypoint.name, new_name);

        // ease path to the waypoint
        for( i = 0; i < (signed)net_.size(); i++ )
        {
            for( j = 0; j < (signed)net_[i].list.size(); j++)
            {
                if( strcmp(net_[i].list[j].end.name, name) == 0 )
                {
                    strcpy(net_[i].list[j].end.name, new_name);
                    break;
                }
            }
        }
        return i;
    }

    template <class PathProp>
    inline
    int path_net_with_property<PathProp>::add_linepath(const char *from, const char *to, PathProp *prop)
    {
        int i, f, t;
        path_unit_with_property_t<PathProp> p;

        if( (f = index_waypoint(from) ) < 0 )  return -1;
        if( (t = index_waypoint(to) ) < 0 )  return -1;
        // already exist
        for( i = 0; i < (signed)net_[f].list.size(); i++ )
        {
            if( strcmp(net_[t].waypoint.name, net_[f].list[i].end.name) == 0 )
            {
                return -1;
            }
        }

        p.end.x = net_[t].waypoint.x;
        p.end.y = net_[t].waypoint.y;
        p.end.theta = atan2( p.end.y - net_[f].waypoint.y, p.end.x - net_[f].waypoint.x );
        strcpy( p.end.name, net_[t].waypoint.name);
        p.curvature = 0;
        memcpy(&p.prop, prop, sizeof(PathProp));

        net_[f].list.push_back(p);

        return 0;
    }

    template <class PathProp>
    inline
    int path_net_with_property<PathProp>::add_linepath_bidirectional(const char *a, const char *b, PathProp *prop)
    {
        if( add_linepath(a,b,prop) < 0 )	return -1;
        if( add_linepath(b,a,prop) < 0 )	return -1;
        return 0;
    }

    template <class PathProp>
    inline
    int path_net_with_property<PathProp>::erase_path(const char *from, const char *to)
    {
        int f, t;
        int i;

        if( (f = index_waypoint(from) ) < 0 || net_[f].waypoint.name[0] == '\0' )  return -1;
        if( (t = index_waypoint(to) ) < 0 || net_[t].waypoint.name[0] == '\0' )  return -1;

        for( i = 0; i < (signed)net_[f].list.size(); i++ )
        {
            if( strcmp(net_[f].list[i].end.name, net_[t].waypoint.name) == 0 )
            {
                net_[f].list.erase(i);
            }
        }
        return 0;
    }
    template <class PathProp>
    inline
    int path_net_with_property<PathProp>::erase_path_bidirectional(const char *a, const char *b)
    {
        erase_path(a, b);
        erase_path(b, a);
        return 0;
    }

    template <class PathProp>
    inline
    int path_net_with_property<PathProp>::get_path_property(const char *from, const char *to, PathProp *prop)
    {
        path_unit_t *pu;
        if( (pu = pointer_path_unit(from, to)) == 0 ) return -1;
        *prop = pu->prop;
        return 0;
    }

    template <class PathProp>
    inline
    int path_net_with_property<PathProp>::set_path_property(const char *from, const char *to, PathProp *prop)
    {
        path_unit_t *pu;
        if( (pu = pointer_path_unit(from, to)) == 0 ) return -1;
        pu->prop = *prop;
        return 0;
    }


    template <class PathProp>
    inline
    int path_net_with_property<PathProp>::name_waypoint(int index, char *name) const
    {
        if( index < 0 || index >= net_.size() ) return -1;
        if( net_[0 + index].waypoint.name[0] == '\0' ) return -1;

        strcpy(name, net_[0 + index].waypoint.name);
        return 0;
    }


    template <class PathProp>
    inline
    int path_net_with_property<PathProp>::index_waypoint( const char *name_waypoint ) const
    {
        int i;

        for( i = 0; i < (signed)net_.size(); i++ )
        {
            if( net_[i].waypoint.name[0] == '\0' ) continue;
            if( strcmp( net_[i].waypoint.name, name_waypoint ) == 0  ) break;
        }
        if( i >= (signed)net_.size() )
        {
            return -1;
        }

        return i;
    }

    template <class PathProp>
    inline
    int path_net_with_property<PathProp>::copy_path_unit(path_unit_with_property_t<PathProp> *dest, const char *from, const char *to)
    {
        const path_unit_with_property_t<PathProp> *p;
        if( !(p = const_pointer_path_unit(from, to)) )
        {
            return -1;
        }

        memcpy(dest, p, sizeof(path_unit_with_property_t<PathProp>) );
        return 0;
    }


    template <class PathProp>
    inline
    int path_net_with_property<PathProp>::copy_path_list(path_with_property_t<PathProp> *dest, const char *waypoint)
    {
        const path_with_property_t<PathProp> *p;
        if( !(p = cosnt_pointer_path_list(waypoint)) )
        {
            return -1;
        }

        *dest = *p;
        return 0;
    }

    template <class PathProp>
    inline
    const path_with_property_t<PathProp>* path_net_with_property<PathProp>::cosnt_pointer_path_list( const char *waypoint ) const
    {
        int i;
        if ( (i = index_waypoint(waypoint)) < 0) return 0;
        return &net_[i];

    }

    template <class PathProp>
    inline
    const path_unit_with_property_t<PathProp>* path_net_with_property<PathProp>::const_pointer_path_unit(const char *from, const char *to)
    {
        int i, j;
        if ( (i = index_waypoint(from)) < 0) return 0;

        for( j = 0; j < (signed)net_[i].list.size(); j++ )
        {
            if( strcmp( net_[i].list[j].end.name, to) == 0 ) break;
        }
        if( j >= (signed) net_[i].list.size() )
        {
            return 0;
        }
        return &net_[i].list[j];
    }

    template <class PathProp>
    inline
    path_unit_list_with_property_t<PathProp>* path_net_with_property<PathProp>::pointer_path_list( const char *waypoint )
    {
        int i;
        if ( (i = index_waypoint(waypoint)) < 0) return 0;
        return &net_[i];

    }

    template <class PathProp>
    inline
    path_unit_with_property_t<PathProp>* path_net_with_property<PathProp>::pointer_path_unit(const char *from, const char *to)
    {
        int i, j;
        if ( (i = index_waypoint(from)) < 0) return 0;

        for( j = 0; j < (signed)net_[i].list.size(); j++ )
        {
            if( strcmp( net_[i].list[j].end.name, to) == 0 ) break;
        }
        if( j >= (signed) net_[i].list.size() )
        {
            return 0;
        }
        return &net_[i].list[j];
    }


    // ---> search path
    template <class PathProp>
    inline
    int path_net_with_property<PathProp>::find_path_dijkstra( path_with_property_t<PathProp>* p, const char *start, const char *dest)
    {
        int i_start;
        int i_dest;
        int i;
        struct node_t
        {
            double cost;
            bool done;
            QQueue< int > path;
        };
        QQueue< node_t > costs;
        node_t ws;

        if( (i_start = index_waypoint(start)) < 0 )	return -1;
        if( (i_dest = index_waypoint(dest)) < 0 )	return -1;

        { // ---> initialize cost
            for( i = 0; i < (signed)costs.size(); i++ )
            {
                ws.cost = DBL_MAX;
                ws.done = false;
                ws.path.clear();
                costs.append(ws);
            }
            // at start, cost is 0
            costs[i_start].cost = 0;
            costs[i_start].path.push_back(i_start);
        } // <--- initialize cost


        { // ---> search

            // ---> scanning loop
            while(1)
            {
                int i_min = -1;
                { // ---> get minimum cost node
                    double min;

                    min = DBL_MAX;
                    for( i = 0; i < (signed)costs.size(); i++ )
                    {
                        if( !costs[i].done && costs[i].cost < min )
                        {
                            min = costs[i].cost;
                            i_min = i;
                        }
                    }

                    // exception: search all waypoint connecting from start
                    if( i_min == -1)
                    {
                        return -1;
                    }
                    // exception:  reach to destination
                    else if( i_min == i_dest )
                    {
                        // finish
                        costs[i_min].path.push_back( i_min );
                        break;
                    }
                } // <--- get minimum cost node


                { // ---> update nodes cost
                    double cost_;
                    for( i = 0; i < (signed)net_[i_min].list.size(); i++ )
                    {
                        int index_ = index_waypoint(net_[i_min].list[i].end.name);
                        if( costs[index_].done ) continue;
                        cost_ = costs[i_min].cost + \
                            sqrt( gnd_square( net_[i_min].list[i].end.x - net_[i_min].waypoint.x ) + gnd_square( net_[i_min].list[i].end.y - net_[i_min].waypoint.y ) );

                        // update
                        if( cost_ < costs[index_].cost )
                        {
                            costs[index_].cost = cost_;
                            costs[index_].path.clear();
                            for(uint j = 0; j < costs[i_min].path.size(); j++)
                            {
                                costs[index_].path.push_back(costs[i_min].path.at(j));

                            }
                            costs[index_].path.push_back( i_min );
                        }
                    }
                } // <--- update nodes cost

                costs[i_min].done = true;
            } // <--- scanning loop
        } // <--- search

        { // ---> packing
            int j = costs[i_dest].path[0];
            path_unit_with_property_t<PathProp>* path_unit_tmp;

            { // ---> set start
                p->start = net_[j].waypoint;
            } // <--- set start

            // set path
            p->path.clear();
            for( i = 1; i < (signed)costs[i_dest].path.size(); i++ )
            {
                if(strcmp(net_[ costs[i_dest].path[i-1] ].waypoint.name,net_[ costs[i_dest].path[i] ].waypoint.name) == 0)
                {
                    continue;
                }
//                p->path.push_back( const_pointer_path_unit( net_[ costs[i_dest].path[i-1] ].waypoint.name,  net_[ costs[i_dest].path[i] ].waypoint.name ) );
//                path_unit_tmp = pointer_path_unit( net_[ costs[i_dest].path[i-1] ].waypoint.name,  net_[ costs[i_dest].path[i] ].waypoint.name );
                p->path.push_back( *pointer_path_unit( net_[ costs[i_dest].path[i-1] ].waypoint.name,  net_[ costs[i_dest].path[i] ].waypoint.name ) );

            }
        } // <--- packing

        return 0;
    }
    // <--- search path

    template <class PathProp>
    inline
    path_unit_list_with_property_t<PathProp>& path_net_with_property<PathProp>::operator[](int i)
    {
        return net_[i];
    }

  } // <--- function definition (path_net_with_propert)

} // <--- function definition


#endif

