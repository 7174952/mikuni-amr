/****************************************
 * gnd-coord-tree.h
 *
 *  Created on: 2011/06/16
 *      Author: tyamada
 *
 *  Updated by ryu, 2023/04/21
 *  .Use Qt v5.14 lib to instead of original Matrix class
 *
 ***************************************/

#ifndef GND_COORD_TREE_H
#define GND_COORD_TREE_H

#include <QMatrix4x4>
#include <QGenericMatrix>
#include <QQueue>
#include <QString>
#include "gnd-matrix-coordinate.h"

namespace gnd
{
/**
 * @brief node of coordinate tree
 */
//class coord_node : public QMatrix4x4
class coord_node: public QGenericMatrix<4,4,double>
{
public:
    int id;                               ///< cooridnate id
    int depth;                            ///< tree depth
    int parent;                           ///< parent node index
    QString name;

public:
    // ---> constructor
    coord_node();
    coord_node( const QGenericMatrix<4,4,double>* c );
    coord_node( const coord_node* c );
    // <--- constructor
    ~coord_node();
};

/**
 * @brief coordinate tree
 */
class coord_tree
{
// ---> constructor, destructor
public:
    coord_tree();
    ~coord_tree();

    // ---> initial
public:
    void initial();

    // ---> constant definition
public:
    static const int InvalidID = -1;
    static const int InvalidDepth = -1;
    static const int InvalidParent = -1;

    // ---> typedef
public:
    typedef coord_node					node_t;		///< coorinate tree node type
    typedef QQueue<node_t>			tree_t;	  ///< coorinate tree storage

    // ---> node list
private:
    tree_t _tree;                         ///< coorinate tree
    int _id;                              ///< issued id No

    // ---> create coordinate
private:
    int create_root(node_t *rt);
public:
    static const int RootID = 0;          ///< coorinate tree root id
    static const int RootDepth = 0;       ///< coorinate tree root depth
    // ---> setter
public:
    int add_node( const QString &n, const QString &p, const QGenericMatrix<4,4,double> *c );
    int add_node( const QString &n, const int p, const QGenericMatrix<4,4,double> *c );

    // ---> set_mapping
public:
    // setter
    int set_coordinate( const int key, const QGenericMatrix<4,4,double> *o );
    int set_coordinate( const QString &key, const QGenericMatrix<4,4,double> *o );

    // ---> find node
public:
    int find(const QString &n);
    int find(const int id);

    // ---> get
public:
    template< typename T1, typename T2>
    int get_convert_matrix(const T1& f, const T2& t, QGenericMatrix<4,4,double> *m );

    int get_node_id(const QString &n);
};
        // <--- type declaration
} //namespace gnd

namespace gnd
{
/**
 * @brief get cooridnate convert matrix
 * @param  [in] f : from (name)
 * @param  [in] t : to   (name)
 * @param [out] m : coordinate convert matrix from f to t
 * @return    < 0 : fail to find node(f or t)
 */
template< typename T1, typename T2>
int coord_tree::get_convert_matrix(const T1& f, const T2& t, QGenericMatrix<4,4,double> *m )
{
    QGenericMatrix<4,4,double> fconv;
    QGenericMatrix<4,4,double> tconv;
    QGenericMatrix<4,4,double> ws;                //work space
    int fi, ti;                   // traversal node index (starting form "f" and "t")

    // find from-cooridnate
    fi = find( f );
    if( fi < 0 )
    {
        return -1;
    }
    // find to-coordinate
    ti = find( t );
    if( ti < 0 )
    {
        return -1;
    }

    // ---> initialize
    fconv.setToIdentity();
    tconv.setToIdentity();

    // ---> tree traversal loop
    // traverse the parent node of both "t" and "f"
    // until traversal nodes starting from both "t" and "f" is identical
    while(_tree[fi].id != _tree[ti].id)
    {
        // check depth
        int depth = _tree[fi].depth > _tree[ti].depth ? _tree[fi].depth : _tree[ti].depth;

        if( _tree[fi].depth >= depth )
        {
            // add to coordinate convert matrix
            ws = fconv;
            fconv = _tree[fi] * ws;

            // get parent node index
            fi = _tree[fi].parent;
        }

        if( _tree[ti].depth >= depth )
        {
            // add to coordinate convert matrix
            ws = tconv;
            tconv = _tree[ti] * ws;
            // get parent node index
            ti = _tree[ti].parent;
        }
    } // ---> tree traversal loop

    // ---> invert matrix
    ws = tconv;
    matrix::_inverse_4x4_(&ws, &tconv);

    // ---> fuse
    *m = tconv * fconv;

    return 0;
}

} //namespace gnd

namespace gnd {
/**
 * @brief constructor
 */
coord_node::coord_node()
  : id(coord_tree::InvalidID), depth(coord_tree::InvalidDepth), parent(coord_tree::InvalidParent)
{
    name.clear();
}

/*
 * @brief copy constructor
 */
coord_node::coord_node( const QGenericMatrix<4,4,double>* c )
  : id(coord_tree::InvalidID), depth(coord_tree::InvalidDepth), parent(coord_tree::InvalidParent)
{
    name.clear();
    c->transposed().copyDataTo(this->data());
}

/*
 * @brief copy constructor
 */
coord_node::coord_node( const coord_node* c )
  : id(coord_tree::InvalidID), depth(coord_tree::InvalidDepth), parent(coord_tree::InvalidParent)
{
    name.clear();
    c->transposed().copyDataTo(this->data());
}

/**
 * @brief destructor
 */
coord_node::~coord_node()
{

}

// ---> constructor, destructor
/**
 * @brief constructor
 */
coord_tree::coord_tree()
{
    initial();
}

/**
 * @brief destructor
 */
coord_tree::~coord_tree()
{

}

/**
 * @brief initialize
 */
void coord_tree::initial()
{
    node_t root;

    // issue id to root node
    _id = RootID;
    root.id = _id;

    // set name
    root.name = "root";
    root.parent = InvalidParent;
    root.depth = RootDepth;
    // set root coordinate matrix(unit matirx)
    root.setToIdentity();
    // put in cooridnate tree
    _tree.push_back( root );
}

/**
 * @brief add coordinate node
 * @param [in] n : added coordinate name
 * @param [in] p : parent node name
 * @param [in] o : added cooridnate matrix
 * @return    <0 : fail to node addition
 *           >=0 : added node id
 */
int coord_tree::add_node( const QString &n, const QString &p, const QGenericMatrix<4,4,double> *o )
{
    node_t node;

    // ---> set parent
    int paridx;	// parent index
    if( (paridx = find(p)) < 0)
    {
        return -1;
    }
    node.parent = paridx;
    node.depth = _tree[paridx].depth + 1;

    // ---> set coordinate convert mantrix
    o->transposed().copyDataTo(node.data());

    // ---> set identification
    node.name = n;
    node.id = ++_id;

    // ---> set in tree
    _tree.push_back(node);

    return _id;
}

/**
 * @brief add coordinate node
 * @param [in] n : added coordinate name
 * @param [in] p : parent node id
 * @param [in] o : added cooridnate matrix
 * @return    <0 : fail to node addition
 *           >=0 : added node id
 */
int coord_tree::add_node( const QString &n, const int p, const QGenericMatrix<4,4,double> *o )
{
    node_t node;

    // ---> set parent
    int paridx;	// parent index
    if( (paridx = find(p)) < 0)
    {
        return -1;
    }
    node.parent = paridx;
    node.depth = _tree[paridx].depth + 1;

    // ---> set coordinate convert mantrix
    o->transposed().copyDataTo(node.data());

    // ---> set identification
    node.name = n;
    node.id = ++_id;

    // ---> set in tree
    _tree.push_back(node);

    return _id;
}

/**
 * @brief set coordinate
 * @param [in] p : node id
 * @param [in] o : cooridnate matrix
 * @return    <0 : fail to find node
 */
int coord_tree::set_coordinate( const int id, const QGenericMatrix<4,4,double> *o )
{
    int paridx;
    if( (paridx = find(id)) < 0 )
    {
      return -1;
    }
    // set coordinate matrix
    o->transposed().copyDataTo(_tree[paridx].data());

    return 0;
}

/**
 * @brief set coordinate
 * @param [in] n : node name
 * @param [in] o : cooridnate matrix
 * @return    <0 : fail to find node
 */
int coord_tree::set_coordinate( const QString &n, const QGenericMatrix<4,4,double> *o )
{
    int paridx;
    if( (paridx = find(n)) < 0 )
    {
        return -1;
    }
    // set coordinate matrix
    o->transposed().copyDataTo(_tree[paridx].data());

    return 0;
}

/**
 * @brief find coordinate node
 * @param [in] n : node name
 * @return   >=0 : node index
 * @return    <0 : fail to find node
 */
int coord_tree::find(const QString &n)
{
    uint64_t i;
    for( i = 0; i < _tree.size(); i++)
    {
        if( (_tree[i].id != InvalidID) && (_tree[i].name.compare(n) == 0) )	return i;
    }
    return -1;
}

/**
 * @brief find coordinate node
 * @param [in] id : node id
 * @return    >=0 : node index
 * @return     <0 : fail to find node
 */
int coord_tree::find(const int id)
{
    uint64_t i;
    for( i = 0; i < _tree.size(); i++)
    {
        if( _tree[i].id == id )	return i;
    }
    return -1;
}

/**
 * @brief get node id
 * @param [in] n : name
 */
int coord_tree::get_node_id(const QString &n)
{
    int i = find(n);
    if( i < 0)	return -1;

    return _tree[i].id;
}

} //End of namespace

#endif /* GND_COORD_TREE_H */
