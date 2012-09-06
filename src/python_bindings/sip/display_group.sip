namespace rviz
{

class DisplayGroup: rviz::Display /NoDefaultCtors/
{
%TypeHeaderCode
#define ROS_PACKAGE_NAME "rviz"
#include <rviz/display_group.h>
%End

public:
  DisplayGroup();
  virtual ~DisplayGroup();

  rviz::Display* createDisplay( const QString& class_id );

  /** @brief Return the number of child objects (Property and Display).
   *
   * Overridden from Property to include the number of child Displays. */
  virtual int numChildren() const;

  /** @brief Return the child with the given index, without
   * checking whether the index is within bounds.
   *
   * Overridden from Property to include Display children. */
  virtual rviz::Property* childAtUnchecked( int index ) const;

  /** @brief Take a child out of the child list, but don't destroy it.
   * @return Returns the child property at the given index, or NULL if the index is out of bounds.
   *
   * This notifies the model about the removal.
   *
   * This is overridden from Property to include Display children. */
  virtual rviz::Property* takeChildAt( int index );

  /** @brief Add a child Property or Display.
   * @param child The child to add.
   * @param index [optional] The index at which to add the child.  If
   *   less than 0 or greater than the number of child properties, the
   *   child will be added at the end.
   *
   * This notifies the model about the addition.
   *
   * This is overridden from Property to keep non-Display child
   * Properties in Property's list of children and Display children in
   * DisplayGroup's list of child Displays. */
  virtual void addChild( rviz::Property* child, int index = -1 );

  /** @brief Return data appropriate for the given column (0 or 1) and
   * role for this DisplayGroup.
   */
  virtual QVariant getViewData( int column, int role ) const;

  /** @brief Return item flags appropriate for the given column (0 or
   * 1) for this DisplayGroup. */
  virtual Qt::ItemFlags getViewFlags( int column ) const;

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  /** @brief Add a child Display to the end of the list of Displays.
   *
   * This also tells the model that we are adding a child, so it can
   * update widgets.
   *
   * @note This does @e not remove @a child from its parent.  That
   * must be done first to avoid problems. */
  virtual void addDisplay( rviz::Display* child );

  /** @brief Remove a child Display from the the list of Displays, but
   *         don't destroy it.
   * @return Returns child if it is found, or NULL if child is not found.
   *
   * This also tells the model that we are removing a child, so it can
   * update widgets. */
  virtual rviz::Display* takeDisplay( rviz::Display* child );

  /** @brief Remove and destroy all child Displays, but preserve any
   * non-Display children. */
  virtual void removeAllDisplays();

  /** @brief Return the number of child Displays. */
  virtual int numDisplays() const;

  /** @brief Return the index-th Display in this group, or NULL if the
   * index is invalid. */
  virtual rviz::Display* getDisplayAt( int index ) const;

  /** @brief Find the index-th child Display in this group.  If the
   * child is itself a DisplayGroup, return the pointer to it.  If it
   * is not, return NULL. */
  virtual rviz::DisplayGroup* getGroupAt( int index ) const;

  /** @brief Call update() on all child Displays. */
  virtual void update( float wall_dt, float ros_dt );

  /** @brief Reset this and all child Displays. */
  virtual void reset();
};

};
