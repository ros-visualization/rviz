namespace rviz
{

class ViewManager: QObject /NoDefaultCtors/
{
%TypeHeaderCode
#define ROS_PACKAGE_NAME "rviz"
#include <rviz/view_manager.h>
%End

public:
//  ViewManager( DisplayContext* context );
//  ~ViewManager();

  void initialize();

  void update( float wall_dt, float ros_dt );

  /** @brief Return the current ViewController in use for the main
   * RenderWindow. */
  rviz::ViewController* getCurrent() const;

  rviz::ViewController* create( const QString& type );

  int getNumViews() const;

  rviz::ViewController* getViewAt( int index ) const;

  void add( rviz::ViewController* view, int index = -1 );

  /** @brief Remove the given ViewController from the list and return
   * it.  If it is not in the list, NULL is returned and nothing
   * changes. */
  rviz::ViewController* take( rviz::ViewController* view );

  /** @brief Remove the ViewController at the given index from the
   * list and return it.  If the index is not valid, NULL is returned
   * and nothing changes. */
  rviz::ViewController* takeAt( int index );

//  PropertyTreeModel* getPropertyModel() { return property_model_; }

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  /** @brief Make a copy of @a view_to_copy and install that as the new current ViewController. */
  void setCurrentFrom( rviz::ViewController* view_to_copy );

  /** @brief Return a copy of source, made by serializing source to
   * YAML and instantiating and loading a new one from that YAML. */
  rviz::ViewController* copy( rviz::ViewController* source );

//  PluginlibFactory<rviz::ViewController>* getFactory() const { return factory_; }

public slots:

  /** @brief Make a copy of the current ViewController and add it to the end of the list of saved views. */
  void copyCurrentToList();

  /** @brief Create a new view controller of the given type and set it
   * up to mimic and replace the previous current view. */
  void setCurrentViewControllerType( const QString& new_class_id );

signals:
  void configChanged();

  /** @brief Emitted just after the current view controller changes. */
  void currentChanged();
};

};
