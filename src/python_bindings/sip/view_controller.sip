namespace rviz
{

class ViewController: rviz::Property /NoDefaultCtors/
{
%TypeHeaderCode
#define ROS_PACKAGE_NAME "rviz"
#include <rviz/view_controller.h>
%End

public:
//  ViewController();
//  virtual ~ViewController();

  /** @brief Do all setup that can't be done in the constructor.
   *
   * Creates camera_ and attaches it to the root scene node.
   *
   * Calls onInitialize() just before returning. */
//  void initialize( DisplayContext* context );

  static QString formatClassId( const QString& class_id );

  /** @brief Overridden from Property to give a different background
   * color and bold font if this view is active. */
  virtual QVariant getViewData( int column, int role ) const;

  /** @brief Overridden from Property to make this draggable if it is not active. */
  virtual Qt::ItemFlags getViewFlags( int column ) const;

  /** @brief Called by RenderPanel when this view controller is about to be used.
   *
   * There is no deactivate() because ViewControllers leaving
   * "current" are destroyed.  Put any cleanup in the destructor. */
  void activate();

  /** @brief Called at 30Hz by ViewManager::update() while this view
   * is active. Override with code that needs to run repeatedly. */
  virtual void update(float dt, float ros_dt);

//  virtual void handleMouseEvent(ViewportMouseEvent& evt);

  void lookAt( float x, float y, float z );

  /** Reset the view controller to some sane initial state, like
   * looking at 0,0,0 from a few meters away. */
  virtual void reset() = 0;

  /** @brief Configure the settings of this view controller to give,
   * as much as possible, a similar view as that given by the
   * @a source_view.
   *
   * @a source_view must return a valid @c Ogre::Camera* from getCamera().
   *
   * This base class implementation does nothing. */
  virtual void mimic( rviz::ViewController* source_view );

  /** @brief Called by ViewManager when this ViewController is being made current.
   * @param previous_view is the previous "current" view, and will not be NULL.
   *
   * This gives ViewController subclasses an opportunity to implement
   * a smooth transition from a previous viewpoint to the new
   * viewpoint.
   *
   * This base class implementation does nothing. */
  virtual void transitionFrom( rviz::ViewController* previous_view );

  /** @brief Subclasses should call this whenever a change is made which would change the results of toString(). */
  void emitConfigChanged();

//  Ogre::Camera* getCamera() const;

  /** @brief Return the class identifier which was used to create this
   * instance.  This version just returns whatever was set with
   * setClassId(). */
  virtual QString getClassId() const;

  /** @brief Set the class identifier used to create this instance.
   * Typically this will be set by the factory object which created it. */
  virtual void setClassId( const QString& class_id );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  bool isActive() const;

  /** @return A mouse cursor representing the current state */
  virtual QCursor getCursor();

signals:
  void configChanged();
};

};
