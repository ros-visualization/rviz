namespace rviz
{

class VisualizationManager: QObject /NoDefaultCtors/
{
%TypeHeaderCode
#define ROS_PACKAGE_NAME "rviz"
#include <rviz/visualization_manager.h>
%End

public:
  /**
   * \brief Constructor
   * Creates managers and sets up global properties.
   * @param render_panel a pointer to the main render panel widget of the app.
   * @param wm a pointer to the window manager (which is really just a
   *        VisualizationFrame, the top-level container widget of rviz).
   */
//  VisualizationManager( RenderPanel* render_panel, WindowManagerInterface* wm = 0 );

  /**
   * \brief Destructor
   * Stops update timers and destroys all displays, tools, and managers.
   */
  virtual ~VisualizationManager();

  /**
   * \brief Do initialization that wasn't done in constructor.
   * Initializes tool manager, view manager, selection manager.
   */
  void initialize();

  /**
   * \brief Start timers.
   * Creates and starts the update and idle timers, both set to 30Hz (33ms).
   */
  void startUpdate();

  /*
   * \brief Stop the update timers. No Displays will be updated and no ROS
   *        callbacks will be called during this period.
   */
  void stopUpdate();

  /**
   * \brief Create and add a display to this panel, by class lookup name
   * @param class_lookup_name "lookup name" of the Display subclass, for pluginlib.
   *        Should be of the form "packagename/displaynameofclass", like "rviz/Image".
   * @param name The name of this display instance shown on the GUI, like "Left arm camera".
   * @param enabled Whether to start enabled
   * @return A pointer to the new display.
   */
  rviz::Display* createDisplay( const QString& class_lookup_name, const QString& name, bool enabled );

  /**
   * \brief Add a display to be managed by this panel
   * @param display The display to be added
   */
  void addDisplay( rviz::Display* display, bool enabled );

  /**
   * \brief Remove and delete all displays
   */
  void removeAllDisplays();

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  /** @brief Return the fixed frame name.
   * @sa setFixedFrame() */
  QString getFixedFrame() const;

  /** @brief Set the coordinate frame we should be transforming all fixed data into.
   * @param frame The name of the frame -- must match the frame name broadcast to libTF
   * @sa getFixedFrame() */
  void setFixedFrame( const QString& frame );
  
  /**
   * @brief Convenience function: returns getFrameManager()->getTFClient().
   */
//  tf::TransformListener* getTFClient() const;

  /**
   * @brief Returns the Ogre::SceneManager used for the main RenderPanel.
   */
//  Ogre::SceneManager* getSceneManager() const;

  /**
   * @brief Return the main RenderPanel.
   */
//  RenderPanel* getRenderPanel() const;

  /**
   * @brief Return the wall clock time, in seconds since 1970.
   */
  double getWallClock();

  /**
   * @brief Return the ROS time, in seconds.
   */
  double getROSTime();

  /**
   * @brief Return the wall clock time in seconds since the last reset.
   */
  double getWallClockElapsed();

  /**
   * @brief Return the ROS time in seconds since the last reset.
   */
  double getROSTimeElapsed();

  /**
   * @brief Handle a single key event for a given RenderPanel.
   *
   * If the key is Escape, switches to the default Tool (via
   * getDefaultTool()).  All other key events are passed to the
   * current Tool (via getCurrentTool()).
   */
//  void handleChar( QKeyEvent* event, RenderPanel* panel );

  /**
   * @brief Handle a mouse event.
   *
   * This just copies the given event into an event queue.  The events
   * in the queue are processed by onUpdate() which is called from the
   * main thread by a timer every 33ms.
   */
//  void handleMouseEvent( const ViewportMouseEvent& event );

  /**
   * @brief Resets the wall and ROS elapsed time to zero and calls resetDisplays().
   */
  void resetTime();

  /**
   * @brief Return a pointer to the SelectionManager.
   */
//  SelectionManager* getSelectionManager() const;

  /** @brief Return a pointer to the ToolManager. */
  virtual rviz::ToolManager* getToolManager() const;

  /** @brief Return a pointer to the ViewManager. */
  virtual rviz::ViewManager* getViewManager() const;

  /**
   * @brief Lock a mutex to delay calls to Ogre::Root::renderOneFrame().
   */
  void lockRender();

  /**
   * @brief Unlock a mutex, allowing calls to Ogre::Root::renderOneFrame().
   */
  void unlockRender();

  /**
   * \brief Queues a render.  Multiple calls before a render happens will only cause a single render.
   * \note This function can be called from any thread.
   */
  void queueRender();

  /**
   * @brief Return the window manager, if any.
   */
//  WindowManagerInterface* getWindowManager() const;

  /**
   * @brief Return the CallbackQueue using the main GUI thread.
   */
//  ros::CallbackQueueInterface* getUpdateQueue();

  /**
   * @brief Return a CallbackQueue using a different thread than the main GUI one.
   */
//  ros::CallbackQueueInterface* getThreadedQueue();

  /** @brief Return the FrameManager instance. */
//  FrameManager* getFrameManager() const;

  /** @brief Return the current value of the frame count.
   *
   * The frame count is just a number which increments each time a
   * frame is rendered.  This lets clients check if a new frame has
   * been rendered since the last time they did something. */
  unsigned long long getFrameCount() const;

  /** @brief Notify this VisualizationManager that something about its
   * display configuration has changed. */
  void notifyConfigChanged();

  /** @brief Return a factory for creating Display subclasses based on a class id string. */
//  virtual DisplayFactory* getDisplayFactory() const;

//  PropertyTreeModel* getDisplayTreeModel() const;

  /** @brief Emits statusUpdate() signal with the given @a message. */
  void emitStatusUpdate( const QString& message );

  rviz::DisplayGroup* getRootDisplayGroup() const;

signals:
  /**
   * @brief Emitted at most once every 100ms.
   */
  void timeChanged();

  /** @brief Emitted whenever the display configuration changes. */
  void configChanged();

  /** @brief Emitted during file-loading and initialization to indicate progress. */
  void statusUpdate( const QString& message );
};

};
