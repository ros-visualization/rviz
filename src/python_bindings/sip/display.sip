namespace rviz
{

class Display: rviz::Property /NoDefaultCtors/
{
%TypeHeaderCode
#define ROS_PACKAGE_NAME "rviz"
#include <rviz/display.h>
%End

public:
  /** @brief Main initialization, called after constructor, before load() or setEnabled(). */
//  void initialize( DisplayContext* context );

  /** @brief Return data appropriate for the given column (0 or 1) and
   * role for this Display.
   */
//  virtual QVariant getViewData( int column, int role ) const;

  /** @brief Return item flags appropriate for the given column (0 or
   * 1) for this Display. */
//  virtual Qt::ItemFlags getViewFlags( int column ) const;

  /** @brief Return the class identifier which was used to create this
   * instance.  This version just returns whatever was set with
   * setClassId(). */
  virtual QString getClassId() const;

  /** @brief Set the class identifier used to create this instance.
   * Typically this will be set by the factory object which created it. */
  virtual void setClassId( const QString& class_id );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  /** @brief Return true if this Display is enabled, false if not. */
  bool isEnabled() const;

  /** @brief Set the fixed frame in this display. */
  void setFixedFrame( const QString& fixed_frame );

  /** @brief Called periodically by the visualization manager.
   * @param wall_dt Wall-clock time, in seconds, since the last time the update list was run through.
   * @param ros_dt ROS time, in seconds, since the last time the update list was run through. */
  virtual void update( float wall_dt, float ros_dt );

  /** @brief Called to tell the display to clear its state */
  virtual void reset();

  /** @brief Show status level and text.
   * @param level One of StatusProperty::Ok, StatusProperty::Warn, or StatusProperty::Error.
   * @param name The name of the child entry to set.
   * @param text Description of the child's state.
   *
   * Every Display has a StatusList to indicate how it is doing.  The
   * StatusList has StatusPropertychildren indicating the status of
   * various subcomponents of the Display.  Each child of the status
   * has a level, a name, and descriptive text.  The top-level
   * StatusList has a level which is set to the worst of all the
   * children's levels.
   */
//  virtual void setStatus( StatusProperty::Level level, const QString& name, const QString& text );

  /** @brief Show status level and text, using a std::string.
   * Convenience function which converts std::string to QString
   * and calls setStatus(). */
//  void setStatusStd( StatusProperty::Level level, const std::string& name, const std::string& text )
//    {
//      setStatus( level, QString::fromStdString( name ), QString::fromStdString( text ));
//    }

  /** @brief Delete the status entry with the given name. */
  virtual void deleteStatus( const QString& name );

  /** @brief Delete the status entry with the given std::string name. */
//  void deleteStatusStd( const std::string& name ) { deleteStatus( QString::fromStdString( name )); }

  void setIcon( const QIcon& icon );

public slots:
  /** @brief Enable or disable this Display.
   *
   * SetEnabled is called after initialize() and at the end of load(),
   * if the Display settings are being loaded from a file. */
  void setEnabled( bool enabled );

  /** @brief Convenience function which calls context_->queueRender(). */
  void queueRender();
};

};
