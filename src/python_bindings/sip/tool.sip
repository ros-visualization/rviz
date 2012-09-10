namespace rviz
{

class Tool /NoDefaultCtors/
{
%TypeHeaderCode
#define ROS_PACKAGE_NAME "rviz"
#include <rviz/tool.h>
%End

public:
  /** Default constructor.  Pluginlib only instantiates classes via
   * default constructors.  Subclasses of Tool should shortcut_key_
   * field in their constructors.
   *
   * Properties to appear in the Tool Properties panel are typically
   * created in the constructor, as children of the property from
   * getPropertyContainer(), which is set up in this Tool
   * constructor. */
//  Tool();
//  virtual ~Tool();

  /** Initialize the tool.  Sets the DisplayContext and calls
   * onInitialize(). */
//  void initialize( DisplayContext* context );

  /** @brief Return the container for properties of this Tool.. */
  virtual rviz::Property* getPropertyContainer();

  char getShortcutKey();

  virtual void activate() = 0;
  virtual void deactivate() = 0;

  virtual void update(float wall_dt, float ros_dt);

  enum {
    Render = 1,
    Finished = 2
  };

  /** Process a mouse event.  This is the central function of all the
   * tools, as it defines how the mouse is used. */
//  virtual int processMouseEvent( ViewportMouseEvent& event ) = 0;

  /** Process a key event.  Override if your tool should handle any
      other keypresses than the tool shortcuts, which are handled
      separately. */
//  virtual int processKeyEvent( QKeyEvent* event, RenderPanel* panel );

  QString getName() const;

  /** @brief Set the name of the tool.
   *
   * This is called by ToolManager during tool initialization.  If you
   * want a different name than it gives you, call this from
   * onInitialize() (or thereafter). */
  void setName( const QString& name );

  /** @brief Set the description of the tool.  This is called by
   * ToolManager during tool initialization. */
  QString getDescription() const;
  void setDescription( const QString& description );

  /** @brief Return the class identifier which was used to create this
   * instance.  This version just returns whatever was set with
   * setClassId(). */
  virtual QString getClassId() const;

  /** @brief Set the class identifier used to create this instance.
   * Typically this will be set by the factory object which created it. */
  virtual void setClassId( const QString& class_id );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  /** @brief Set the toolbar icon for this tool (will also set its cursor). */
  void setIcon( const QIcon& icon );

  /** @brief Get the icon of this tool. */
  const QIcon& getIcon();

  /** @brief Set the cursor for this tool. */
  void setCursor( const QCursor& cursor );

  /** @brief Get current cursor of this tool. */
  const QCursor& getCursor();
};

};
