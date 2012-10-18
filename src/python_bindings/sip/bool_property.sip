namespace rviz
{

class BoolProperty: rviz::Property /NoDefaultCtors/
{
%TypeHeaderCode
#define ROS_PACKAGE_NAME "rviz"
#include <rviz/properties/bool_property.h>
%End

public:
  BoolProperty( const QString& name = QString(),
                bool default_value = false,
                const QString& description = QString(),
                rviz::Property* parent = 0,
                const char *changed_slot = 0,
                QObject* receiver = 0 );

  virtual ~BoolProperty();

  virtual bool getBool() const;

public slots:
  bool setBool( bool value );
};

};
