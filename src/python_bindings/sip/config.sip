namespace rviz
{

class Config /NoDefaultCtors/
{
%TypeHeaderCode
#define ROS_PACKAGE_NAME "rviz"
#include <rviz/config.h>
%End

public:
  Config();
  /** @brief Copy constructor.  Copies only the reference to the data, so creates a shallow copy. */
  Config( const rviz::Config& source );

  enum Type { Map, Sequence, Scalar, Invalid };
  rviz::Config::Type getType() const;

  rviz::Config makeChild( const QString& name );
  rviz::Config getChild( const QString& name ) const;
  bool isValid() const;

  /** @brief Ensures this is a valid Config object then sets the value. */
  void setValue( const QVariant& value );

  /** @brief If this config object is valid, this returns its value.  If not, it returns an invalid QVariant. */
  QVariant getValue() const;

  /** @brief Makes this Config element a sequence container and returns a reference to the sequence interface. */
  rviz::ConfigSequence makeSequence();

  /** @brief If this Config element is a sequence container, this
   * returns a reference to the sequence interface.  If it is not,
   * returns an invalid reference. */
  rviz::ConfigSequence getSequence() const;

  /** @brief Return a new iterator for looping over key/value pairs. */
  rviz::ConfigMapIterator mapIterator() const;
};

};
