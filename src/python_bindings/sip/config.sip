namespace rviz
{

class Config /NoDefaultCtors/
{
%TypeHeaderCode
#define ROS_PACKAGE_NAME "rviz"
#include <rviz/config.h>
%End

public:
  /** @brief Default constructor.  Creates an empty config object. */
  Config();
  /** @brief Copy constructor.  Copies only the reference to the data, so creates a shallow copy. */
  Config( const rviz::Config& source );
  /** @brief Convenience constructor, makes a Value type Config object with the given value. */
  Config( QVariant value );

  enum Type { Map, List, Value, Empty, Invalid };
  rviz::Config::Type getType() const;
  void setType( rviz::Config::Type );

  /** @brief Returns getType() != Invalid. */
  bool isValid() const;

  void mapSetValue( const QString& key, QVariant value );
  rviz::Config mapMakeChild( const QString& key );
  rviz::Config mapGetChild( const QString& key ) const;

  /** @brief Return a new iterator for looping over key/value pairs. */
  rviz::Config::MapIterator mapIterator() const;

  /** @brief Ensures this is a valid Config object, sets the type to Value then sets the value. */
  void setValue( const QVariant& value );
  /** @brief If this config object is valid, this returns its value.  If not, it returns an invalid QVariant. */
  QVariant getValue() const;

  int listLength() const;
  rviz::Config listChildAt( int i ) const;
  rviz::Config listAppendNew();

  class MapIterator /NoDefaultCtors/
  {
%TypeHeaderCode
#define ROS_PACKAGE_NAME "rviz"
#include <rviz/config.h>
%End
  public:
    /** @brief Advance iterator to next entry. */
    void advance();

    /** @brief Return true if there is a next entry, false if not. */
    bool isValid();
  
    /** @brief Resets the iterator to the start of the map. */
    void start();

    /** @brief Return the name of the current map entry. */
    QString currentKey();

    /** @brief Return the config reference of the current map entry. */
    rviz::Config currentChild();
  };
};

};
