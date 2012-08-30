namespace rviz
{

class ConfigMapIterator /NoDefaultCtors/
{
%TypeHeaderCode
#define ROS_PACKAGE_NAME "rviz"
#include <rviz/config_map_iterator.h>
%End

public:
  /** @brief Advance iterator to next entry. */
  void next();

  /** @brief Return true if there is a next entry, false if not. */
  bool hasNext();
  
  /** @brief Resets the iterator to the start of the map. */
  void start();

  /** @brief Return the name of the current map entry. */
  QString currentKey();

  /** @brief Return the config reference of the current map entry. */
  rviz::Config currentChild();
};

};
