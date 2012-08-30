namespace rviz
{

class ConfigSequence /NoDefaultCtors/
{
%TypeHeaderCode
#define ROS_PACKAGE_NAME "rviz"
#include <rviz/config_sequence.h>
%End

public:
  /** @brief If not at the end of the sequence, returns the next one
   * and advances the iterator.  If at the end, creates and appends
   * a new item and returns it. */
  rviz::Config makeNext();

  /** @brief Advance the iterator and return the next Config.  If at
   * the end of the sequence, returns an invalid Config. */
  rviz::Config getNext();

  /** @brief Resets the iterator to the start of the sequence. */
  void start();

  /** @brief Returns true if the iterator is not at the end of the
   * sequence, false if it is at the end. */
  bool hasNext();
};

};
