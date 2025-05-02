#include <rviz/noetic_eol_dialog.h>

namespace rviz
{
NoeticEOLDialog::NoeticEOLDialog(QWidget* parent) : QMessageBox(parent)
{
  setIcon(QMessageBox::Critical);
  setTextFormat(Qt::RichText);
  setText(
      "<p>ROS Noetic goes end-of-life 2025-05-31.</p>"
      "<p>Users are encouraged to migrate to ROS 2 as soon as possible!"
      "Alternatively, switch to the <a href='https://ros.packages.techfak.net'>ROS One "
      "distribution</a>.</p>"
      "<p>For more information see <a href='https://www.ros.org/blog/noetic-eol'>this blog post</a>.</p>"
      "<p>To disable this dialog, set the <code>DISABLE_ROS1_EOL_WARNINGS</code> environment "
      "variable.</p>");
  setWindowTitle("ROS 1 End-of-Life is May 31st, 2025");
  setStandardButtons(QMessageBox::Ok);
}
} // end namespace rviz
