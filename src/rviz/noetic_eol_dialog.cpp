#include <rviz/noetic_eol_dialog.h>

namespace rviz
{
NoeticEOLDialog::NoeticEOLDialog(QWidget* parent) : QMessageBox(parent)
{
  setIcon(QMessageBox::Critical);
  std::stringstream ss;
  ss << "ROS Noetic goes end-of-life 2025-05-31\n\n";
  ss << "Users are encouraged to migrate to ROS 2 as soon as possible!\n";
  ss << "Alternatively, switch to the ROS One distribution\n\n";
  ss << "For more information see: \n";
  ss << "<some URL TBD>\n\n";
  ss << "To disable this dialog set the DISABLE_ROS1_EOL_WARNINGS environment variable.\n";
  setText(QString::fromStdString(ss.str()));
  setWindowTitle("ROS 1 End-of-Life is May 31st, 2025");
  setStandardButtons(QMessageBox::Ok);
}
} // end namespace rviz
